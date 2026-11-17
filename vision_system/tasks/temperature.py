#!/usr/bin/env python3
# coding: utf-8
"""楼宇异常温度楼层识别任务（总决赛 10 分项）。

与 fire / meter 一致采用抓拍即返回：到点取一帧即放行导航，后台判定。
判定分两级：

1. 标记级。赛题在楼面上放置表示高温的标记（红/橙色块或文字牌）。
   有热成像话题（THERMAL_TOPIC）时优先用辐射值判定；没有热成像时，
   在 HSV 空间找高饱和暖色块，取面积最大且位于画面中部的候选。
2. 文本级。在标记附近裁一块送 OCR，读“2F / 2 层 / 二楼”这类楼层文本；
   读不到就回落到 config.FLOOR_REGIONS 里按路点序号登记的楼层，
   两者都没有时上报 UNKNOWN_FLOOR，宁可不计分也不编造楼层。

输出字段按裁判系统要求组织：building、floor、method、confidence。
"""

import json
import queue
import re
import threading
import time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import config

try:
    from core.judge_client import get_client as get_judge_client
except Exception:
    from vision_system.core.judge_client import get_judge_client


CN_DIGITS = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
             "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}
FLOOR_RE = re.compile(r"(\d{1,2})\s*(?:F|f|层|楼)|([一二两三四五六七八九十]{1,2})\s*(?:层|楼)")


def _lazy_cv():
    try:
        import cv2
        return cv2
    except Exception:
        return None


def _lazy_np():
    try:
        import numpy
        return numpy
    except Exception:
        return None


def parse_floor(text):
    """从 OCR 文本里取楼层号，取不到返回 None。"""
    if not text:
        return None
    match = FLOOR_RE.search(text)
    if match:
        if match.group(1):
            return int(match.group(1))
        cn = match.group(2)
        if cn in CN_DIGITS:
            return CN_DIGITS[cn]
        if len(cn) == 2 and cn[0] == "十" and cn[1] in CN_DIGITS:
            return 10 + CN_DIGITS[cn[1]]
    # 再退一步：整串只有一个 1—20 的数字，且文本很短
    if len(text.strip()) <= 4:
        digits = re.findall(r"\d{1,2}", text)
        if len(digits) == 1:
            value = int(digits[0])
            if 1 <= value <= 20:
                return value
    return None


class TemperatureTask(object):
    """异常温度楼层识别。"""

    def __init__(self):
        self.active = False
        self.done_callback = None
        self.status_callback = None
        self.current_seq_id = 0
        self.frame_hint = {}

        self.ocr_pub = rospy.Publisher("/vision/driver/ocr/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.snapshot_pub = rospy.Publisher("/vision/snapshot", Image, queue_size=1)
        self.ocr_sub = rospy.Subscriber("/vision/driver/ocr/result", String, self.on_ocr_data)
        self.yolo_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        self.yolo_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_yolo_data)
        self.cam_sub = None
        self.thermal_sub = None

        self.image_topic = config.DEFAULT_CONFIG.get("image_topic", "/camera/color/image_raw")
        self.thermal_topic = getattr(config, "THERMAL_TOPIC", "")
        self.latest_thermal = None

        self.task_queue = queue.Queue()
        self.ocr_event = threading.Event()
        self.yolo_event = threading.Event()
        self.current_job = {}
        self._bridge = None

        self.judge = get_judge_client(
            config=getattr(config, "JUDGE_CONFIG", None),
            node_name="vision_master",
            logger=rospy.loginfo)

        if self.thermal_topic:
            try:
                self.thermal_sub = rospy.Subscriber(
                    self.thermal_topic, Image, self.on_thermal_frame, queue_size=1)
            except Exception as exc:
                rospy.logwarn("[TemperatureTask] thermal topic unavailable: %s" % exc)

        self.worker = threading.Thread(target=self.worker_loop, name="temperature-worker")
        self.worker.daemon = True
        self.worker.start()

    # ---------------- 前台 ----------------
    def start(self, done_callback, status_callback=None, seq_id=0):
        self.active = True
        self.done_callback = done_callback
        self.status_callback = status_callback
        self.current_seq_id = int(seq_id or 0)
        self.frame_hint = self._floor_hint(self.current_seq_id)
        self._report_status("capturing")
        rospy.loginfo("[TemperatureTask] waiting for camera frame, seq=%d" % self.current_seq_id)
        self.cam_sub = rospy.Subscriber(self.image_topic, Image, self.on_camera_frame)

    def on_camera_frame(self, msg):
        if not self.active:
            return
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None

        if self.done_callback:
            self.done_callback("done_temperature snapshot_taken")
            self.done_callback = None

        self.task_queue.put((msg, self.current_seq_id, time.time(),
                             self.latest_thermal))
        self.active = False

    def on_thermal_frame(self, msg):
        self.latest_thermal = msg

    def stop(self):
        self.active = False
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None

    # ---------------- 后台 ----------------
    def worker_loop(self):
        while not rospy.is_shutdown():
            try:
                frame_msg, seq_id, captured_at, thermal_msg = self.task_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                self.process(frame_msg, seq_id, captured_at, thermal_msg)
            except Exception as exc:
                rospy.logwarn("[TemperatureTask] process failed: %s" % exc)
                self.judge.report(
                    "temperature",
                    {"building": self.frame_hint.get("building", "D"),
                     "floor": None, "status": "ERROR", "error": str(exc)},
                    text="异常温度识别失败", seq=seq_id)
            finally:
                self.task_queue.task_done()

    def process(self, frame_msg, seq_id, captured_at, thermal_msg):
        cv2 = _lazy_cv()
        np = _lazy_np()
        if self._bridge is None:
            try:
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
            except Exception:
                self._bridge = None

        image = None
        if cv2 is not None and self._bridge is not None:
            try:
                image = self._bridge.imgmsg_to_cv2(frame_msg, "bgr8")
            except Exception as exc:
                rospy.logwarn("[TemperatureTask] cv_bridge failed: %s" % exc)

        building = self.frame_hint.get("building", getattr(config, "TEMPERATURE_BUILDING", "D"))
        method = "none"
        confidence = 0.0
        marker = None
        thermal_peak = None

        # 1) 热成像优先
        if thermal_msg is not None and cv2 is not None and np is not None and self._bridge is not None:
            thermal_peak = self._thermal_peak(np, thermal_msg)
            if thermal_peak is not None:
                method = "thermal"
                confidence = 0.9

        # 2) 可见光暖色标记
        if method == "none" and image is not None:
            marker = self._find_marker(cv2, np, image)
            if marker is not None:
                method = "color_marker"
                confidence = round(marker.get("confidence", 0.5), 3)

        # 3) 楼层文本：优先裁标记附近，其次整帧
        crop_msg = frame_msg
        if marker is not None and cv2 is not None and self._bridge is not None:
            crop = self._crop_around(cv2, image, marker)
            if crop is not None:
                converted = self._to_imgmsg(crop)
                if converted is not None:
                    crop_msg = converted

        self.current_job = {"texts": []}
        self.ocr_event.clear()
        self.snapshot_pub.publish(crop_msg)
        rospy.sleep(0.2)
        self.ocr_pub.publish("snapshot")
        got_ocr = self.ocr_event.wait(timeout=float(getattr(config, "TEMPERATURE_OCR_TIMEOUT", 8.0)))
        texts = list(self.current_job.get("texts") or [])

        floor = None
        floor_source = None
        for text in texts:
            parsed = parse_floor(text)
            if parsed is not None:
                floor = parsed
                floor_source = "ocr:%s" % text.strip()[:16]
                break

        if floor is None:
            hint_floor = self.frame_hint.get("floor")
            if hint_floor is not None:
                floor = int(hint_floor)
                floor_source = "config_floor_region"

        status = "OK" if (floor is not None and (marker is not None or thermal_peak is not None)) else "PARTIAL"
        if floor is None:
            status = "UNKNOWN_FLOOR"
        elif marker is None and thermal_peak is None:
            status = "NO_MARKER"

        elapsed_ms = round((time.time() - captured_at) * 1000.0, 1)
        result = {
            "building": building,
            "floor": floor,
            "method": method,
            "floor_source": floor_source,
            "confidence": confidence,
            "marker": marker,
            "thermal_peak": thermal_peak,
            "ocr_timeout": not got_ocr,
            "elapsed_ms": elapsed_ms,
            "status": status,
        }
        text = self._speech_text(building, floor, status)

        self.judge.report("temperature", result, text=text, seq=seq_id,
                          evidence={"captured_at": round(captured_at, 3),
                                    "ocr_texts": texts[:5]})
        self._report_status("done")
        self._speak(text)
        rospy.loginfo("[TemperatureTask] seq=%d floor=%s method=%s status=%s (%.1f ms)"
                      % (seq_id, floor, method, status, elapsed_ms))

    # ---------------- 检测 ----------------
    def _thermal_peak(self, np, thermal_msg):
        try:
            if self._bridge is None:
                return None
            raw = self._bridge.imgmsg_to_cv2(thermal_msg, desired_encoding="passthrough")
            arr = np.asarray(raw, dtype=np.float32)
            if arr.size == 0:
                return None
            peak = float(arr.max())
            mean = float(arr.mean())
            # 仅当存在明显离群高温时认为检出
            if peak - mean < max(5.0, 0.15 * abs(mean)):
                return None
            idx = int(np.argmax(arr))
            h, w = arr.shape[:2]
            return {"value": round(peak, 3), "mean": round(mean, 3),
                    "uv": [int(idx % w), int(idx // w)]}
        except Exception as exc:
            rospy.logwarn("[TemperatureTask] thermal decode failed: %s" % exc)
            return None

    @staticmethod
    def _find_marker(cv2, np, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masks = [
            cv2.inRange(hsv, (0, 120, 110), (12, 255, 255)),
            cv2.inRange(hsv, (168, 120, 110), (180, 255, 255)),
            cv2.inRange(hsv, (12, 120, 110), (30, 255, 255)),
        ]
        mask = masks[0]
        for extra in masks[1:]:
            mask = cv2.bitwise_or(mask, extra)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        h, w = image.shape[:2]
        frame_area = float(h * w)
        best = None
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < frame_area * 0.0004:      # 太小，多为噪点
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            aspect = bw / float(bh) if bh else 0.0
            if not (0.35 <= aspect <= 3.0):
                continue
            cx, cy = x + bw / 2.0, y + bh / 2.0
            centre_dist = abs(cx - w / 2.0) / max(1.0, w / 2.0)
            score = area / frame_area * (1.0 - 0.4 * centre_dist)
            cand = {
                "bbox": [int(x), int(y), int(x + bw), int(y + bh)],
                "area_ratio": round(area / frame_area, 4),
                "uv": [int(cx), int(cy)],
                "confidence": min(0.95, 0.45 + area / frame_area * 6.0),
                "score": round(score, 5),
            }
            if best is None or cand["score"] > best["score"]:
                best = cand
        return best

    @staticmethod
    def _crop_around(cv2, image, marker, pad=0.6):
        x1, y1, x2, y2 = marker["bbox"]
        h, w = image.shape[:2]
        bw, bh = max(1, x2 - x1), max(1, y2 - y1)
        px, py = int(bw * pad), int(bh * pad)
        X1, Y1 = max(0, x1 - px), max(0, y1 - py)
        X2, Y2 = min(w, x2 + px), min(h, y2 + py)
        if X2 - X1 < 8 or Y2 - Y1 < 8:
            return None
        return image[Y1:Y2, X1:X2].copy()

    # ---------------- 配置与交付 ----------------
    @staticmethod
    def _floor_hint(seq_id):
        regions = getattr(config, "FLOOR_REGIONS", {}) or {}
        hint = regions.get(seq_id) or regions.get(str(seq_id)) or {}
        return dict(hint)

    def _speech_text(self, building, floor, status):
        if floor is None:
            return "%s 号楼未识别到异常温度楼层" % building
        if status == "NO_MARKER":
            return "%s 号楼在 %d 楼附近未检出高温标记" % (building, floor)
        return "%s 号楼在 %d 楼发现高温" % (building, floor)

    def on_ocr_data(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        texts = data.get("texts") or []
        if texts:
            self.current_job["texts"] = texts
            self.ocr_event.set()

    def on_yolo_data(self, msg):
        # 预留：若后续训练了高温标记检测模型，在此接入
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get("model") == "temp":
            self.current_job["detections"] = data.get("boxes") or []
            self.yolo_event.set()

    def _report_status(self, state):
        if self.status_callback:
            try:
                self.status_callback("status_temperature %s | seq=%d"
                                     % (state, self.current_seq_id))
            except Exception:
                pass

    def _speak(self, text):
        if not getattr(config, "VOICE_MIRROR", False):
            return
        try:
            self.voice_pub.publish(json.dumps(
                {"action": "dispatch", "task": "system", "data": text}))
        except Exception:
            pass

    def _to_imgmsg(self, bgr):
        if self._bridge is None:
            try:
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
            except Exception:
                return None
        try:
            return self._bridge.cv2_to_imgmsg(bgr, "bgr8")
        except Exception:
            return None
