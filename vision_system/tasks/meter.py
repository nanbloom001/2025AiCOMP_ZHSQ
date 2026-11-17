#!/usr/bin/env python3
# coding: utf-8
"""站房仪表读取任务（总决赛 14 分项）。

时序与 fire / plate 一致：抓拍即返回。
    收到相机第一帧 -> 立刻回 done_meter snapshot_taken，导航继续
    同一帧进入后台队列 -> 定位表盘 -> 数字表走 OCR，指针表走角度解算
                    -> 结果提交裁判系统 -> 回执写入本地日志

表型由 config.METER_TABLES 按任务序号指定，未登记的序号回落到默认项：
    kind        digital / analog / auto
    unit        单位（MPa、kWh、V、A、℃ 等），参与播报文本
    r0, r_max   量程起止读数
    theta0_deg  量程起点对应的指针方向（度，0 度指向 +x，逆时针为正）
    theta_max_deg
    tolerance   允许偏差，仅用于记录合格性，不作为上报值

cv2 采用延迟导入：vision_master 运行在基础环境，若该环境没有 OpenCV，
任务仍可通过整帧 OCR 工作，只是失去表盘定位与指针解算能力。
"""

import json
import math
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
except Exception:  # 兼容直接以脚本目录为根运行
    from vision_system.core.judge_client import get_judge_client


NUM_RE = re.compile(r"[-+]?\d+(?:[.,]\d+)?")
UNIT_HINT = ["MPa", "kPa", "mPa", "kWh", "kw", "V", "A", "mA", "℃", "°C",
             "m3/h", "m³/h", "L/min", "r/min", "Hz", "bar", "Pa", "%"]


def _lazy_cv():
    """延迟导入 OpenCV；没有就返回 None，调用方回落到纯 OCR 路径。"""
    try:
        import cv2
        return cv2
    except Exception:
        return None


def _lazy_bridge():
    try:
        from cv_bridge import CvBridge
        return CvBridge()
    except Exception:
        return None


class MeterTask(object):
    """仪表读取任务控制器。"""

    def __init__(self):
        self.active = False
        self.done_callback = None
        self.status_callback = None
        self.current_seq_id = 0

        self.ocr_pub = rospy.Publisher("/vision/driver/ocr/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.snapshot_pub = rospy.Publisher("/vision/snapshot", Image, queue_size=1)
        self.ocr_sub = rospy.Subscriber("/vision/driver/ocr/result", String, self.on_ocr_data)
        self.cam_sub = None

        self.image_topic = config.DEFAULT_CONFIG.get("image_topic", "/camera/color/image_raw")

        self.task_queue = queue.Queue()
        self.ocr_event = threading.Event()
        self.current_job = {}
        self._bridge = None

        self.judge = get_judge_client(
            config=getattr(config, "JUDGE_CONFIG", None),
            node_name="vision_master",
            logger=rospy.loginfo)

        self.worker = threading.Thread(target=self.worker_loop, name="meter-worker")
        self.worker.daemon = True
        self.worker.start()

    # ---------------- 前台 ----------------
    def start(self, done_callback, status_callback=None, seq_id=0):
        self.active = True
        self.done_callback = done_callback
        self.status_callback = status_callback
        self.current_seq_id = int(seq_id or 0)
        self._report_status("capturing")
        rospy.loginfo("[MeterTask] waiting for camera frame, seq=%d" % self.current_seq_id)
        self.cam_sub = rospy.Subscriber(self.image_topic, Image, self.on_camera_frame)

    def on_camera_frame(self, msg):
        if not self.active:
            return
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None

        # 先放行导航，再处理图像：这一帧足够做表盘读数，不必等推理
        if self.done_callback:
            self.done_callback("done_meter snapshot_taken")
            self.done_callback = None

        self.task_queue.put((msg, self.current_seq_id, time.time()))
        self.active = False

    def stop(self):
        self.active = False
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None

    # ---------------- 后台 ----------------
    def worker_loop(self):
        while not rospy.is_shutdown():
            try:
                frame_msg, seq_id, captured_at = self.task_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                self.process(frame_msg, seq_id, captured_at)
            except Exception as exc:
                rospy.logwarn("[MeterTask] process failed: %s" % exc)
                self.judge.report(
                    "meter", {"meter_id": seq_id, "status": "ERROR", "error": str(exc)},
                    text="仪表读取失败", seq=seq_id)
            finally:
                self.task_queue.task_done()

    def process(self, frame_msg, seq_id, captured_at):
        table = self._table_config(seq_id)
        cv2 = _lazy_cv()
        if self._bridge is None:
            self._bridge = _lazy_bridge()

        image = None
        if cv2 is not None and self._bridge is not None:
            try:
                image = self._bridge.imgmsg_to_cv2(frame_msg, "bgr8")
            except Exception as exc:
                rospy.logwarn("[MeterTask] cv_bridge failed: %s" % exc)

        roi = None
        dial = None
        angle = None
        if image is not None:
            dial = self._locate_dial(cv2, image)
            if dial is not None:
                roi = self._crop(cv2, image, dial)
                if table.get("kind") in ("analog", "auto"):
                    angle = self._needle_angle(cv2, image, dial)

        # 有 ROI 就把 ROI 送去 OCR；否则整帧送去，靠正则兜底
        payload_msg = frame_msg
        if roi is not None:
            converted = self._to_imgmsg(roi)
            if converted is not None:
                payload_msg = converted
        self.current_job = {"texts": [], "boxes": [], "done": False}
        self.ocr_event.clear()
        self.snapshot_pub.publish(payload_msg)
        rospy.sleep(0.2)
        self.ocr_pub.publish("snapshot")

        got_ocr = self.ocr_event.wait(timeout=float(table.get("ocr_timeout", 8.0)))
        texts = list(self.current_job.get("texts") or [])
        if not got_ocr:
            rospy.logwarn("[MeterTask] OCR timeout, fall back to angle only")

        value, unit, kind, confidence = self._decide(table, texts, angle, dial)
        elapsed_ms = round((time.time() - captured_at) * 1000.0, 1)

        result = {
            "meter_id": seq_id,
            "kind": kind,
            "value": value,
            "unit": unit,
            "confidence": round(confidence, 3),
            "angle_deg": None if angle is None else round(angle, 2),
            "dial": dial,
            "tolerance": table.get("tolerance"),
            "elapsed_ms": elapsed_ms,
            "status": "OK" if value is not None else "NO_READING",
        }
        text = self._speech_text(seq_id, value, unit)

        self.judge.report("meter", result, text=text, seq=seq_id,
                          evidence={"captured_at": round(captured_at, 3),
                                    "ocr_texts": texts[:5]})
        self._report_status("done")
        self._speak(text)
        rospy.loginfo("[MeterTask] seq=%d value=%s %s conf=%.2f (%.1f ms)"
                      % (seq_id, value, unit, confidence, elapsed_ms))

    # ---------------- 视觉算法 ----------------
    @staticmethod
    def _table_config(seq_id):
        tables = getattr(config, "METER_TABLES", {}) or {}
        default = getattr(config, "METER_DEFAULT", {})
        table = dict(default)
        table.update(tables.get(seq_id) or tables.get(str(seq_id)) or {})
        return table

    @staticmethod
    def _locate_dial(cv2, image):
        """霍夫圆找表盘；找不到返回 None。"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        h, w = gray.shape[:2]
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=int(min(h, w) * 0.4),
            param1=120, param2=35,
            minRadius=int(min(h, w) * 0.10), maxRadius=int(min(h, w) * 0.45))
        if circles is None:
            return None
        best = None
        for c in circles[0]:
            cx, cy, r = float(c[0]), float(c[1]), float(c[2])
            centre_dist = math.hypot(cx - w / 2.0, cy - h / 2.0)
            score = r - centre_dist * 0.5
            if best is None or score > best["score"]:
                best = {"cx": round(cx, 1), "cy": round(cy, 1), "r": round(r, 1),
                        "score": round(score, 1)}
        return best

    @staticmethod
    def _crop(cv2, image, dial, margin=1.05):
        h, w = image.shape[:2]
        r = dial["r"] * margin
        x1 = max(0, int(dial["cx"] - r))
        y1 = max(0, int(dial["cy"] - r))
        x2 = min(w, int(dial["cx"] + r))
        y2 = min(h, int(dial["cy"] + r))
        if x2 - x1 < 8 or y2 - y1 < 8:
            return None
        return image[y1:y2, x1:x2].copy()

    @staticmethod
    def _needle_angle(cv2, image, dial, samples=360):
        """沿半径方向采样，取平均亮度最低的方向作为指针方向（度）。"""
        cx, cy, r = dial["cx"], dial["cy"], dial["r"]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        best_angle, best_score = None, None
        second = None
        for deg in range(samples):
            theta = math.radians(deg)
            dark = 0.0
            total = 0
            for frac in (0.35, 0.45, 0.55, 0.65, 0.75, 0.85):
                x = int(cx + math.cos(theta) * r * frac)
                y = int(cy - math.sin(theta) * r * frac)
                if 0 <= x < w and 0 <= y < h:
                    dark += 255.0 - float(gray[y, x])
                    total += 1
            if total == 0:
                continue
            score = dark / total
            if best_score is None or score > best_score:
                second = best_score
                best_score, best_angle = score, deg
            elif second is None or score > second:
                second = score
        if best_angle is None or second is None:
            return None
        # 指针与背景对比不足时视为不可读，避免输出随机角度
        if best_score - second < 12.0:
            return None
        return float(best_angle)

    @staticmethod
    def _map_angle(table, angle_deg):
        theta0 = float(table.get("theta0_deg", -135.0))
        theta_max = float(table.get("theta_max_deg", 135.0))
        r0 = float(table.get("r0", 0.0))
        r_max = float(table.get("r_max", 1.0))
        if abs(theta_max - theta0) < 1e-6:
            return None
        # 归一到 0—360 后按量程方向展开，处理跨 0 度的情况
        delta = (angle_deg - theta0) % 360.0
        span = (theta_max - theta0) % 360.0
        if span <= 0:
            span += 360.0
        frac = delta / span
        frac = max(0.0, min(1.0, frac))
        return r0 + frac * (r_max - r0)

    @staticmethod
    def _parse_digital(texts, table):
        unit = table.get("unit") or ""
        best = None
        for raw in texts:
            text = (raw or "").strip()
            if not text:
                continue
            digits = NUM_RE.findall(text)
            if not digits:
                continue
            value = float(digits[0].replace(",", "."))
            score = 0.4
            if unit and unit.lower() in text.lower():
                score += 0.4
            if any(h.lower() in text.lower() for h in UNIT_HINT):
                score += 0.1
            if best is None or score > best[2]:
                best = (value, unit, score)
        return best

    def _decide(self, table, texts, angle, dial):
        kind = table.get("kind", "auto")
        unit = table.get("unit", "")

        digital = self._parse_digital(texts, table)
        analog_value = None
        if angle is not None:
            analog_value = self._map_angle(table, angle)

        if kind == "digital":
            if digital:
                return digital[0], digital[1] or unit, "digital", digital[2]
            return None, unit, "digital", 0.0

        if kind == "analog":
            if analog_value is not None:
                conf = 0.75 if dial else 0.5
                return round(analog_value, 3), unit, "analog", conf
            if digital:
                return digital[0], digital[1] or unit, "digital", digital[2] * 0.8
            return None, unit, "analog", 0.0

        # auto：数字优先（OCR 带单位时更可信），否则用角度
        if digital and digital[2] >= 0.6:
            return digital[0], digital[1] or unit, "digital", digital[2]
        if analog_value is not None:
            conf = 0.75 if dial else 0.5
            return round(analog_value, 3), unit, "analog", conf
        if digital:
            return digital[0], digital[1] or unit, "digital", digital[2]
        return None, unit, "unknown", 0.0

    @staticmethod
    def _speech_text(seq_id, value, unit):
        if value is None:
            return "%d 号表未读到有效读数" % seq_id
        if isinstance(value, float) and abs(value - round(value)) < 1e-6:
            shown = "%d" % int(round(value))
        else:
            shown = ("%.3f" % value).rstrip("0").rstrip(".")
        return "%d 号表读数为 %s %s" % (seq_id, shown, unit)

    # ---------------- 话题与工具 ----------------
    def on_ocr_data(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        texts = data.get("texts") or []
        if texts:
            self.current_job["texts"] = texts
            self.current_job["boxes"] = data.get("boxes") or []
            self.ocr_event.set()

    def _report_status(self, state):
        if self.status_callback:
            try:
                self.status_callback("status_meter %s | seq=%d"
                                     % (state, self.current_seq_id))
            except Exception:
                pass

    def _speak(self, text):
        """语音保留为可选通道；裁判系统为主通道，播报失败不影响上报。"""
        if not getattr(config, "VOICE_MIRROR", False):
            return
        try:
            self.voice_pub.publish(json.dumps(
                {"action": "dispatch", "task": "system", "data": text}))
        except Exception:
            pass

    def _to_imgmsg(self, bgr):
        if self._bridge is None:
            self._bridge = _lazy_bridge()
        if self._bridge is None:
            return None
        try:
            return self._bridge.cv2_to_imgmsg(bgr, "bgr8")
        except Exception:
            return None
