import rospy
import json
import time
import threading
import re
import math
from std_msgs.msg import String

class FireTask:
    """
    火焰检测任务控制器。
    流程：检测火焰 -> 识别文字 -> 报告结果
    """
    def __init__(self):
        self.active = False
        # Publishers
        self.yolo_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        self.ocr_pub = rospy.Publisher("/vision/driver/ocr/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        
        # Subscribers
        self.yolo_sub = None
        self.ocr_sub = None
        self.voice_status_sub = None
        
        self.result_callback_func = None 
        self.state = "IDLE" # IDLE, FIND_FIRE, READ_TEXT
        self.fire_info = {}
        self.receipt_event = threading.Event()
        
        # Timers
        self.yolo_timer = None
        self.ocr_timer = None

    def start(self, callback_func, status_callback=None, seq_id=0):
        """任务开始"""
        self.active = True
        self.result_callback_func = callback_func
        self.state = "FIND_FIRE"
        self.fire_info = {"count": 0, "text": "未知区域"}
        self.receipt_event.clear()
        
        self.start_time = time.time()
        self.yolo_done_time = 0
        self.ocr_done_time = 0
        self.yolo_timing = {}
        self.ocr_timing = {}
        
        # 1. 订阅 YOLO 结果
        self.yolo_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_yolo_data)
        
        # 2. 订阅语音回执
        if self.voice_status_sub:
            self.voice_status_sub.unregister()
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        
        # 3. 开启 YOLO (fire)
        rospy.loginfo("[FireTask] Step 1: Start YOLO (fire)...")
        self.yolo_pub.publish("start fire")
        
        # 4. 开启 YOLO 超时计时器 (5秒)
        self.yolo_timer = threading.Timer(5.0, self.on_yolo_timeout)
        self.yolo_timer.start()

    def stop(self):
        """任务强制停止"""
        self.active = False
        self.state = "IDLE"
        
        # 取消计时器
        if self.yolo_timer:
            self.yolo_timer.cancel()
            self.yolo_timer = None
        if self.ocr_timer:
            self.ocr_timer.cancel()
            self.ocr_timer = None
        
        # 关闭所有工具
        self.yolo_pub.publish("stop")
        self.ocr_pub.publish("stop")
        
        if self.yolo_sub:
            self.yolo_sub.unregister()
            self.yolo_sub = None
        if self.ocr_sub:
            self.ocr_sub.unregister()
            self.ocr_sub = None
        if self.voice_status_sub:
            self.voice_status_sub.unregister()
            self.voice_status_sub = None

    def voice_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("status") == "received" and data.get("original_task") == "fire":
                self.receipt_event.set()
        except:
            pass

    def nms(self, boxes, scores, iou_threshold=0.5):
        if not boxes: return [], []
        # boxes: [[x1,y1,x2,y2], ...]
        # scores: [s1, ...]
        
        # Sort
        indices = sorted(range(len(scores)), key=lambda i: scores[i], reverse=True)
        keep = []
        
        while indices:
            current = indices.pop(0)
            keep.append(current)
            remaining = []
            for i in indices:
                iou = self.calculate_iou(boxes[current], boxes[i])
                if iou < iou_threshold:
                    remaining.append(i)
                else:
                    rospy.loginfo(f"[FireTask] NMS suppressed box with IoU={iou:.2f} (Threshold={iou_threshold})")
            indices = remaining
            
        return [boxes[i] for i in keep], [scores[i] for i in keep]

    def calculate_iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        
        inter_area = max(0, x2 - x1) * max(0, y2 - y1)
        box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
        box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
        
        union_area = box1_area + box2_area - inter_area
        return inter_area / union_area if union_area > 0 else 0

    def on_yolo_data(self, msg):
        if not self.active or self.state != "FIND_FIRE": return

        try:
            data = json.loads(msg.data)
            if data.get("model") != "fire": return
                
            boxes = data.get("boxes", [])
            scores = data.get("scores", [])
            self.yolo_timing = data.get("timing", {})
            
            if len(boxes) > 0:
                self.yolo_done_time = time.time()
                # 取消 YOLO 超时
                if self.yolo_timer:
                    self.yolo_timer.cancel()
                    self.yolo_timer = None

                # 立即停止 YOLO
                self.yolo_pub.publish("stop")
                if self.yolo_sub:
                    self.yolo_sub.unregister()
                    self.yolo_sub = None

                # Manual NMS
                nms_boxes, nms_scores = self.nms(boxes, scores)
                count = len(nms_boxes)
                
                rospy.loginfo(f"[FireTask] Fire detected! Raw: {len(boxes)}, NMS: {count}. Switching to OCR...")
                self.fire_info["count"] = count
                
                self.switch_to_ocr()
                    
        except json.JSONDecodeError:
            pass

    def on_yolo_timeout(self):
        """YOLO 超时处理"""
        if not self.active or self.state != "FIND_FIRE": return
        
        self.yolo_done_time = time.time()
        rospy.logwarn("[FireTask] YOLO timeout. No fire detected. Switching to OCR...")
        
        # 停止 YOLO
        self.yolo_pub.publish("stop")
        if self.yolo_sub:
            self.yolo_sub.unregister()
            self.yolo_sub = None
            
        self.switch_to_ocr()

    def switch_to_ocr(self):
        """切换到 OCR 阶段"""
        self.state = "READ_TEXT"
        
        # 启动 OCR
        self.ocr_sub = rospy.Subscriber("/vision/driver/ocr/result", String, self.on_ocr_data)
        self.ocr_pub.publish("start")
        
        # 启动 OCR 超时计时器
        self.ocr_timer = threading.Timer(3.0, self.on_ocr_timeout)
        self.ocr_timer.start()

    def on_ocr_timeout(self):
        """OCR 超时处理"""
        if not self.active or self.state != "READ_TEXT": return
        
        rospy.logwarn("[FireTask] OCR timeout. Broadcasting result.")
        
        # 停止 OCR
        self.ocr_pub.publish("stop")
        if self.ocr_sub:
            self.ocr_sub.unregister()
            self.ocr_sub = None
            
        # 播报结果
        self.broadcast_result(text="未知区域")

    def fuzzy_correct_text(self, text):
        """
        模糊匹配与纠错机制。
        针对 OCR 容易将 '大厦' 识别为 '大原', '大屋' 等情况进行修正。
        """
        # 1. 包含 "大厦" (Exact match)
        if "大厦" in text:
            return text
            
        # 2. 针对 "大厦" 的模糊匹配
        # 常见误识字: 原, 屋, 夏, 复, 厂, 广, 庆, 厌, 厍
        # 匹配规则: (任意前缀) + 大 + (误识字) + (任意后缀)
        suspicious_chars = "原屋夏复厂广庆厌厍"
        pattern = re.compile(f'(.*)大[{suspicious_chars}](.*)')
        
        match = pattern.search(text)
        if match:
            corrected = match.group(1) + "大厦" + match.group(2)
            rospy.loginfo(f"[FireTask] Fuzzy correction: '{text}' -> '{corrected}'")
            return corrected
            
        return None

    def on_ocr_data(self, msg):
        if not self.active or self.state != "READ_TEXT": return

        try:
            data = json.loads(msg.data)
            texts = data.get("texts", [])
            boxes = data.get("boxes", [])
            self.ocr_timing = data.get("timing", {})
            
            # 取消 OCR 超时
            if self.ocr_timer:
                self.ocr_timer.cancel()
                self.ocr_timer = None
            
            # 只要收到数据（哪怕是空的），就停止 OCR 并处理
            # 立即停止 OCR
            self.ocr_pub.publish("stop")
            if self.ocr_sub:
                self.ocr_sub.unregister()
                self.ocr_sub = None

            final_text = "未知区域"
            if texts:
                # 过滤与择优逻辑
                candidates = []
                
                for i, text in enumerate(texts):
                    # 使用模糊匹配纠错
                    corrected = self.fuzzy_correct_text(text)
                    if corrected:
                        dist = float('inf')
                        # 计算距离中心点的距离
                        if i < len(boxes):
                            box = boxes[i]
                            # 假设 box 是点列表 [[x,y], ...]
                            if box:
                                try:
                                    xs = [p[0] for p in box]
                                    ys = [p[1] for p in box]
                                    cx = sum(xs) / len(xs)
                                    cy = sum(ys) / len(ys)
                                    # 假设图像分辨率 640x480
                                    dist = math.hypot(cx - 320, cy - 240)
                                except Exception:
                                    pass
                        candidates.append((corrected, dist))
                
                if candidates:
                    # 按距离排序，取最近的
                    candidates.sort(key=lambda x: x[1])
                    final_text = candidates[0][0]
                    rospy.loginfo(f"[FireTask] Selected text: {final_text} (Dist: {candidates[0][1]:.1f})")
                else:
                    rospy.loginfo(f"[FireTask] No valid '大厦' text found in: {texts}")
            else:
                rospy.loginfo("[FireTask] OCR returned empty result.")

            self.fire_info["text"] = final_text
            self.broadcast_result(text=final_text)
                
        except json.JSONDecodeError:
            pass

    def broadcast_result(self, text):
        """发送语音播报并结束任务"""
        self.fire_info["text"] = text
        count = self.fire_info.get("count", 0)
        
        # 使用 VoiceTaskDispatcher 格式的数据
        cmd = {
            "action": "dispatch",
            "task": "fire",
            "data": {
                "text": text,
                "count": count
            }
        }
        self.voice_pub.publish(json.dumps(cmd))
        rospy.loginfo(f"[FireTask] Voice command sent: {text} count={count}")

        # 启动异步线程等待回执，然后结束任务
        threading.Thread(target=self.wait_for_receipt_and_finish).start()

    def wait_for_receipt_and_finish(self):
        rospy.loginfo("[FireTask] Waiting for voice receipt...")
        if self.receipt_event.wait(timeout=2.0):
            rospy.loginfo("[FireTask] Receipt confirmed.")
        else:
            rospy.logwarn("[FireTask] Receipt timeout, proceeding anyway.")
            
        self.finish_task()

    def finish_task(self):
        # 先保存结果，因为 stop() 会清空状态
        count = self.fire_info.get("count", 0)
        text = self.fire_info.get("text", "unknown")
        
        # Calculate timings
        end_time = time.time()
        # Use milliseconds for all printed timings
        wait_yolo_ms = round((self.yolo_done_time - self.start_time) * 1000, 2) if self.yolo_done_time > 0 else 0
        total_ctrl_ms = round((end_time - self.start_time) * 1000, 2)

        ctrl_timing = {
            "wait_yolo_ms": wait_yolo_ms,
            "yolo_algo_ms": self.yolo_timing,
            "ocr_algo_ms": self.ocr_timing,
            "total_ctrl_ms": total_ctrl_ms
        }
        
        # 报告给 Master
        # Log with units
        rospy.loginfo(f"[FireTask] Timing (ms): {json.dumps(ctrl_timing)}")
        if self.result_callback_func:
            self.result_callback_func(f"done_fire count={count} text={text} | timing={json.dumps(ctrl_timing)}")
            
        self.stop()
