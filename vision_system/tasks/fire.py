import rospy
import json
import time
import threading
import queue
import re
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
import config

class FireTask:
    """
    火焰检测任务控制器 (Snapshot Mode + Background Queue).
    流程：截图 -> 返回Done -> 入队 -> 后台(YOLO检测火焰 -> OCR识别文字 -> 报告结果)
    """
    def __init__(self):
        self.active = False
        # Publishers
        self.yolo_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        self.ocr_pub = rospy.Publisher("/vision/driver/ocr/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.snapshot_pub = rospy.Publisher("/vision/snapshot", Image, queue_size=1)
        
        # Subscribers
        self.yolo_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_yolo_data)
        self.ocr_sub = rospy.Subscriber("/vision/driver/ocr/result", String, self.on_ocr_data)
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        self.cam_sub = None
        
        self.image_topic = config.DEFAULT_CONFIG["image_topic"]
        
        # Background Processing Queue
        self.task_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        # Synchronization Events for Worker
        self.yolo_event = threading.Event()
        self.ocr_event = threading.Event()
        self.voice_event = threading.Event()
        
        # Data containers for current processing job
        self.current_job_data = {}

    def start(self, callback_func, status_callback=None, seq_id=0):
        """任务开始: 仅负责截图和入队"""
        self.active = True
        self.result_callback_func = callback_func
        
        rospy.loginfo("[FireTask] Waiting for camera frame...")
        # 订阅摄像头获取一帧
        self.cam_sub = rospy.Subscriber(self.image_topic, Image, self.on_camera_frame)

    def on_camera_frame(self, msg):
        """获取到一帧图像后的处理"""
        if not self.active: return
        
        # 1. 停止订阅摄像头
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None
            
        rospy.loginfo("[FireTask] Frame captured.")
        print("\n[Foreground] Fire Task Finished. Snapshot taken.\n")
        
        # 2. 立即返回前台任务完成
        if self.result_callback_func:
            self.result_callback_func("done_fire snapshot_taken")
            self.result_callback_func = None
            
        # 3. 将任务放入后台队列
        self.task_queue.put(msg)

    def stop(self):
        """任务强制停止"""
        self.active = False
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None
        # 注意：不停止后台线程，只停止前台采集

    def worker_loop(self):
        """后台处理线程"""
        while not rospy.is_shutdown():
            try:
                # 获取任务 (阻塞等待)
                frame_msg = self.task_queue.get(timeout=1.0)
            except queue.Empty:
                continue
                
            rospy.loginfo("[FireTask] Background processing started.")
            
            # --- Step 1: Initialize Job State ---
            self.current_job_data = {
                "fire_count": 0,
                "text": "未知区域",
                "yolo_done": False,
                "ocr_done": False
            }
            self.yolo_event.clear()
            self.ocr_event.clear()
            self.voice_event.clear()
            
            # --- Step 2: Publish Snapshot & Trigger YOLO ---
            self.snapshot_pub.publish(frame_msg)
            rospy.sleep(0.2) # 给驱动一点时间接收图片
            self.yolo_pub.publish("snapshot fire")
            
            # Wait for YOLO
            if self.yolo_event.wait(timeout=10.0):
                rospy.loginfo(f"[FireTask] YOLO finished. Count: {self.current_job_data['fire_count']}")
            else:
                rospy.logwarn("[FireTask] YOLO timeout.")
                
            # --- Step 3: Trigger OCR ---
            # 无论 YOLO 结果如何，都尝试 OCR (或者根据逻辑优化)
            if self.current_job_data['fire_count'] > 0:
                self.ocr_pub.publish("snapshot")
                if self.ocr_event.wait(timeout=15.0):
                    rospy.loginfo(f"[FireTask] OCR finished. Text: {self.current_job_data['text']}")
                else:
                    rospy.logwarn("[FireTask] OCR timeout.")
            else:
                rospy.loginfo("[FireTask] No fire detected, skipping OCR.")
            
            # --- Step 4: Voice Broadcast ---
            text = self.current_job_data["text"]
            count = self.current_job_data["fire_count"]
            
            cmd = {
                "action": "dispatch",
                "task": "fire",
                "data": {
                    "text": text,
                    "count": count
                }
            }
            self.voice_pub.publish(json.dumps(cmd))
            
            # Wait for Voice Receipt
            if self.voice_event.wait(timeout=5.0):
                rospy.loginfo("[FireTask] Voice receipt confirmed.")
            else:
                rospy.logwarn("[FireTask] Voice receipt timeout.")
                
            print("\n[Background] Fire Task Finished. Voice done.\n")
            self.task_queue.task_done()

    def on_yolo_data(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("model") != "fire": return
            
            boxes = data.get("boxes", [])
            scores = data.get("scores", [])
            
            # Simple NMS or just count
            # 这里简化处理，假设驱动已经做过 NMS 或者直接用数量
            count = len(boxes)
            
            self.current_job_data["fire_count"] = count
            self.current_job_data["yolo_done"] = True
            self.yolo_event.set()
        except:
            pass

    def on_ocr_data(self, msg):
        try:
            data = json.loads(msg.data)
            texts = data.get("texts", [])
            
            final_text = "未知区域"
            if texts:
                # 模糊匹配逻辑
                for text in texts:
                    corrected = self.fuzzy_correct_text(text)
                    if corrected:
                        final_text = corrected
                        break
                else:
                    final_text = texts[0]
            
            self.current_job_data["text"] = final_text
            self.current_job_data["ocr_done"] = True
            self.ocr_event.set()
        except:
            pass

    def voice_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("status") == "received" and data.get("original_task") == "fire":
                self.voice_event.set()
        except:
            pass

    def fuzzy_correct_text(self, text):
        if "大厦" in text: return text
        suspicious_chars = "原屋夏复厂广庆厌厍"
        pattern = re.compile(f'(.*)大[{suspicious_chars}](.*)')
        match = pattern.search(text)
        if match:
            return match.group(1) + "大厦" + match.group(2)
        return None

