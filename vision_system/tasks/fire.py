import rospy
import json
import time
import threading
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
        
        self.result_callback_func = None 
        self.state = "IDLE" # IDLE, FIND_FIRE, READ_TEXT
        self.fire_info = {}

    def start(self, callback_func, status_callback=None, seq_id=0):
        """任务开始"""
        self.active = True
        self.result_callback_func = callback_func
        self.state = "FIND_FIRE"
        self.fire_info = {}
        
        # 1. 订阅 YOLO 结果
        self.yolo_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_yolo_data)
        
        # 2. 开启 YOLO (fire)
        rospy.loginfo("[FireTask] Step 1: Start YOLO (fire)...")
        self.yolo_pub.publish("start fire")

    def stop(self):
        """任务强制停止"""
        self.active = False
        self.state = "IDLE"
        
        # 关闭所有工具
        self.yolo_pub.publish("stop")
        self.ocr_pub.publish("stop")
        
        if self.yolo_sub:
            self.yolo_sub.unregister()
            self.yolo_sub = None
        if self.ocr_sub:
            self.ocr_sub.unregister()
            self.ocr_sub = None

    def on_yolo_data(self, msg):
        if not self.active or self.state != "FIND_FIRE": return

        try:
            data = json.loads(msg.data)
            if data.get("model") != "fire": return
                
            boxes = data.get("boxes", [])
            if len(boxes) > 0:
                count = len(boxes)
                rospy.loginfo(f"[FireTask] Fire detected! Count: {count}. Switching to OCR...")
                self.fire_info["count"] = count
                
                # 切换到 OCR 阶段
                self.state = "READ_TEXT"
                
                # 停止 YOLO
                self.yolo_pub.publish("stop")
                if self.yolo_sub:
                    self.yolo_sub.unregister()
                    self.yolo_sub = None
                
                # 启动 OCR
                self.ocr_sub = rospy.Subscriber("/vision/driver/ocr/result", String, self.on_ocr_data)
                self.ocr_pub.publish("start")
                    
        except json.JSONDecodeError:
            pass

    def on_ocr_data(self, msg):
        if not self.active or self.state != "READ_TEXT": return

        try:
            data = json.loads(msg.data)
            texts = data.get("texts", [])
            
            if texts:
                text_str = "".join(texts)
                rospy.loginfo(f"[FireTask] Text detected: {text_str}")
                
                self.fire_info["text"] = text_str
                
                # Voice Broadcast
                count = self.fire_info.get("count", 0)
                voice_msg = f"{text_str}发现火灾隐患{count}个"
                
                cmd = {
                    "action": "say",
                    "text": voice_msg,
                    "id": "fire_result",
                    "speed": 1.1
                }
                self.voice_pub.publish(json.dumps(cmd))
                
                # Stop OCR immediately
                if self.ocr_sub:
                    self.ocr_sub.unregister()
                    self.ocr_sub = None

                # Finish with delay
                threading.Thread(target=self.finish_sequence).start()
                
        except json.JSONDecodeError:
            pass

    def finish_sequence(self):
        time.sleep(1.0)
        self.finish_task()

    def finish_task(self):
        self.stop()
        if self.result_callback_func:
            count = self.fire_info.get("count", 0)
            text = self.fire_info.get("text", "unknown")
            # 格式: done_fire count=X text=Y
            self.result_callback_func(f"done_fire count={count} text={text}")
