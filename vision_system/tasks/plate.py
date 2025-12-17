import rospy
import json
import re
import math
import threading
import queue
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
import config

class PlateTask:
    """
    车牌识别任务控制器 (Snapshot Mode + Background Queue).
    """
    def __init__(self):
        self.active = False
        self.driver_pub = rospy.Publisher("/vision/driver/ocr/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.snapshot_pub = rospy.Publisher("/vision/snapshot", Image, queue_size=1)
        
        self.driver_sub = rospy.Subscriber("/vision/driver/ocr/result", String, self.on_driver_data)
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        self.cam_sub = None
        
        self.image_topic = config.DEFAULT_CONFIG["image_topic"]
        
        # Background Queue
        self.task_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        self.ocr_event = threading.Event()
        self.voice_event = threading.Event()
        self.current_job_data = {}

    def start(self, callback_func, status_callback=None, seq_id=0):
        """任务开始"""
        self.active = True
        self.result_callback_func = callback_func
        self.current_seq_id = seq_id
        
        rospy.loginfo("[PlateTask] Waiting for camera frame...")
        self.cam_sub = rospy.Subscriber(self.image_topic, Image, self.on_camera_frame)

    def on_camera_frame(self, msg):
        if not self.active: return
        
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None
            
        rospy.loginfo("[PlateTask] Frame captured.")
        print("\n[Foreground] Plate Task Finished. Snapshot taken.\n")
        
        if self.result_callback_func:
            self.result_callback_func("done_plate snapshot_taken")
            self.result_callback_func = None
            
        # Put into queue with seq_id
        self.task_queue.put((msg, self.current_seq_id))

    def stop(self):
        self.active = False
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None

    def worker_loop(self):
        while not rospy.is_shutdown():
            try:
                item = self.task_queue.get(timeout=1.0)
                frame_msg, seq_id = item
            except queue.Empty:
                continue
                
            rospy.loginfo(f"[PlateTask] Background processing started for seq {seq_id}.")
            
            self.current_job_data = {"plate": None}
            self.ocr_event.clear()
            self.voice_event.clear()
            
            # Publish Snapshot & Trigger OCR
            self.snapshot_pub.publish(frame_msg)
            rospy.sleep(0.2)
            self.driver_pub.publish("snapshot")
            
            # Wait for OCR
            if self.ocr_event.wait(timeout=5.0):
                rospy.loginfo(f"[PlateTask] OCR finished. Plate: {self.current_job_data['plate']}")
            else:
                rospy.logwarn("[PlateTask] OCR timeout.")
                
            # Voice Broadcast
            if self.current_job_data['plate']:
                best_plate = self.current_job_data['plate']
                plate_chn = best_plate[0]
                plate_eng = best_plate[1:]
                parking_id = seq_id if seq_id > 0 else 1
                
                cmd = {
                    "action": "dispatch",
                    "task": "car_plate",
                    "data": {
                        "parking_id": parking_id,
                        "plate_chn": plate_chn,
                        "plate_eng": plate_eng
                    }
                }
                self.voice_pub.publish(json.dumps(cmd))
                
                if self.voice_event.wait(timeout=5.0):
                    rospy.loginfo("[PlateTask] Voice receipt confirmed.")
                else:
                    rospy.logwarn("[PlateTask] Voice receipt timeout.")
            else:
                rospy.logwarn("[PlateTask] No valid plate to broadcast.")
                
            print("\n[Background] Plate Task Finished. Voice done.\n")
            self.task_queue.task_done()

    def on_driver_data(self, msg):
        try:
            data = json.loads(msg.data)
            texts = data.get("texts", [])
            boxes = data.get("boxes", [])
            img_w = data.get("width", 640)
            img_h = data.get("height", 480)
            
            if not texts: return
            
            candidates = []
            pattern = re.compile(r'^[\u4e00-\u9fa5][A-Z][A-Z0-9]{5,6}$')
            
            for i, text in enumerate(texts):
                clean_text = re.sub(r'[^\u4e00-\u9fa5a-zA-Z0-9]', '', text).upper()
                if pattern.match(clean_text):
                    box = boxes[i] if i < len(boxes) else []
                    dist = float('inf')
                    if box:
                        xs = [p[0] for p in box]
                        ys = [p[1] for p in box]
                        if xs and ys:
                            cx = sum(xs) / len(xs)
                            cy = sum(ys) / len(ys)
                            dist = math.hypot(cx - img_w/2, cy - img_h/2)
                    candidates.append((clean_text, dist))
            
            if candidates:
                candidates.sort(key=lambda x: x[1])
                best_plate = candidates[0][0]
                self.current_job_data["plate"] = best_plate
                self.ocr_event.set()
                    
        except json.JSONDecodeError:
            pass

    def voice_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("status") == "received" and data.get("original_task") == "car_plate":
                self.voice_event.set()
        except:
            pass

 