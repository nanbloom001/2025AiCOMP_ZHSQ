import rospy
import json
import time
import threading
import queue
from std_msgs.msg import String
from sensor_msgs.msg import Image
import config

class GarbageController:
    """
    垃圾分类任务控制器 (Snapshot Mode + Background Queue).
    流程：截图 -> 返回Done -> 入队 -> 后台(YOLO检测+分类 -> 语音播报)
    """
    def __init__(self):
        self.active = False
        # Publishers
        self.driver_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.snapshot_pub = rospy.Publisher("/vision/snapshot", Image, queue_size=1)
        
        # Subscribers
        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_driver_data)
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        self.cam_sub = None
        
        self.image_topic = config.DEFAULT_CONFIG["image_topic"]
        
        # Background Queue
        self.task_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        # Sync Events
        self.yolo_event = threading.Event()
        self.voice_event = threading.Event()
        self.current_job_data = {}

    def start(self, done_callback, status_callback, seq_id=0):
        """任务开始"""
        self.active = True
        self.done_callback = done_callback
        self.current_seq_id = seq_id
        
        rospy.loginfo("[GarbageController] Waiting for camera frame...")
        self.cam_sub = rospy.Subscriber(self.image_topic, Image, self.on_camera_frame)

    def on_camera_frame(self, msg):
        if not self.active: return
        
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None
            
        rospy.loginfo("[GarbageController] Frame captured.")
        print("\n[Foreground] Garbage Task Finished. Snapshot taken.\n")
        
        if self.done_callback:
            self.done_callback("done_garbage snapshot_taken")
            self.done_callback = None
            
        self.task_queue.put((msg, self.current_seq_id))

    def stop(self):
        self.active = False
        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None

    def worker_loop(self):
        while not rospy.is_shutdown():
            try:
                item = self.task_queue.get(timeout=2.0)
                frame_msg, seq_id = item
            except queue.Empty:
                continue
                
            rospy.loginfo(f"[GarbageController] Background processing started for seq {seq_id}.")
            
            self.yolo_event.clear()
            self.voice_event.clear()
            
            # Publish Snapshot & Trigger YOLO
            self.snapshot_pub.publish(frame_msg)
            rospy.sleep(0.2)
            self.driver_pub.publish("snapshot garbage")
            
            # Wait for YOLO (GarbageTask in YOLO node does the processing and sends voice cmd)
            if self.yolo_event.wait(timeout=15.0):
                rospy.loginfo("[GarbageController] YOLO processing finished.")
            else:
                rospy.logwarn("[GarbageController] YOLO timeout.")
                
            # Wait for Voice Receipt (GarbageTask sends the voice command)
            if self.voice_event.wait(timeout=10.0):
                rospy.loginfo("[GarbageController] Voice receipt confirmed.")
            else:
                rospy.logwarn("[GarbageController] Voice receipt timeout.")
                
            print("\n[Background] Garbage Task Finished. Voice done.\n")
            self.task_queue.task_done()

    def on_driver_data(self, msg):
        data = msg.data
        # GarbageTask returns "done:..." or "voice:..."
        if data.startswith("voice:") or data.startswith("done:"):
            self.yolo_event.set()

    def voice_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("status") == "received" and data.get("original_task") == "trash_bin":
                self.voice_event.set()
        except:
            pass

