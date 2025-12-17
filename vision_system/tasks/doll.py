import rospy
import json
import threading
import queue
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from collections import Counter
import config

class DollTask:
    """
    玩偶检测任务控制器 (Snapshot Mode + Background Queue).
    """
    def __init__(self):
        self.active = False
        self.driver_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.snapshot_pub = rospy.Publisher("/vision/snapshot", Image, queue_size=1)
        
        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_driver_data)
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        self.cam_sub = None
        
        self.image_topic = config.DEFAULT_CONFIG["image_topic"]
        
        # Background Queue
        self.task_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self.worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        self.yolo_event = threading.Event()
        self.voice_event = threading.Event()
        self.current_job_data = {}
        
        # Global State for Doll Task (Accumulated across snapshots)
        self.block_a_results = set()
        self.block_b_results = set()
        self.target_classes = ['bad0', 'bad1', 'bad2', 'bad3', 'coffee', 'cook', 'dective', 'doctor', 'engineer', 'fireman', 'gardener', 'guitar', 'it', 'office', 'painter', 'photo', 'postman', 'professor', 'rapper', 'security', 'teacher']

    def start(self, done_callback, status_callback, seq_id=0):
        """任务开始"""
        self.active = True
        self.done_callback_func = done_callback
        self.current_seq_id = seq_id
        
        if not (0 <= seq_id <= 5):
            rospy.logerr(f"[DollTask] Invalid sequence ID: {seq_id}")
            if self.done_callback_func:
                self.done_callback_func("done_doll invalid_seq")
            return

        rospy.loginfo(f"[DollTask] Waiting for camera frame (seq={seq_id})...")
        self.cam_sub = rospy.Subscriber(self.image_topic, Image, self.on_camera_frame)

    def on_camera_frame(self, msg):
        if not self.active: return
        
        # 1. 优先返回 Done，确保导航立即继续
        if self.done_callback_func:
            self.done_callback_func("done_doll snapshot_taken")
            self.done_callback_func = None

        if self.cam_sub:
            self.cam_sub.unregister()
            self.cam_sub = None
            
        rospy.loginfo("[DollTask] Frame captured. Nav should resume immediately.")
        print("\n[Foreground] Doll Task Finished. Snapshot taken.\n")
            
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
                
            rospy.loginfo(f"[DollTask] Background processing started for seq {seq_id}.")
            
            self.current_job_data = {"classes": []}
            self.yolo_event.clear()
            self.voice_event.clear()
            
            # Publish Snapshot & Trigger YOLO
            self.snapshot_pub.publish(frame_msg)
            rospy.sleep(0.2)
            self.driver_pub.publish("snapshot doll")
            
            # Wait for YOLO
            if self.yolo_event.wait(timeout=5.0):
                detected = self.current_job_data["classes"]
                rospy.loginfo(f"[DollTask] YOLO finished. Detected: {detected}")
                self.update_global_state(seq_id, detected)
            else:
                rospy.logwarn("[DollTask] YOLO timeout.")
                
            # If seq_id == 5, trigger voice
            if seq_id == 5:
                self.finish_sequence()
            else:
                print("\n[Background] Doll Task Finished (Intermediate).\n")
                
            self.task_queue.task_done()

    def update_global_state(self, seq_id, detected_classes):
        if seq_id == 0: return # Test mode
        
        if 1 <= seq_id <= 3:
            for cls in detected_classes:
                self.block_a_results.add(cls)
        elif 4 <= seq_id <= 5:
            for cls in detected_classes:
                self.block_b_results.add(cls)
                if cls in self.block_a_results:
                    self.block_a_results.remove(cls)

    def finish_sequence(self):
        rospy.loginfo("="*30)
        rospy.loginfo(" FINAL DOLL RESULTS ")
        rospy.loginfo(f" Block A: {list(self.block_a_results)}")
        rospy.loginfo(f" Block B: {list(self.block_b_results)}")
        
        bad_guys = {'bad0', 'bad1', 'bad2', 'bad3'}
        def get_stats(results):
            bad_count = 0
            good_count = 0
            for name in results:
                if name in bad_guys:
                    bad_count += 1
                else:
                    good_count += 1
            return bad_count, good_count

        a_bad, a_good = get_stats(self.block_a_results)
        b_bad, b_good = get_stats(self.block_b_results)
        
        n2 = a_bad + a_good
        n3 = b_bad + b_good
        n1 = n2 + n3
        n4 = a_bad
        n5 = b_bad
        
        voice_data = {
            "n1": n1, "n2": n2, "n3": n3, "n4": n4, "n5": n5
        }
        
        cmd = {
            "action": "dispatch",
            "task": "stacking",
            "data": voice_data
        }
        self.voice_pub.publish(json.dumps(cmd))
        
        if self.voice_event.wait(timeout=5.0):
            rospy.loginfo("[DollTask] Voice receipt confirmed.")
        else:
            rospy.logwarn("[DollTask] Voice receipt timeout.")
            
        print("\n[Background] Doll Task Finished. Voice done.\n")

    def on_driver_data(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("model") != "doll": return
            
            class_ids = data.get("class_ids", [])
            current_frame_classes = []
            for cid in class_ids:
                if isinstance(cid, int) and 0 <= cid < len(self.target_classes):
                    current_frame_classes.append(self.target_classes[cid])
            
            self.current_job_data["classes"] = list(set(current_frame_classes))
            self.yolo_event.set()
        except:
            pass

    def voice_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("status") == "received" and data.get("original_task") == "stacking":
                self.voice_event.set()
        except:
            pass

