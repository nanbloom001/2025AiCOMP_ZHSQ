import rospy
import json
import time
import threading
import sys
import os
from std_msgs.msg import String

# Try to import config to get PRINT_TIMING_INFO
try:
    import config
except ImportError:
    # Add parent dir to path if running from tasks subdir
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    try:
        import config
    except ImportError:
        config = None

class GarbageController:
    """
    垃圾分类任务控制器 (Master Side)。
    负责发送指令给 YOLO Node 启动垃圾识别任务。
    收到结果后，调用 Voice Node 播报，并延迟返回 Done。
    """
    def __init__(self):
        self.active = False
        self.driver_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.driver_sub = None
        self.voice_status_sub = None
        self.done_callback = None
        self.receipt_event = threading.Event()

    def start(self, done_callback, status_callback, seq_id=0):
        """启动任务"""
        self.active = True
        self.done_callback = done_callback
        self.receipt_event.clear()
        self.start_time = time.time()
        self.yolo_time = 0
        
        # 订阅结果话题
        if self.driver_sub:
            self.driver_sub.unregister()
        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.result_cb)
        
        # 订阅语音回执
        if self.voice_status_sub:
            self.voice_status_sub.unregister()
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        
        # 发送启动指令
        rospy.loginfo("[GarbageController] Sending start command...")
        self.driver_pub.publish("start garbage")

    def stop(self):
        """停止任务"""
        self.active = False
        if self.driver_sub:
            self.driver_sub.unregister()
            self.driver_sub = None
        if self.voice_status_sub:
            self.voice_status_sub.unregister()
            self.voice_status_sub = None
        self.driver_pub.publish("stop")

    def voice_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            # 确认收到 trash_bin 任务的回执
            if data.get("status") == "received" and data.get("original_task") == "trash_bin":
                self.receipt_event.set()
        except:
            pass

    def result_cb(self, msg):
        if not self.active: return
        
        self.yolo_time = time.time()
        data = msg.data
        # 兼容 "voice:" (旧) 和 "done:" (新) 前缀
        if data.startswith("voice:") or data.startswith("done:"):
            # 提取文本
            voice_text = data.replace("voice:", "").replace("done:", "").strip()
            rospy.loginfo(f"[GarbageController] Result received: {voice_text}")
            
            # 启动异步线程等待回执，然后结束任务
            threading.Thread(target=self.wait_for_receipt_and_finish, args=(data,)).start()
            
            # 立即停止 YOLO 识别，防止重复触发
            # 注意：不要调用 self.stop() 因为会注销 voice_status_sub，导致收不到回执
            # 只发送停止指令
            self.driver_pub.publish("stop")
            
            # 安全注销 driver_sub
            if self.driver_sub:
                try:
                    self.driver_sub.unregister()
                except Exception as e:
                    rospy.logwarn(f"Failed to unregister driver_sub: {e}")
                self.driver_sub = None

    def wait_for_receipt_and_finish(self, result_data):
        rospy.loginfo("[GarbageController] Waiting for voice receipt...")
        # 等待回执，超时 2 秒
        if self.receipt_event.wait(timeout=2.0):
            rospy.loginfo("[GarbageController] Receipt confirmed.")
        else:
            rospy.logwarn("[GarbageController] Receipt timeout, proceeding anyway.")
        
        voice_end_time = time.time()
        wait_yolo = round(self.yolo_time - self.start_time, 3)
        wait_voice = round(voice_end_time - self.yolo_time, 3)
        
        ctrl_timing = {'wait_yolo': wait_yolo, 'wait_voice': wait_voice}
        
        if config and getattr(config, 'PRINT_TIMING_INFO', False):
            print(f"[\033[36mGarbageController Detail\033[0m] {json.dumps(ctrl_timing)}")
        
        # Append controller timing to result
        final_result = f"{result_data} | ctrl_timing={json.dumps(ctrl_timing)}"
        
        if self.done_callback:
            self.done_callback(final_result)
        
        # 最后彻底停止
        self.stop()

