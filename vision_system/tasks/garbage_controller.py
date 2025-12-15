import rospy
import json
import time
import threading
from std_msgs.msg import String

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
        self.done_callback = None
        
        # 播报缓冲时间 (秒)
        self.VOICE_BUFFER_TIME = 3.0 

    def start(self, done_callback, status_callback, seq_id=0):
        """启动任务"""
        self.active = True
        self.done_callback = done_callback
        
        # 订阅结果话题
        if self.driver_sub:
            self.driver_sub.unregister()
        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.result_cb)
        
        # 发送启动指令
        rospy.loginfo("[GarbageController] Sending start command...")
        self.driver_pub.publish("start garbage")

    def stop(self):
        """停止任务"""
        self.active = False
        if self.driver_sub:
            self.driver_sub.unregister()
            self.driver_sub = None
        self.driver_pub.publish("stop")

    def result_cb(self, msg):
        if not self.active: return
        
        data = msg.data
        if data.startswith("voice:"):
            # 提取语音文本
            voice_text = data.replace("voice:", "").strip()
            rospy.loginfo(f"[GarbageController] Result received: {voice_text}")
            
            # 1. (已移除) 发送语音播报指令 - 现在由 GarbageTask 直接发送
            # cmd = {
            #     "action": "say",
            #     "text": voice_text,
            #     "id": "garbage_result",
            #     "speed": 1.1
            # }
            # self.voice_pub.publish(json.dumps(cmd))
            
            # 2. 启动异步线程等待缓冲时间，然后结束任务
            threading.Thread(target=self.finish_sequence, args=(data,)).start()
            
            # 立即停止 YOLO 识别，防止重复触发
            self.stop()

    def finish_sequence(self, result_data):
        rospy.loginfo(f"[GarbageController] Waiting {self.VOICE_BUFFER_TIME}s for voice buffer...")
        time.sleep(self.VOICE_BUFFER_TIME)
        
        if self.done_callback:
            rospy.loginfo("[GarbageController] Buffer done. Reporting task success.")
            self.done_callback(result_data)

