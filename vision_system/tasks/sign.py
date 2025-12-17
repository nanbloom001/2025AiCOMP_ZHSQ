import rospy
import json
import time
import threading
from std_msgs.msg import String

class SignTask:
    """
    Sign 任务控制器。
    功能：到达点位后，触发语音播放 'stop.WAV'，然后结束任务。
    """
    def __init__(self):
        self.active = False
        # 语音发布者
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.done_callback_func = None
        self.timer = None

    def start(self, callback_func, status_callback=None, seq_id=0):
        """任务开始"""
        self.active = True
        self.done_callback_func = callback_func
        
        rospy.loginfo("[SignTask] Started. Returning DONE immediately.")
        
        # 1. 立即结束任务，让机器人继续移动
        if self.done_callback_func:
            self.done_callback_func("done_sign success")
            self.done_callback_func = None
            
        # 2. 启动后台线程进行延时播报
        threading.Thread(target=self.delayed_voice_task).start()
        
        self.active = False

    def delayed_voice_task(self):
        """后台延时播报"""
        rospy.loginfo("[SignTask] Waiting 1.0s before voice...")
        time.sleep(1.0)
        
        # 构造指令数据
        cmd = {
            "action": "dispatch",
            "task": "system",
            "data": "stop"
        }
        
        # 发布语音指令
        self.voice_pub.publish(json.dumps(cmd))
        rospy.loginfo("[SignTask] Voice command sent (stop.WAV).")

    def stop(self):
        """任务强制停止"""
        self.active = False

    def finish(self):
        """任务完成 (已弃用，直接在 start 中完成)"""
        pass
