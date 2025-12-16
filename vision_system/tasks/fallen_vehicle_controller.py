import rospy
import threading
import json
from std_msgs.msg import String

class FallenVehicleController:
    """
    倒车检测任务控制器 (Master Side)。
    负责发送指令给 YOLO Node 启动倒车检测任务。
    """
    def __init__(self):
        self.active = False
        self.driver_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        self.driver_sub = None
        self.voice_status_sub = None
        self.done_callback = None
        self.receipt_event = threading.Event()

    def start(self, done_callback, status_callback, seq_id=0):
        """启动任务"""
        self.active = True
        self.done_callback = done_callback
        self.receipt_event.clear()
        
        # 订阅结果话题
        if self.driver_sub:
            self.driver_sub.unregister()
        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.result_cb)
        
        # 订阅语音回执
        if self.voice_status_sub:
            self.voice_status_sub.unregister()
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        
        # 发送启动指令
        rospy.loginfo("[FallenVehicleController] Sending start command...")
        self.driver_pub.publish("start fallen_vehicle")

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
            if data.get("status") == "received" and data.get("original_task") == "bikes":
                self.receipt_event.set()
        except:
            pass

    def result_cb(self, msg):
        """处理返回结果"""
        if not self.active: return
        
        data = msg.data
        # 检查是否收到 FallenVehicleTask 的完成信号
        if "done_fallen_vehicle" in data:
            # 启动异步线程等待回执，然后结束任务
            threading.Thread(target=self.wait_for_receipt_and_finish, args=(data,)).start()
            
            # 立即停止 YOLO
            self.driver_pub.publish("stop")
            if self.driver_sub:
                self.driver_sub.unregister()
                self.driver_sub = None

    def wait_for_receipt_and_finish(self, result_data):
        rospy.loginfo("[FallenVehicleController] Waiting for voice receipt...")
        if self.receipt_event.wait(timeout=2.0):
            rospy.loginfo("[FallenVehicleController] Receipt confirmed.")
        else:
            rospy.logwarn("[FallenVehicleController] Receipt timeout, proceeding anyway.")
            
        if self.done_callback:
            self.done_callback(result_data)
        self.stop()
