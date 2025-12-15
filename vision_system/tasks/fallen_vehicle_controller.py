import rospy
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
        self.done_callback = None

    def start(self, done_callback, status_callback, seq_id=0):
        """启动任务"""
        self.active = True
        self.done_callback = done_callback
        
        # 订阅结果话题
        if self.driver_sub:
            self.driver_sub.unregister()
        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.result_cb)
        
        # 发送启动指令
        rospy.loginfo("[FallenVehicleController] Sending start command...")
        self.driver_pub.publish("start fallen_vehicle")

    def stop(self):
        """停止任务"""
        self.active = False
        if self.driver_sub:
            self.driver_sub.unregister()
            self.driver_sub = None
        self.driver_pub.publish("stop")

    def result_cb(self, msg):
        """处理返回结果"""
        if not self.active: return
        
        data = msg.data
        # 检查是否收到 FallenVehicleTask 的完成信号
        if "done_fallen_vehicle" in data:
            if self.done_callback:
                self.done_callback(data)
            self.stop()
