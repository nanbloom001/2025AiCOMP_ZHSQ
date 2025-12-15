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
        self.active = True
        self.done_callback = done_callback
        
        # Subscribe to result
        if self.driver_sub:
            self.driver_sub.unregister()
        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.result_cb)
        
        # Send Start Command
        rospy.loginfo("[FallenVehicleController] Sending start command...")
        self.driver_pub.publish("start fallen_vehicle")

    def stop(self):
        self.active = False
        if self.driver_sub:
            self.driver_sub.unregister()
            self.driver_sub = None
        self.driver_pub.publish("stop")

    def result_cb(self, msg):
        if not self.active: return
        
        data = msg.data
        # Check for specific done message from FallenVehicleTask
        if "done_fallen_vehicle" in data:
            if self.done_callback:
                self.done_callback(data)
            self.stop()
