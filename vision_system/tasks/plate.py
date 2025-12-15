import rospy
import json
from std_msgs.msg import String

class PlateTask:
    """
    车牌识别任务控制器。
    运行在 Master 侧，负责调度 OCR 工具。
    """
    def __init__(self):
        self.active = False
        # 控制 OCR 工具的句柄
        self.driver_pub = rospy.Publisher("/vision/driver/ocr/cmd", String, queue_size=1)
        # 接收 OCR 工具数据的句柄
        self.driver_sub = None 
        self.result_callback_func = None # 回调给 Master 报告完成

    def start(self, callback_func, status_callback=None, seq_id=0):
        """任务开始"""
        self.active = True
        self.result_callback_func = callback_func
        
        # 1. 订阅 OCR 驱动的结果
        self.driver_sub = rospy.Subscriber("/vision/driver/ocr/result", String, self.on_driver_data)
        
        # 2. 开启 OCR 工具
        rospy.loginfo("[PlateTask] Requesting OCR Driver START...")
        self.driver_pub.publish("start")

    def stop(self):
        """任务强制停止"""
        self.active = False
        # 关闭 OCR 工具
        self.driver_pub.publish("stop")
        if self.driver_sub:
            self.driver_sub.unregister()
            self.driver_sub = None

    def on_driver_data(self, msg):
        """处理来自底层工具的数据"""
        if not self.active: return

        try:
            data = json.loads(msg.data)
            texts = data.get("texts", [])
            
            # --- 业务逻辑层 ---
            # 这里决定什么算“成功”。例如：必须包含数字，或者长度大于3
            if texts:
                result_str = ", ".join(texts)
                rospy.loginfo(f"[PlateTask] Logic satisfied! Result: {result_str}")
                
                # 任务完成，关闭工具
                self.stop()
                
                # 报告给 Master -> Nav
                if self.result_callback_func:
                    self.result_callback_func(f"done_plate {result_str}")
                    
        except json.JSONDecodeError:
            pass
 