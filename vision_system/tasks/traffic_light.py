import rospy
import json
import time
from std_msgs.msg import String
class TrafficLightTask:
    """
    交通灯任务控制器。
    运行在 Master 侧，负责调度 YOLO 工具。
    """
    def __init__(self, model_list=None):
        self.active = False
        # 控制 YOLO 工具的句柄
        self.driver_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        # 接收 YOLO 工具数据的句柄
        self.driver_sub = None 
        self.status_callback_func = None 
        self.done_callback_func = None
        
        # Load class names for 'lights' model
        # Default fallback if model_list not provided
        self.class_names = ["red", "green", "yellow"]
        
        if model_list:
            cfg = next((m for m in model_list if m["name"] == "lights"), None)
            if cfg:
                self.class_names = cfg["class_names"]

    def start(self, done_callback, status_callback, seq_id=0):
        """任务开始"""
        self.active = True
        self.status_callback_func = status_callback
        self.done_callback_func = done_callback
        # timing
        self.start_time = time.time()
        
        # 1. 订阅 YOLO 驱动的结果
        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_driver_data)
        
        # 2. 开启 YOLO 工具 (指定模型为 traffic_light -> 对应 config 中的 lights)
        rospy.loginfo("[TrafficLightTask] Requesting YOLO Driver START (model=traffic_light)...")
        self.driver_pub.publish("start traffic_light")

    def stop(self):
        """任务强制停止"""
        self.active = False
        # 关闭 YOLO 工具
        self.driver_pub.publish("stop")
        if self.driver_sub:
            self.driver_sub.unregister()
            self.driver_sub = None

    def on_driver_data(self, msg):
        """处理来自底层工具的数据"""
        if not self.active: return

        try:
            data = json.loads(msg.data)
            # 检查是否是 traffic_light 模型的结果
            if data.get("model") != "traffic_light":
                return
                
            class_ids = data.get("class_ids", [])
            timing = data.get("timing", {})
            
            found_status = "null"
            current_labels = []
            
            for cid in class_ids:
                if cid < len(self.class_names):
                    current_labels.append(self.class_names[cid])
            
            if "red" in current_labels:
                found_status = "red_light"
            elif "yellow" in current_labels:
                found_status = "yellow_light"
            elif "green" in current_labels:
                found_status = "green_light"
            
            # 报告状态
            if self.status_callback_func:
                # compute duration since start
                dur_ms = round((time.time() - getattr(self, 'start_time', time.time())) * 1000, 2)
                msg = f"status_traffic_light {found_status} | timing={json.dumps(timing)} | duration_ms={dur_ms}"
                rospy.loginfo(f"[TrafficLightTask] Status: {found_status}, timing (driver): {json.dumps(timing)} (ms), duration_ms={dur_ms}")
                self.status_callback_func(msg)

            # 如果是绿灯，任务完成
            if found_status == "green_light" and self.done_callback_func:
                # 仅发送完成信号，不主动停止，由 Nav 节点控制停止
                rospy.loginfo_throttle(1.0, "[TrafficLightTask] Green light detected. Sending done signal...")
                self.done_callback_func("done_traffic_light green_light")
                    
        except json.JSONDecodeError:
            pass
