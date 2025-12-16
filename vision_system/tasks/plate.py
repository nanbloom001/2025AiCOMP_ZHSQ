import rospy
import json
import re
import math
import threading
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
        # 语音发布者
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        # 接收 OCR 工具数据的句柄
        self.driver_sub = None 
        self.voice_status_sub = None
        self.result_callback_func = None # 回调给 Master 报告完成
        self.receipt_event = threading.Event()

    def start(self, callback_func, status_callback=None, seq_id=0):
        """任务开始"""
        self.active = True
        self.result_callback_func = callback_func
        self.seq_id = seq_id
        self.receipt_event.clear()
        
        # 1. 订阅 OCR 驱动的结果
        self.driver_sub = rospy.Subscriber("/vision/driver/ocr/result", String, self.on_driver_data)
        
        # 2. 订阅语音回执
        if self.voice_status_sub:
            self.voice_status_sub.unregister()
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        
        # 3. 开启 OCR 工具
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
        if self.voice_status_sub:
            self.voice_status_sub.unregister()
            self.voice_status_sub = None

    def voice_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("status") == "received" and data.get("original_task") == "car_plate":
                self.receipt_event.set()
        except:
            pass

    def on_driver_data(self, msg):
        """处理来自底层工具的数据"""
        if not self.active: return

        try:
            data = json.loads(msg.data)
            texts = data.get("texts", [])
            boxes = data.get("boxes", [])
            
            if not texts: return
            
            # 1. 预处理与筛选
            candidates = []
            # 匹配规则：汉字 + 大写字母 + 5-6位字母或数字
            # 例如：苏A12345 (7位) 或 新能源 苏AD12345 (8位)
            pattern = re.compile(r'^[\u4e00-\u9fa5][A-Z][A-Z0-9]{5,6}$')
            
            for i, text in enumerate(texts):
                # 清洗字符：去除空格、特殊符号，转大写
                clean_text = re.sub(r'[^\u4e00-\u9fa5a-zA-Z0-9]', '', text).upper()
                
                if pattern.match(clean_text):
                    # 计算中心点距离
                    box = boxes[i] if i < len(boxes) else []
                    dist = float('inf')
                    if box:
                        # box is list of [x, y]
                        xs = [p[0] for p in box]
                        ys = [p[1] for p in box]
                        if xs and ys:
                            cx = sum(xs) / len(xs)
                            cy = sum(ys) / len(ys)
                            # 假设图像分辨率 640x480
                            dist = math.hypot(cx - 320, cy - 240)
                    
                    candidates.append((clean_text, dist))
            
            # 2. 决策逻辑
            if candidates:
                # 优先选择符合格式的，如果有多个，选择离中心最近的
                candidates.sort(key=lambda x: x[1])
                best_plate = candidates[0][0]
                
                rospy.loginfo(f"[PlateTask] Valid plate found: {best_plate} (Dist: {candidates[0][1]:.1f})")
                
                # Voice Broadcast
                # 假设 best_plate 格式为 "苏A12345"
                # 拆分为 中文部分(TTS) 和 字母数字部分(WAV)
                plate_chn = best_plate[0]
                plate_eng = best_plate[1:]
                
                # 使用传入的 seq_id 作为停车场ID
                parking_id = self.seq_id if hasattr(self, 'seq_id') and self.seq_id > 0 else 1
                
                cmd = {
                    "action": "dispatch",
                    "task": "car_plate",
                    "data": {
                        "parking_id": parking_id,
                        "plate_chn": plate_chn,
                        "plate_eng": plate_eng
                    }
                }
                self.voice_pub.publish(json.dumps(cmd))
                rospy.loginfo(f"[PlateTask] Voice command sent: {best_plate}")

                # 启动异步线程等待回执，然后结束任务
                threading.Thread(target=self.wait_for_receipt_and_finish, args=(best_plate,)).start()
                
                # 立即停止 OCR，防止重复触发
                self.driver_pub.publish("stop")
                if self.driver_sub:
                    self.driver_sub.unregister()
                    self.driver_sub = None
            else:
                # 如果没有符合格式的，继续等待下一帧
                pass
                    
        except json.JSONDecodeError:
            pass

    def wait_for_receipt_and_finish(self, best_plate):
        rospy.loginfo("[PlateTask] Waiting for voice receipt...")
        if self.receipt_event.wait(timeout=2.0):
            rospy.loginfo("[PlateTask] Receipt confirmed.")
        else:
            rospy.logwarn("[PlateTask] Receipt timeout, proceeding anyway.")
            
        if self.result_callback_func:
            self.result_callback_func(f"done_plate {best_plate}")
        self.stop()
 