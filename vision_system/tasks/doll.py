import rospy
import json
import threading
import time
from std_msgs.msg import String
from collections import Counter

class DollTask:
    """
    玩偶检测任务控制器。
    运行在 Master 侧，负责调度 YOLO 工具。
    """
    def __init__(self):
        self.active = False
        self.driver_pub = rospy.Publisher("/vision/driver/yolo/cmd", String, queue_size=1)
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        self.driver_sub = None 
        self.voice_status_sub = None
        self.done_callback_func = None 
        self.receipt_event = threading.Event()
        
        # 存储 A 街区和 B 街区的识别结果 (Set of strings)
        self.block_a_results = set()
        self.block_b_results = set()
        
        # 状态变量
        self.current_seq_id = 0
        self.frame_count = 0
        self.detection_buffer = [] # List of lists of detected classes
        
        # 目标类别
        self.target_classes = ['bad0', 'bad1', 'bad2', 'bad3', 'coffee', 'cook', 'dective', 'doctor', 'engineer', 'fireman', 'gardener', 'guitar', 'it', 'office', 'painter', 'photo', 'postman', 'professor', 'rapper', 'security', 'teacher']

    def start(self, done_callback, status_callback, seq_id=0):
        """任务开始"""
        self.active = True
        self.done_callback_func = done_callback
        self.current_seq_id = seq_id
        self.frame_count = 0
        self.detection_buffer = []
        self.receipt_event.clear()
        # timing
        self.start_time = time.time()
        
        # 异常机制：如果出现除了0-5之外的序号则控制台报错，并直接返回任务成功
        if not (0 <= seq_id <= 5):
            rospy.logerr(f"[DollTask] Invalid sequence ID: {seq_id}. Must be 0-5.")
            self.stop()
            if self.done_callback_func:
                self.done_callback_func("done_doll invalid_seq")
            return

        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_driver_data)
        
        # 订阅语音回执
        if self.voice_status_sub:
            self.voice_status_sub.unregister()
        self.voice_status_sub = rospy.Subscriber("/vision/driver/voice/status", String, self.voice_status_cb)
        
        # 使用 people_v2 模型 (Driver 中映射为 "doll")
        rospy.loginfo(f"[DollTask] Starting task with seq_id={seq_id}. Requesting YOLO Driver START (model=doll)...")
        self.driver_pub.publish("start doll")

    def stop(self):
        """任务强制停止"""
        self.active = False
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
            if data.get("status") == "received" and data.get("original_task") == "stacking":
                self.receipt_event.set()
        except:
            pass

    def on_driver_data(self, msg):
        if not self.active: return

        try:
            data = json.loads(msg.data)
            # 兼容 model 名
            if data.get("model") != "doll":
                return
            
            timing = data.get("timing", {})
            
            # 立即停止 YOLO，实现"单帧"效果
            self.driver_pub.publish("stop")
            if self.driver_sub:
                self.driver_sub.unregister()
                self.driver_sub = None
                
            class_ids = data.get("class_ids", [])
            # 提取当前帧检测到的目标类别
            current_frame_classes = []
            for cid in class_ids:
                if isinstance(cid, int) and 0 <= cid < len(self.target_classes):
                    current_frame_classes.append(self.target_classes[cid])
            
            # 直接处理单帧结果
            self.process_results(current_frame_classes, timing)
            self.stop()
                    
        except json.JSONDecodeError:
            pass

    def process_results(self, detected_classes, timing=None):
        # 单帧模式，直接使用检测结果
        # 去重
        detected_classes = list(set(detected_classes))
        
        rospy.loginfo(f"[DollTask] Seq {self.current_seq_id} Single Frame Detection: {detected_classes}")

        # 如果出现序号0，则直接返回识别结果，但不存入列表
        if self.current_seq_id == 0:
            result_str = f"done_doll {len(detected_classes)}"
            if timing:
                result_str += f" | timing={json.dumps(timing)}"
            # append duration in ms
            dur_ms = round((time.time() - getattr(self, 'start_time', time.time())) * 1000, 2)
            result_str += f" | duration_ms={dur_ms}"
            if self.done_callback_func:
                self.done_callback_func(result_str)
            return

        # 存入列表
        # 1和2和3为1组，称为A街区人群
        if 1 <= self.current_seq_id <= 3:
            for cls in detected_classes:
                self.block_a_results.add(cls)
                
        # 4和5为一组称为B街区人群
        elif 4 <= self.current_seq_id <= 5:
            for cls in detected_classes:
                self.block_b_results.add(cls)
                # B街区对A街区结果有覆盖
                if cls in self.block_a_results:
                    self.block_a_results.remove(cls)
                    rospy.loginfo(f"[DollTask] Override: {cls} moved from Block A to Block B")

        # 每进行一个任务打印一次结果
        rospy.loginfo(f"[DollTask] Current Block A: {list(self.block_a_results)}")
        rospy.loginfo(f"[DollTask] Current Block B: {list(self.block_b_results)}")

        # 当执行完第五个街区的任务后分别打印A B街区的最终结果
        if self.current_seq_id == 5:
            rospy.loginfo("="*30)
            rospy.loginfo(" FINAL DOLL RESULTS ")
            rospy.loginfo(f" Block A: {list(self.block_a_results)}")
            rospy.loginfo(f" Block B: {list(self.block_b_results)}")
            
            # 统计社区人员和非社区人员
            # 非社区人员: bad0, bad1, bad2, bad3
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
            
            rospy.loginfo("-" * 20)
            rospy.loginfo(f" Block A Stats: Non-Community (Bad)={a_bad}, Community (Good)={a_good}")
            rospy.loginfo(f" Block B Stats: Non-Community (Bad)={b_bad}, Community (Good)={b_good}")
            rospy.loginfo("="*30)

            # 发送语音播报
            n2 = a_bad + a_good
            n3 = b_bad + b_good
            n1 = n2 + n3
            n4 = a_bad
            n5 = b_bad
            
            voice_data = {
                "n1": n1,
                "n2": n2,
                "n3": n3,
                "n4": n4,
                "n5": n5
            }
            
            cmd = {
                "action": "dispatch",
                "task": "stacking",
                "data": voice_data
            }
            self.voice_pub.publish(json.dumps(cmd))
            rospy.loginfo(f"[DollTask] Voice command sent: {voice_data}")
            
            # 启动异步线程等待回执，然后结束任务
            threading.Thread(target=self.wait_for_receipt_and_finish, args=(len(detected_classes), timing)).start()
            return

        result_str = f"done_doll {len(detected_classes)}"
        if timing:
            result_str += f" | timing={json.dumps(timing)}"
        dur_ms = round((time.time() - getattr(self, 'start_time', time.time())) * 1000, 2)
        result_str += f" | duration_ms={dur_ms}"
            
        if self.done_callback_func:
            self.done_callback_func(result_str)
        self.stop()

    def wait_for_receipt_and_finish(self, count, timing=None):
        rospy.loginfo("[DollTask] Waiting for voice receipt...")
        if self.receipt_event.wait(timeout=2.0):
            rospy.loginfo("[DollTask] Receipt confirmed.")
        else:
            rospy.logwarn("[DollTask] Receipt timeout, proceeding anyway.")
            
        result_str = f"done_doll {count}"
        if timing:
            result_str += f" | timing={json.dumps(timing)}"
            
        if self.done_callback_func:
            self.done_callback_func(result_str)
        self.stop()
            
        if self.done_callback_func:
            self.done_callback_func(f"done_doll {count}")
        self.stop()
