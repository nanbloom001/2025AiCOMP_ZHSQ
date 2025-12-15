import rospy
import json
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
        self.driver_sub = None 
        self.done_callback_func = None 
        
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
        
        # 异常机制：如果出现除了0-5之外的序号则控制台报错，并直接返回任务成功
        if not (0 <= seq_id <= 5):
            rospy.logerr(f"[DollTask] Invalid sequence ID: {seq_id}. Must be 0-5.")
            self.stop()
            if self.done_callback_func:
                self.done_callback_func("done_doll invalid_seq")
            return

        self.driver_sub = rospy.Subscriber("/vision/driver/yolo/result", String, self.on_driver_data)
        
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

    def on_driver_data(self, msg):
        if not self.active: return

        try:
            data = json.loads(msg.data)
            # 兼容 model 名
            if data.get("model") != "doll":
                return
                
            class_ids = data.get("class_ids", [])
            # 提取当前帧检测到的目标类别
            current_frame_classes = []
            for cid in class_ids:
                if isinstance(cid, int) and 0 <= cid < len(self.target_classes):
                    current_frame_classes.append(self.target_classes[cid])
            
            self.detection_buffer.append(current_frame_classes)
            self.frame_count += 1
            
            # 连续保存5次的识别结果
            if self.frame_count >= 5:
                self.process_results()
                self.stop()
                    
        except json.JSONDecodeError:
            pass

    def process_results(self):
        # 统计 5 帧内的结果
        # 设定一个阈值如3次，超过这个阈值即判断为真
        
        class_counts = Counter()
        for frame_classes in self.detection_buffer:
            # 去重，一帧内多次出现只算一次
            unique_classes_in_frame = set(frame_classes)
            for cls in unique_classes_in_frame:
                class_counts[cls] += 1
        
        detected_classes = []
        for cls, count in class_counts.items():
            if count >= 3:
                detected_classes.append(cls)
        
        rospy.loginfo(f"[DollTask] Seq {self.current_seq_id} Raw Detection (>=3/5 frames): {detected_classes}")

        # 如果出现序号0，则直接返回识别结果，但不存入列表
        if self.current_seq_id == 0:
            result_str = f"done_doll {len(detected_classes)}"
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

        if self.done_callback_func:
            self.done_callback_func(f"done_doll {len(detected_classes)}")
