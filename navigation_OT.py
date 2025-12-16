#!/home/mowen/miniconda3/envs/zzz/bin/python
# coding:utf-8
import rospy
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

class NavigationController:
    def __init__(self):
        self.position = [0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        
        # 目标点列表：[x, y, yaw, task_type]
        # task_type: 'none' - 无任务, 'ocr' - OCR识别, 'traffic_light' - 红绿灯检测, 'yolo' - 人员检测, 'wait' - 等待
        self.goals = [
            [1.069, -0.04, -0.038, 'traffic_light'],      # 第一个红绿灯 - OCR识别
            [3.279, -0.082, 0.262, 'none'],     # 无任务
            [3.386, -0.151, 1.4979, 'none'],     # 无任务

            [3.442, 0.989, 1.4617, 'none'],     # 无任务
            [3.451, 0.967, 3.109, 'none'],     # 无任务

            [2.695, 0.968, 3.081, 'none'],     # 无任务

            [2.760, 1.028, 1.6967, 'rog'],     # shequ
            [2.789, 1.021, -1.705, 'rog'],     # shequ
            [2.783, 0.999, 3.086, 'none'],     # 无任务

            [2.007, 1.034, 3.094, 'none'],     # 无任务
            [1.852, 1.059, 1.428, 'none'],     # 无任务

            [1.920, 1.834, 1.415, 'traffic_light'],      # 红绿灯 - OCR识别

            [1.920, 3.254, 1.432, 'none'],     # 无任务
            [2.000, 3.292, -3.074, 'none'],     # 无任务


            [0.828, 3.318, 3.117, 'ocr'],     # 车牌１
            [0.828, 2.734, 3.111, 'ocr'],     # 车牌2
            [0.740, 2.122, 3.107, 'ocr'],     # 车牌3
            [0.673, 2.088, -1.645, 'none'],     # 

            [0.520, -0.050, 0.135, 'none'],     # 无任务
            [0.066, -0.061, 0.041, 'none'],    # 无任务
        ]  
        
        self.current_goal_index = 0
        self.client = None
        self.lock = threading.Lock()
        self.is_navigating = False
        
        # 任务触发发布者
        self.task_pub = rospy.Publisher('/navigation_task', String, queue_size=10)
        
        # 任务完成标志
        self.task_completed = False
        self.task_result = None
        
        # 订阅任务完成话题
        self.ocr_done_sub = rospy.Subscriber('/ocr_task_done', String, self.ocr_done_callback)
        self.traffic_light_done_sub = rospy.Subscriber('/traffic_light_done', String, self.traffic_light_done_callback)
        self.yolo_done_sub = rospy.Subscriber('/yolo_task_done', String, self.yolo_done_callback)
        
    def ocr_done_callback(self, msg):
        """OCR任务完成回调"""
        rospy.loginfo("OCR task result: {}".format(msg.data))
        self.task_result = msg.data
        self.task_completed = True
    
    def traffic_light_done_callback(self, msg):
        """红绿灯检测完成回调"""
        rospy.loginfo("Traffic light result: {}".format(msg.data))
        self.task_result = msg.data
        self.task_completed = True
    
    def yolo_done_callback(self, msg):
        """YOLO检测完成回调"""
        rospy.loginfo("YOLO task result: {}".format(msg.data))
        self.task_result = msg.data
        self.task_completed = True
        
    def amcl_callback(self, data):
        pose = data.pose.pose
        self.position = [pose.position.x, pose.position.y]
        self.orientation = [
            pose.orientation.x,
            pose.orientation.y, 
            pose.orientation.z,
            pose.orientation.w
        ]
    
    def create_goal(self, goal_data):
        """创建目标点"""
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = "map"
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.pose.position.x = goal_data[0]
        goal_pose.target_pose.pose.position.y = goal_data[1]
        goal_pose.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, goal_data[2])
        goal_pose.target_pose.pose.orientation.x = q[0]
        goal_pose.target_pose.pose.orientation.y = q[1]
        goal_pose.target_pose.pose.orientation.z = q[2]
        goal_pose.target_pose.pose.orientation.w = q[3]
        return goal_pose
    
    def execute_task(self, task_type, goal_index):
        """执行目标点的任务"""
        if task_type == 'none':
            return True
        
        rospy.loginfo("=== Executing task: %s at goal %d ===", task_type, goal_index)
        
        # 重置任务完成标志
        self.task_completed = False
        self.task_result = None
        
        if task_type == 'ocr':
            # 发布OCR任务触发消息
            self.task_pub.publish("trigger_ocr")
            rospy.loginfo("OCR task triggered, waiting for completion...")
            
            # 等待OCR任务完成（带超时）
            timeout = rospy.Time.now() + rospy.Duration(15.0)  # 15秒超时
            rate = rospy.Rate(10)
            
            while not self.task_completed and rospy.Time.now() < timeout and not rospy.is_shutdown():
                rate.sleep()
            
            if self.task_completed:
                rospy.loginfo("OCR task completed: {}".format(self.task_result))
                return True
            else:
                rospy.logwarn("OCR task timeout!")
                return False
        
        elif task_type == 'rog':
            # 发布YOLO检测任务触发消息
            self.task_pub.publish("trigger_rog")
            rospy.loginfo("YOLO detection triggered, waiting for completion...")
            
            # 等待YOLO任务完成（带超时）
            timeout = rospy.Time.now() + rospy.Duration(15.0)  # 15秒超时
            rate = rospy.Rate(10)
            
            while not self.task_completed and rospy.Time.now() < timeout and not rospy.is_shutdown():
                rate.sleep()
            
            if self.task_completed:
                rospy.loginfo("YOLO task completed: {}".format(self.task_result))
                return True
            else:
                rospy.logwarn("YOLO task timeout!")
                return False
                
        elif task_type == 'traffic_light':
            # 发布红绿灯检测任务触发消息
            self.task_pub.publish("trigger_traffic_light")
            rospy.loginfo("Traffic light detection triggered, waiting for GREEN light...")
            
            # 等待绿灯（无超时，一直等待）
            rate = rospy.Rate(10)
            
            while not self.task_completed and not rospy.is_shutdown():
                rate.sleep()
            
            if self.task_completed and self.task_result == "green_light":
                rospy.loginfo("GREEN LIGHT confirmed! Proceeding to next goal...")
                return True
            else:
                rospy.logerr("Traffic light detection failed!")
                return False
                
        elif task_type == 'wait':
            # 简单等待任务
            rospy.loginfo("Waiting at goal point...")
            rospy.sleep(3.0)
            return True
        
        rospy.loginfo("=== Task completed ===")
        return True
    
    def navigate_to_goal(self, goal_index):
        """导航到指定目标点"""
        with self.lock:
            if self.is_navigating:
                rospy.logwarn("Navigation already in progress")
                return False
                
            self.is_navigating = True
        
        # 创建新的客户端实例
        client = SimpleActionClient('move_base', MoveBaseAction)
        
        try:
            rospy.loginfo("Waiting for move_base action server...")
            if not client.wait_for_server(rospy.Duration(5.0)):
                rospy.logerr("move_base action server not available")
                return False
            
            goal = self.create_goal(self.goals[goal_index])
            task_type = self.goals[goal_index][3]
            
            rospy.loginfo("Sending goal %d: (%f, %f) - Task: %s", 
                         goal_index, 
                         goal.target_pose.pose.position.x, 
                         goal.target_pose.pose.position.y,
                         task_type)
            
            client.send_goal(goal)
            client.wait_for_result()
            
            result = client.get_state()
            if result == 3:  # SUCCEEDED
                rospy.loginfo("Successfully reached goal %d", goal_index)
                rospy.loginfo("Actual position: (%f, %f)", self.position[0], self.position[1])
                
                # 执行目标点的任务
                task_success = self.execute_task(task_type, goal_index)
                
                if not task_success:
                    rospy.logerr("Task failed at goal %d", goal_index)
                    return False
                
                return True
            else:
                rospy.logwarn("Failed to reach goal %d, state: %d", goal_index, result)
                return False
                
        except Exception as e:
            rospy.logerr("Error during navigation: %s", str(e))
            return False
        finally:
            with self.lock:
                self.is_navigating = False
            # 只在目标未完成时取消
            try:
                state = client.get_state()
                # 只有在活跃状态时才取消目标
                # 0=PENDING, 1=ACTIVE, 2=PREEMPTED, 3=SUCCEEDED, 4=ABORTED, etc.
                if state in [0, 1]:  # PENDING or ACTIVE
                    client.cancel_goal()
            except:
                pass
    
    def run_navigation(self):
        """执行完整的导航序列"""
        rospy.init_node('navigation_controller', anonymous=True)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        # 等待系统稳定
        rospy.loginfo("Initializing, please wait...")
        rospy.sleep(3.0)
        
        # 依次导航到所有目标点
        for i in range(len(self.goals)):
            rospy.loginfo("=== Starting navigation to goal %d/%d ===", i+1, len(self.goals))
            
            success = self.navigate_to_goal(i)
            
            if not success:
                rospy.logerr("Failed to reach goal %d, stopping navigation", i)
                break
                
            # 在目标点之间暂停
            if i < len(self.goals) - 1:
                rospy.loginfo("Pausing before next goal...")
                rospy.sleep(1.0)
        
        rospy.loginfo("=== Navigation completed ===")

if __name__ == '__main__':
    try:
        controller = NavigationController()
        controller.run_navigation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")