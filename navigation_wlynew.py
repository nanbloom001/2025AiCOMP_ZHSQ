#!/usr/bin/env python
# coding:utf-8
import rospy
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

class NavigationController:
    def __init__(self):
        self.position = [0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.goals = [
            [1.007, -0.0313, -0.0269, 'none'],           # 1
            [2.4271, -0.0878, -0.0327, 'none'],          # 2
            [2.5021, -0.0609, 1.5036, 'peoA1'],          # 3 - peoA1
            [2.5052, -0.0683, -0.023, 'none'],           # 4
            [3.1925, -0.1134, -0.0705, 'none'],          # 5
            [3.3601, -0.0975, 1.5568, 'none'],           # 6
            [3.3723, 0.3622, 1.4943, 'none'],            # 7
            [3.368, 0.4439, -3.1309, 'peoA2'],           # 8 - peoA2
            [3.3441, 0.4354, 1.4854, 'none'],            # 9
            [3.3543, 1.0162, 1.5449, 'sign'],            # 10 - sign
            [3.3497, 1.0412, 3.0944, 'none'],            # 11
            [2.5216, 1.0237, -3.1118, 'none'],           # 12
            [2.5265, 1.0277, -1.766, 'peoA3'],           # 13 - peoA3
            [2.5296, 1.0299, 1.4427, 'peoB1'],           # 14 - peoB1
            [2.5499, 1.0173, 3.0361, 'none'],            # 15
            [1.9494, 1.0248, 3.1086, 'building1'],       # 16 - building1
            [1.8582, 1.0409, 1.4775, 'none'],            # 17
            [1.891, 1.6092, 1.5274, 'none'],             # 18
            [1.9052, 1.8039, -0.2063, 'peoB2'],          # 19 - peoB2
            [1.8859, 1.7875, 1.484, 'lights'],           # 20 - lights
            [1.9451, 3.2437, 1.5285, 'none'],            # 21
            [1.9425, 3.2753, 3.094, 'none'],             # 22
            [0.7866, 3.2978, 3.1343, 'car1'],            # 23 - car1
            [0.7836, 3.2986, -1.586, 'none'],            # 24
            [0.7706, 2.8457, -1.613, 'none'],            # 25
            [0.7684, 2.7052, -3.1104, 'car2'],           # 26 - car2
            [0.7772, 2.7176, -1.6326, 'none'],           # 27
            [0.78, 2.6127, -1.5742, 'none'],             # 28
            [0.7963, 2.5053, 0.0159, 'bin'],             # 29 - bin
            [0.7877, 2.5066, -1.6758, 'none'],           # 30
            [0.7645, 2.1732, -1.6683, 'none'],           # 31
            [0.7594, 2.0563, -3.115, 'car3'],            # 32 - car3
            [0.7868, 2.0749, -1.6737, 'none'],           # 33
            [0.7986, 1.455, 0.0373, 'building3'],        # 34 - building3
            [0.8201, 1.4647, -1.5834, 'none'],           # 35
            [0.7978, 1.2881, -1.6691, 'none'],           # 36
            [0.805, 1.3, -3.0572, 'motor1'],             # 37 - motor1
            [0.8148, 1.3056, -1.6782, 'none'],           # 38
            [0.8085, 0.7411, -1.591, 'none'],            # 39
            [0.7932, 0.7422, 3.1065, 'motor2'],          # 40 - motor2
            [0.8055, 0.7527, -1.6335, 'none'],           # 41
            [0.8228, -0.0583, -1.5503, 'none'],          # 42
            [0.821, -0.0609, 0.0368, 'none'],            # 43
            [0.1694, -0.1258, 0.1259, 'none'],           # 44
        ]  
        self.current_goal_index = 0
        self.client = None
        self.lock = threading.Lock()
        self.is_navigating = False
        self.client_initialized = False
        
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
    
    def initialize_client(self):
        """初始化action client（只调用一次）"""
        if not self.client_initialized:
            rospy.loginfo("Initializing move_base action client...")
            self.client = SimpleActionClient('move_base', MoveBaseAction)
            rospy.loginfo("Waiting for move_base action server...")
            if not self.client.wait_for_server(rospy.Duration(10.0)):
                rospy.logerr("move_base action server not available")
                return False
            rospy.loginfo("Connected to move_base action server")
            self.client_initialized = True
        return True
    
    def navigate_to_goal(self, goal_index):
        """导航到指定目标点"""
        with self.lock:
            if self.is_navigating:
                rospy.logwarn("Navigation already in progress")
                return False
                
            self.is_navigating = True
        
        try:
            goal = self.create_goal(self.goals[goal_index])
            rospy.loginfo("Sending goal %d: (%f, %f)", goal_index, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            
            # 先取消所有之前的目标，确保状态干净
            self.client.cancel_all_goals()
            rospy.sleep(0.1)  # 短暂等待取消完成
            
            self.client.send_goal(goal)
            self.client.wait_for_result()
            
            result = self.client.get_state()
            if result == 3:  # SUCCEEDED
                rospy.loginfo("Successfully reached goal %d", goal_index)
                rospy.loginfo("Actual position: (%f, %f)", self.position[0], self.position[1])
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
    
    def run_navigation(self):
        """执行完整的导航序列"""
        rospy.init_node('navigation_controller', anonymous=True)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        # 等待系统稳定
        rospy.loginfo("Initializing, please wait...")
        rospy.sleep(3.0)
        
        # 初始化action client（只初始化一次）
        if not self.initialize_client():
            rospy.logerr("Failed to initialize action client")
            return
        
        # 依次导航到所有目标点
        for i in range(len(self.goals)):
            rospy.loginfo("=== Starting navigation to goal %d/%d ===", i+1, len(self.goals))
            
            success = self.navigate_to_goal(i)
            
            if not success:
                rospy.logerr("Failed to reach goal %d, stopping navigation", i)
                break
            
            # 检查是否需要在此点位停顿
            task_type = self.goals[i][3]
            if task_type != 'none':
                rospy.loginfo("Task at waypoint %d: %s - Pausing 1 second...", i+1, task_type)
                rospy.sleep(1.0)
        
        rospy.loginfo("=== Navigation completed ===")

if __name__ == '__main__':
    try:
        controller = NavigationController()
        controller.run_navigation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")