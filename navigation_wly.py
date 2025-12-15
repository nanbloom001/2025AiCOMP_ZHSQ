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
            [0.9429, -0.0583, 0.0371, 'traffic_lights'],      # 1 - traffic lights
            [2.3328, -0.1042, 1.4668, 'peoA1'],               # 2 - peoA1
            [3.2139, -0.147, 1.5392, 'none'],                 # 3
            [3.2577, 0.4444, -3.134, 'peoA2'],                # 4 - peoA2
            [3.2688, 0.936, 1.4961, 'stand'],                 # 5 - stand
            [2.6178, 1.0504, -1.6989, 'peoA3'],               # 6 - peoA3
            [2.6268, 1.0371, 1.5123, 'peoB1'],                # 7 - peoB1
            [1.8897, 1.0891, 3.0539, 'building1'],            # 8 - building1
            [1.8848, 1.7391, -0.0301, 'peoB2'],               # 9 - peoB2
            [1.8746, 1.855, 3.0066, 'building2'],             # 10 - building2
            [1.9116, 1.8266, 1.4895, 'traffic_lights'],       # 11 - traffic lights
            [1.9626, 3.2948, 3.048, 'none'],                  # 12
            [0.8344, 3.3136, 3.1305, 'car1'],                 # 13 - car1
            [0.6486, 2.6971, -3.0959, 'car2'],                # 14 - car2
            [0.8369, 2.5385, 0.1118, 'rubbish'],              # 15 - rubbish
            [0.8223, 2.1517, -3.0367, 'car3'],                # 16 - car3
            [0.6486, 1.5608, -0.0986, 'building3'],           # 17 - building3
            [0.7678, 1.2594, -3.0761, 'motor1'],              # 18 - motor1
            [0.7489, 0.7814, -3.1277, 'motor2'],              # 19 - motor2
            [0.638, 0.787, -1.5945, 'none'],                  # 20
            [0.6002, -0.0134, -0.0988, 'none'],               # 21
            [0.1952, -0.0231, 0.0502, 'none'],                # 22
        ]  
        self.current_goal_index = 0
        self.client = None
        self.lock = threading.Lock()
        self.is_navigating = False
        
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
    
    def navigate_to_goal(self, goal_index):
        """导航到指定目标点"""
        with self.lock:
            if self.is_navigating:
                rospy.logwarn("Navigation already in progress")
                return False
                
            self.is_navigating = True
        
        # 创建新的客户端实例（关键！）
        client = SimpleActionClient('move_base', MoveBaseAction)
        
        try:
            rospy.loginfo("Waiting for move_base action server...")
            if not client.wait_for_server(rospy.Duration(5.0)):
                rospy.logerr("move_base action server not available")
                return False
            
            goal = self.create_goal(self.goals[goal_index])
            rospy.loginfo("Sending goal %d: (%f, %f)", goal_index, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            
            client.send_goal(goal)
            client.wait_for_result()
            
            result = client.get_state()
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
            # 清理客户端
            try:
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