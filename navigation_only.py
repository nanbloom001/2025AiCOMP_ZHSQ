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
            [1.069, -0.061, -0.038, 'none'],      # 第一个红绿灯 - OCR识别
            [3.279, -0.082, 0.262, 'none'],     # 无任务
            [3.386, -0.151, 1.4979, 'none'],     # 无任务

            [3.442, 0.989, 1.4617, 'none'],     # 无任务
            [3.451, 0.967, 3.109, 'none'],     # 无任务

            [2.695, 0.968, 3.081, 'none'],     # 无任务

            [2.760, 1.028, 1.6967, 'none'],     # shequ
            [2.789, 1.021, -1.705, 'none'],     # shequ
            [2.783, 0.999, 3.086, 'none'],     # 无任务

            [2.007, 1.034, 3.094, 'none'],     # 无任务
            [1.852, 1.059, 1.428, 'none'],     # 无任务

            [1.958, 1.834, 1.415, 'none'],      # 红绿灯 - OCR识别

            [2.142, 3.254, 1.432, 'none'],     # 无任务
            [2.081, 3.292, -3.074, 'none'],     # 无任务


            [1.035, 3.318, 3.117, 'none'],     # 车牌１
            [0.828, 2.734, 3.111, 'none'],     # 车牌2
            [0.740, 2.122, 3.107, 'none'],     # 车牌3
            [0.673, 2.088, -1.645, 'none'],     # 

            [0.520, -0.050, 0.135, 'none'],     # 无任务
            [0.066, -0.061, 0.041, 'none'],    # 无任务
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
                
            # 在目标点之间暂停
            if i < len(self.goals) - 1:
                rospy.loginfo("Pausing before next goal...")
                # rospy.sleep(2.0)  # 2秒暂停
        
        rospy.loginfo("=== Navigation completed ===")

if __name__ == '__main__':
    try:
        controller = NavigationController()
        controller.run_navigation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")