#!/usr/bin/env python
# coding:utf-8
import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class PoseMonitorSimple:
    def __init__(self):
        self.current_pose = None
        
    def amcl_callback(self, data):
        # 只更新数据，不输出
        pose = data.pose.pose
        x = pose.position.x
        y = pose.position.y
        
        # 四元数转欧拉角
        orientation = pose.orientation
        quaternion = [
            orientation.x,
            orientation.y, 
            orientation.z,
            orientation.w
        ]
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]  # 偏航角
        
        self.current_pose = (x, y, yaw)
    
    def timer_callback(self, event):
        if self.current_pose is not None:
            x, y, yaw = self.current_pose
            yaw_deg = math.degrees(yaw)
            
            # 使用回车符覆盖上一行输出
            print(f"\r机器人位置 - X: {x:.3f}m, Y: {y:.3f}m, 朝向: {yaw_deg/180*math.pi:.3f}", end="", flush=True)
    
    def start_monitoring(self):
        rospy.init_node('pose_monitor', anonymous=True)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        # 创建定时器，每2秒输出一次
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        print("开始监控机器人位置（每2秒更新一次）...")
        print("按 Ctrl+C 停止")
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = PoseMonitorSimple()
        monitor.start_monitoring()
    except rospy.ROSInterruptException:
        print("\n监控结束")

# goals = [
#     [1.200, 0.000, 0.000],  #红绿灯
#     [3.300, 0.000, 0.000],
#     [3.400, 0.000, 1.577],
#     [3.389, 1.064, 1.577],
#     [3.389, 1.064, 3.141],
#     [2.688, 1.048, 3.141],
#     [2.707, 1.043, 1.577],
#     [2.771, 1.088, -1.577],
#     [2.781, 1.083, 3.141],
#     [1.875, 1.140, 3.141],
#     [1.868, 1.140, 1.577],
#     [3.389, 1.064, 1.577],
#     [1.943, 1.710, 1.577], #红绿灯
#     [1.900, 3.173, 1.577],
#     [1.989, 3.169, 3.141],
#     [0.827, 3.287, 3.141],
#     [0.745, 3.090, -1.577],
#     [0.646, 0.095, -1.577],
#     [0.642, -.022, 0.000],
#     [0.047, -0.046,0.000],
# ]  