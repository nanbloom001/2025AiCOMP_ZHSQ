#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty
import json
import os
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import tf.transformations as t

# -------------------------------
# 全局变量
# -------------------------------
current_pose = None

# -------------------------------
# AMCL 回调
# -------------------------------
def amcl_cb(msg):
    global current_pose
    current_pose = msg.pose.pose


# -------------------------------
# 读取单个按键（非阻塞）
# -------------------------------
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


# -------------------------------
# 主程序
# -------------------------------
if __name__ == "__main__":
    rospy.init_node("teleop_and_collect")

    # 订阅 AMCL
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_cb)

    # 发布速度
    pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    twist = Twist()

    # 保存路径
    save_path = os.path.expanduser("/home/mowen/bit_ws/src/nav_demo/nav_point.json")
    points = []

    # 遥控参数
    speed = 0.25
    turn_speed = 1.0

    print("\n========== 遥控 + 采点工具 ==========")
    print("  W：前进       S：后退")
    print("  A：左转       D：右转")
    print("  空格：停止")
    print("  P：记录采点（AMCL）")
    print("  Q：退出程序")
    print("====================================\n")

    while not rospy.is_shutdown():
        key = get_key()

        # -------------------------------
        # 速度控制
        # -------------------------------
        if key == "w":
            twist.linear.x = speed
            twist.angular.z = 0.0

        elif key == "s":
            twist.linear.x = -speed
            twist.angular.z = 0.0

        elif key == "a":
            twist.linear.x = 0.0
            twist.angular.z = turn_speed

        elif key == "d":
            twist.linear.x = 0.0
            twist.angular.z = -turn_speed

        elif key == " ":
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # -------------------------------
        # 采点
        # -------------------------------
        elif key == 'p':
            if current_pose is None:
                print("[警告] AMCL 还未发布位姿")
                continue

            x = current_pose.position.x
            y = current_pose.position.y
            q = current_pose.orientation
            yaw = t.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            point = {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)}
            points.append(point)

            print(f"[记录成功] x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

            # 实时写入文件
            with open(save_path, "w") as f:
                json.dump(points, f, indent=2)

        # -------------------------------
        # 退出
        # -------------------------------
        elif key == "q":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub_vel.publish(twist)

            print("\n退出程序")
            break

        else:
            # 其他键停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # 发布速度
        pub_vel.publish(twist)

    print(f"\n采点数据已保存至：{save_path}\n")
