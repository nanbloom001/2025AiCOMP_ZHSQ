#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, termios, tty

# 读取单个按键（非阻塞）
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key

if __name__ == "__main__":
    rospy.init_node("my_keyboard_teleop")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    speed = 0.25        # 前后速度
    turn_speed = 1.0    # 旋转速度

    twist = Twist()

    print("\n===== 键盘遥控小车 =====")
    print("  W：前进")
    print("  S：后退")
    print("  A：左转")
    print("  D：右转")
    print("  空格：停止")
    print("  Q：退出程序")
    print("=======================\n")

    while not rospy.is_shutdown():
        key = get_key()

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

        elif key == "q":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
            print("退出遥控程序")
            break

        else:
            # 任意其他键都停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        pub.publish(twist)
