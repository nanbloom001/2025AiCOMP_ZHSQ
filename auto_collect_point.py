#!/usr/bin/env python3
import rospy
import sys, termios, tty
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as t
import json
import os

current_pose = None

def amcl_cb(msg):
    global current_pose
    current_pose = msg.pose.pose

# 非阻塞读取按键
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

if __name__ == "__main__":
    rospy.init_node("auto_collect_point")

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_cb)

    save_path = os.path.expanduser("/home/mowen/bit_ws/src/nav_demo/nav_point.json")
    points = []

    print("\n==============================")
    print("   自动采点工具已启动")
    print("   小车开到某个位置 → 按 p 记录点")
    print("   按 q 退出")
    print("==============================\n")

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'p':
            if current_pose is None:
                print("[警告] 还没接收到 AMCL 位姿")
                continue

            x = current_pose.position.x
            y = current_pose.position.y

            q = current_pose.orientation
            yaw = t.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            point = {"x": x, "y": y, "yaw": yaw}
            points.append(point)

            print(f"[记录成功] x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

            # 写入文件
            with open(save_path, "w") as f:
                json.dump(points, f, indent=2)

        elif key == 'q':
            print("退出采点工具")
            break

    print(f"\n点已保存至：{save_path}\n")
