#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraSubscriber:
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node('camera_viewer_node', anonymous=True)

        # 2. 创建 CvBridge 对象 (用于 ROS <-> OpenCV 转换)
        self.bridge = CvBridge()

        # 3. 订阅话题
        # 话题名：/camera/color/image_raw
        # 消息类型：Image
        self.sub = rospy.Subscriber(
            "/camera/color/image_raw", 
            Image, 
            self.callback
        )
        
        print("正在等待相机图像...")

    def callback(self, msg):
        try:
            # 4. 将 ROS 消息转换为 OpenCV 格式 (bgr8)
            # 你的日志显示相机输出的是 MJPG，但在 ROS topic 中通常解码为 raw rgb/bgr
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # --- 在这里添加你的处理逻辑 (例如 YOLO 推理或拼接) ---
            
            # 5. 显示图像 (仅作测试用)
            cv2.imshow("Orbbec Camera Viewer", cv_image)
            
            # 按 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("用户退出")

        except CvBridgeError as e:
            print(f"转换错误: {e}")

if __name__ == '__main__':
    try:
        viewer = CameraSubscriber()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()