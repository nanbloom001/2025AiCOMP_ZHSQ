#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
from std_msgs.msg import String

def trigger_voice_stop():
    """
    触发语音播报：播放 stop.WAV
    """
    # 1. 初始化 ROS 节点
    rospy.init_node('voice_stop_trigger', anonymous=True)
    
    # 2. 创建发布者，指向 voice_node 订阅的话题
    pub = rospy.Publisher('/vision/driver/voice/cmd', String, queue_size=10)
    
    # 等待连接，确保消息能发布出去
    rospy.sleep(0.5)

    # 3. 构造指令数据
    # 根据 voice_node.py 的 dispatch_task 逻辑：
    # task_name == "system" 会调用 wav_handler.task_system
    # voice_wav_only.py 中的 task_system 会播放 ["stop"] 序列
    stop_cmd = {
        "action": "dispatch",
        "task": "system",
        "data": "stop"
    }

    # 4. 发布消息
    rospy.loginfo("Sending stop voice command...")
    pub.publish(json.dumps(stop_cmd))
    
    # 给一点时间让消息发出
    rospy.sleep(0.05)
    rospy.loginfo("Command sent.")

if __name__ == "__main__":
    try:
        trigger_voice_stop()
    except rospy.ROSInterruptException:
        pass