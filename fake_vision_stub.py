#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
假视觉任务节点：
- 订阅 /navigation_task，收到 trigger_xxx 后立即发布对应的 *_task_done
- ocr/yolo/rog：发布字符串 "ok"
- traffic_light：发布字符串 "green_light"
"""
import rospy
from std_msgs.msg import String

class FakeVisionStub:
    def __init__(self):
        self.pub_ocr = rospy.Publisher("/ocr_task_done", String, queue_size=5)
        self.pub_yolo = rospy.Publisher("/yolo_task_done", String, queue_size=5)
        self.pub_tl = rospy.Publisher("/traffic_light_status", String, queue_size=5)
        self.sub = rospy.Subscriber("/navigation_task", String, self.task_cb)
        
        self.tl_timer = None
        self.tl_state_idx = 0
        # 模拟红绿灯序列: 红 -> 黄 -> 绿 -> 红 ...
        self.tl_sequence = ["red_light"] * 30 + ["yellow_light"] * 5 + ["green_light"] * 50
        
        rospy.loginfo("fake_vision_stub ready.")

    def tl_publish_loop(self, event):
        """定时发布红绿灯状态"""
        if self.tl_state_idx >= len(self.tl_sequence):
            self.tl_state_idx = 0
        
        status = self.tl_sequence[self.tl_state_idx]
        self.pub_tl.publish(String(status))
        self.tl_state_idx += 1

    def task_cb(self, msg: String):
        data = msg.data.strip().lower()
        
        if data == "trigger_ocr":
            self.pub_ocr.publish(String("ok"))
            rospy.loginfo("fake vision: ocr -> ok")
            
        elif data in ("trigger_rog", "trigger_yolo"):
            self.pub_yolo.publish(String("ok"))
            rospy.loginfo("fake vision: yolo/rog -> ok")
            
        elif data == "trigger_traffic_light":
            rospy.loginfo("fake vision: START traffic light detection loop")
            if self.tl_timer is None:
                self.tl_state_idx = 0
                # 10Hz 发布状态
                self.tl_timer = rospy.Timer(rospy.Duration(0.1), self.tl_publish_loop)
                
        elif data == "done_traffic_light":
            rospy.loginfo("fake vision: STOP traffic light detection loop")
            if self.tl_timer is not None:
                self.tl_timer.shutdown()
                self.tl_timer = None
                
        else:
            rospy.logwarn("fake vision: unknown task '%s'", data)

def main():
    rospy.init_node("fake_vision_stub")
    FakeVisionStub()
    rospy.spin()

if __name__ == "__main__":
    main()