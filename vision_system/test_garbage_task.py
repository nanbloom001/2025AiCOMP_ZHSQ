#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import time
import sys
from std_msgs.msg import String

class GarbageTaskTester:
    def __init__(self):
        rospy.init_node("test_garbage_task", anonymous=False)
        
        # 1. 触发指令发布者
        self.cmd_pub = rospy.Publisher("/vision/cmd", String, queue_size=1)
        
        # 2. 监听最终结果
        self.done_sub = rospy.Subscriber("/vision/done", String, self.done_callback)
        
        # 3. 监听语音指令 (验证逻辑是否正确生成了语音数据)
        self.voice_sub = rospy.Subscriber("/vision/driver/voice/cmd", String, self.voice_callback)
        
        self.is_finished = False

    def done_callback(self, msg):
        print(f"\n✅ [Vision Master] 任务完成: {msg.data}")
        self.is_finished = True

    def voice_callback(self, msg):
        try:
            data = json.loads(msg.data)
            task = data.get("task")
            if task == "trash_bin":
                print(f"\n🔊 [Voice Logic] 捕获到垃圾桶语音指令:")
                print(json.dumps(data, indent=2, ensure_ascii=False))
            else:
                print(f"\n🔊 [Voice Logic] 捕获到其他语音指令: {task}")
        except:
            print(f"\n🔊 [Voice Logic] 收到非JSON语音指令: {msg.data}")

    def run(self):
        print("\n" + "="*60)
        print("       垃圾桶任务逻辑测试 (Garbage Task Tester)")
        print("       功能: 触发视觉 -> 监听语音生成 -> 等待任务结束")
        print("="*60)
        
        # 等待连接建立
        time.sleep(1.0)
        
        print(">>> 正在发送触发指令: 'trigger garbage 1' ...")
        self.cmd_pub.publish("trigger garbage 1")
        
        print(">>> 指令已发送，正在等待结果 (请将摄像头对准垃圾桶)...")
        
        # 等待循环
        start_time = time.time()
        while not rospy.is_shutdown() and not self.is_finished:
            if time.time() - start_time > 30:
                print("\n❌ [Timeout] 30秒内未收到完成信号。")
                break
            time.sleep(0.1)
            
        if self.is_finished:
            print("\n>>> 测试通过！逻辑流程正常。")
        else:
            print("\n>>> 测试结束 (未完成或超时)。")

if __name__ == "__main__":
    try:
        tester = GarbageTaskTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass
