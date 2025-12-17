#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import threading
import queue
import os
import sys
from std_msgs.msg import String

# Add current directory to path to ensure we can import modules if needed
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

try:
    from voice_models.voice_wav_only import VoiceWavOnly
    from voice_models.voice_wav_tts import VoiceWavTTS
    from voice_models.audio_manager import AudioManager
except ImportError as e:
    print(f"[VoiceNode] Error importing voice models: {e}")
    VoiceWavOnly = None
    VoiceWavTTS = None
    AudioManager = None

class VoiceNode:
    """
    语音合成节点 (Voice Node) - 重构版
    
    功能:
    1. 接收 ROS 指令 (/vision/driver/voice/cmd)。
    2. 使用 VoiceTaskDispatcher 逻辑分发任务。
    3. 调用 VoiceWavOnly 或 VoiceWavTTS 进行播放。
    4. 统一管理音频资源 (AudioManager)。
    """
    def __init__(self):
        rospy.init_node('voice_node', anonymous=False)
        rospy.loginfo("[VoiceNode] Initializing...")

        # 统一音频管理器 (Singleton)
        self.audio_manager = AudioManager() if AudioManager else None

        # 初始化处理器
        self.wav_handler = VoiceWavOnly(self.audio_manager) if VoiceWavOnly else None
        self.tts_handler = VoiceWavTTS(self.audio_manager) if VoiceWavTTS else None
        
        if not self.wav_handler or not self.tts_handler:
            rospy.logwarn("[VoiceNode] Handlers not fully initialized. Check imports.")

        # 消息队列
        self.msg_queue = queue.Queue()
        
        # 订阅与发布
        self.cmd_sub = rospy.Subscriber("/vision/driver/voice/cmd", String, self.cmd_callback)
        self.status_pub = rospy.Publisher("/vision/driver/voice/status", String, queue_size=5)
        
        # 工作线程
        self.worker_thread = threading.Thread(target=self.process_queue)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("[VoiceNode] Ready.")

    def shutdown_hook(self):
        """节点关闭时的清理"""
        rospy.loginfo("[VoiceNode] Shutting down...")
        if self.audio_manager:
            self.audio_manager.stop()

    def cmd_callback(self, msg):
        """处理 ROS 指令"""
        try:
            data = json.loads(msg.data)
            self.msg_queue.put(data)
            
            # 发送回执 (Receipt)
            # 告诉发送者：指令已收到，正在排队
            receipt = {
                "status": "received",
                "original_action": data.get("action"),
                "original_task": data.get("task"),
                "timestamp": rospy.get_time()
            }
            self.status_pub.publish(json.dumps(receipt))
            
        except json.JSONDecodeError:
            # 兼容旧格式：直接发送文本
            self.msg_queue.put({"action": "say", "text": msg.data})
            self.status_pub.publish(json.dumps({"status": "received", "text": msg.data}))

    def process_queue(self):
        """处理队列中的任务"""
        while not rospy.is_shutdown():
            try:
                task = self.msg_queue.get(timeout=1.0)
                self.dispatch_task(task)
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"[VoiceNode] Error processing task: {e}")

    def dispatch_task(self, data):
        """分发任务逻辑 (原 VoiceTaskDispatcher)"""
        action = data.get("action", "say")
        
        if action == "say":
            # 通用 TTS
            text = data.get("text", "")
            if self.tts_handler and text:
                self.tts_handler.task_custom_tts(text)
                
        elif action == "dispatch":
            # 任务分发模式 (Task Dispatch Mode)
            # 格式: {"action": "dispatch", "task": "fire", "data": {...}}
            task_name = data.get("task")
            result = data.get("data")
            
            rospy.loginfo(f"[VoiceNode] Dispatching task: {task_name}")
            
            if not task_name: return

            # --- 纯 WAV 任务 ---
            if task_name == "stacking":
                if self.wav_handler: self.wav_handler.task_stacking(result)
            elif task_name == "bikes":
                if self.wav_handler: self.wav_handler.task_bikes(result)
            elif task_name == "trash_bin":
                if self.wav_handler: self.wav_handler.task_trash_bin(result)
            elif task_name == "system":
                if self.wav_handler: self.wav_handler.task_system(result)
                
            # --- 混合 TTS 任务 ---
            elif task_name == "fire":
                if self.tts_handler: self.tts_handler.task_fire(result)
            elif task_name == "car_plate":
                if self.tts_handler: self.tts_handler.task_car_plate(result)
            elif task_name == "ocr":
                text = result if isinstance(result, str) else result.get("text")
                if self.tts_handler: self.tts_handler.task_ocr(text)
            elif task_name == "description":
                desc = result if isinstance(result, str) else result.get("description")
                if self.tts_handler: self.tts_handler.task_description(desc)
            elif task_name == "custom":
                text = result if isinstance(result, str) else result.get("text")
                if self.tts_handler: self.tts_handler.task_custom_tts(text)
            else:
                rospy.logwarn(f"[VoiceNode] Unknown task: {task_name}")
        
        # 兼容旧接口：直接通过 ID 判断任务类型 (如果 action 不是 dispatch)
        # 例如 garbage 任务发送的是 {"action": "say", "id": "garbage_direct", ...}
        elif action == "say" and "id" in data:
            cmd_id = data.get("id")
            if cmd_id == "garbage_direct":
                # Garbage 任务直接发送了完整的 TTS 文本，所以上面 action="say" 已经处理了
                pass

if __name__ == "__main__":
    try:
        VoiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
