#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import queue
import threading
import time
import os
import re
import subprocess
import soundfile as sf
import numpy as np
from std_msgs.msg import String

# 尝试导入 sherpa_onnx
try:
    import sherpa_onnx
except ImportError:
    print("错误: 未安装 sherpa_onnx，请运行 pip3 install sherpa-onnx")
    exit(1)

# --- 配置部分 ---
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_ROOT = os.path.join(CURRENT_DIR, "voice_models", "tts_models")

# 单一中文模型 (Piper - Huayan)
ZH_MODEL_DIR = os.path.join(MODELS_ROOT, "vits-piper-zh_CN-huayan-medium")
ZH_CONFIG = {
    "model": os.path.join(ZH_MODEL_DIR, "zh_CN-huayan-medium.onnx"),
    "tokens": os.path.join(ZH_MODEL_DIR, "tokens.txt"),
    "data_dir": os.path.join(ZH_MODEL_DIR, "espeak-ng-data"),
}

OUTPUT_WAV = "/tmp/robot_voice.wav"

class VoiceNode:
    """
    语音合成节点 (Voice Node)
    
    功能:
    1. 接收文本指令并转换为语音 (TTS)。
    2. 使用 sherpa_onnx 进行离线语音合成。
    3. 支持队列管理、暂停、跳过和语速调整。
    4. 自动过滤非中文文本 (根据需求)。
    """
    def __init__(self):
        print("[VoiceNode] Initializing ROS node...")
        rospy.init_node('voice_node', anonymous=False)
        print("[VoiceNode] ROS node initialized.")
        
        # 1. 初始化 TTS 模型
        self.tts_engine = self.init_sherpa_model(ZH_CONFIG, "Piper-Huayan (ZH)")
        
        # 2. 初始化状态变量与消息队列
        self.msg_queue = queue.Queue()
        self.current_process = None
        self.is_speaking = False
        self.stop_flag = False
        self.default_speed = 1.5
        
        # 3. 定义 ROS 通信接口
        # 订阅指令: JSON 字符串 { "action": "say"|"stop"|"skip"|"speed", "text": "...", "speed": 1.0, "id": "..." }
        self.cmd_sub = rospy.Subscriber("/vision/driver/voice/cmd", String, self.cmd_callback)
        
        # 发布确认: JSON 字符串 { "id": "...", "status": "received" }
        self.ack_pub = rospy.Publisher("/vision/driver/voice/ack", String, queue_size=5)
        
        # 发布完成状态: JSON 字符串 { "id": "...", "status": "done"|"skipped"|"stopped" }
        self.done_pub = rospy.Publisher("/vision/driver/voice/done", String, queue_size=5)
        
        # 发布实时状态: JSON 字符串 { "state": "idle"|"speaking", "queue_len": 0 }
        self.status_pub = rospy.Publisher("/vision/driver/voice/status", String, queue_size=5)

        # 4. 启动语音合成工作线程
        self.worker_thread = threading.Thread(target=self.process_queue)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        # 5. 启动状态汇报线程
        self.status_thread = threading.Thread(target=self.status_loop)
        self.status_thread.daemon = True
        self.status_thread.start()
        
        self.start_console()
        rospy.loginfo(f"[VoiceNode] Started with single model: Piper-Huayan (ZH)")

    def init_sherpa_model(self, cfg, name):
        """初始化 sherpa_onnx TTS 模型。"""
        if not os.path.exists(cfg["model"]):
            rospy.logwarn(f"[VoiceNode] {name} model not found at: {cfg['model']}")
            return None
            
        try:
            rospy.loginfo(f"[VoiceNode] Loading {name}...")
            
            vits_config = sherpa_onnx.OfflineTtsVitsModelConfig(
                model=cfg["model"],
                tokens=cfg["tokens"],
                data_dir=cfg["data_dir"],
            )
            
            config = sherpa_onnx.OfflineTtsConfig(
                model=sherpa_onnx.OfflineTtsModelConfig(
                    vits=vits_config,
                    provider="cpu",
                    num_threads=1,
                    debug=False,
                ),
                rule_fsts="",
                max_num_sentences=1,
            )
            return sherpa_onnx.OfflineTts(config)
        except Exception as e:
            rospy.logerr(f"[VoiceNode] Failed to load {name}: {e}")
            return None

    # Removed old init_model
    
    def start_console(self):
        """启动控制台输入监听线程，用于手动调试。"""
        def input_loop():
            print("\n" + "="*40)
            print("       Voice Node Console")
            print("="*40)
            print(" [say <text>]  Speak text")
            print(" [stop]        Stop current & clear queue")
            print(" [skip]        Skip current sentence")
            print(" [speed <val>] Set speed (e.g. 1.0)")
            print("="*40)
            while not rospy.is_shutdown():
                try:
                    # Use input() which reads from stdin. 
                    # If run in background or without TTY, this might raise EOFError immediately.
                    try:
                        line = input()
                    except EOFError:
                        # If EOF (e.g. no TTY), just sleep to keep thread alive but do nothing
                        time.sleep(1)
                        continue
                        
                    if not line: continue
                    parts = line.strip().split()
                    cmd = parts[0]
                    
                    if cmd == "say":
                        text = " ".join(parts[1:])
                        self.handle_command({"action": "say", "text": text})
                    elif cmd == "stop":
                        self.handle_command({"action": "stop"})
                    elif cmd == "skip":
                        self.handle_command({"action": "skip"})
                    elif cmd == "speed":
                        if len(parts) > 1:
                            self.default_speed = float(parts[1])
                            print(f">>> Speed set to {self.default_speed}")
                except Exception:
                    pass
        t = threading.Thread(target=input_loop)
        t.daemon = True
        t.start()

    def cmd_callback(self, msg):
        """处理 ROS 指令回调。"""
        try:
            data = json.loads(msg.data)
            self.handle_command(data)
        except json.JSONDecodeError:
            # Fallback for plain text (legacy support)
            self.handle_command({"action": "say", "text": msg.data})

    def handle_command(self, data):
        """统一处理指令逻辑。"""
        action = data.get("action", "say")
        cmd_id = data.get("id", "unknown")
        
        if action == "say":
            text = data.get("text", "")
            if text:
                self.msg_queue.put(data)
                self.ack_pub.publish(json.dumps({"id": cmd_id, "status": "received"}))
                rospy.loginfo(f"[VoiceNode] Queued: {text[:20]}...")
                
        elif action == "stop":
            self.stop_flag = True
            self.kill_aplay()
            # Clear queue
            with self.msg_queue.mutex:
                self.msg_queue.queue.clear()
            rospy.loginfo("[VoiceNode] Stopped and queue cleared.")
            self.done_pub.publish(json.dumps({"id": cmd_id, "status": "stopped"}))
            
        elif action == "skip":
            self.kill_aplay()
            rospy.loginfo("[VoiceNode] Skipped current.")
            
        elif action == "speed":
            self.default_speed = float(data.get("speed", 1.0))
            rospy.loginfo(f"[VoiceNode] Speed updated to {self.default_speed}")

    def kill_aplay(self):
        """终止当前的音频播放进程。"""
        if self.current_process:
            try:
                self.current_process.terminate()
                self.current_process.wait(timeout=0.2)
            except Exception:
                pass
            self.current_process = None

    def process_queue(self):
        """工作线程：从队列获取文本并合成语音。"""
        while not rospy.is_shutdown():
            try:
                task = self.msg_queue.get(timeout=0.5)
                self.is_speaking = True
                self.stop_flag = False
                
                text = task.get("text", "")
                cmd_id = task.get("id", "unknown")
                speed = task.get("speed", self.default_speed)
                
                # --- 过滤逻辑: 如果不包含中文，则跳过 ---
                has_chinese = bool(re.search(r'[\u4e00-\u9fa5]', text))
                if not has_chinese:
                    rospy.logwarn(f"[VoiceNode] Skipped non-Chinese text: '{text}'")
                    self.done_pub.publish(json.dumps({"id": cmd_id, "status": "skipped"}))
                    self.is_speaking = False
                    continue

                if not self.tts_engine:
                    rospy.logerr("[VoiceNode] TTS engine not initialized!")
                    self.is_speaking = False
                    continue

                rospy.loginfo(f"[VoiceNode] Processing: '{text}' (Speed: {speed})")

                # 按逗号分割，实现自然停顿
                segments = re.split(r'[，,]', text)
                segments = [s.strip() for s in segments if s.strip()]
                
                all_segments_audio = []
                
                # 逐段生成音频
                for i, seg in enumerate(segments):
                    if self.stop_flag: break
                    
                    # 跳过纯英文片段 (根据需求)
                    if not bool(re.search(r'[\u4e00-\u9fa5]', seg)):
                        rospy.logwarn(f"[VoiceNode] Skipping English segment: '{seg}'")
                        continue

                    rospy.loginfo(f"[VoiceNode] Generating: '{seg}'")
                    try:
                        start_t = time.time()
                        audio = self.tts_engine.generate(seg, sid=0, speed=speed)
                        gen_time = time.time() - start_t
                        
                        if len(audio.samples) > 0:
                            all_segments_audio.append(audio.samples)
                            # 添加停顿
                            if i < len(segments) - 1:
                                pause_duration = 0.2
                                pause_samples = int(pause_duration * audio.sample_rate)
                                all_segments_audio.append(np.zeros(pause_samples, dtype=np.float32))
                        else:
                            rospy.logwarn(f"[VoiceNode] Empty audio for: '{seg}'")
                    except Exception as e:
                        rospy.logerr(f"[VoiceNode] Gen error: {e}")

                if self.stop_flag:
                    self.is_speaking = False
                    continue

                if all_segments_audio:
                    # 拼接所有片段
                    full_audio = np.concatenate(all_segments_audio)
                    
                    # 添加开始静音
                    silence_duration = 0.5
                    silence_samples = int(silence_duration * audio.sample_rate)
                    silence = np.zeros(silence_samples, dtype=np.float32)
                    final_samples = np.concatenate((silence, full_audio))
                    
                    rospy.loginfo(f"[VoiceNode] Playing ({len(final_samples)} samples)...")
                    sf.write(OUTPUT_WAV, final_samples, audio.sample_rate)
                    
                    # 调用 aplay 播放
                    self.current_process = subprocess.Popen(
                        ["aplay", "-q", OUTPUT_WAV],
                        stdout=subprocess.DEVNULL, 
                        stderr=subprocess.DEVNULL
                    )
                    self.current_process.wait()
                    self.current_process = None
                    
                    self.done_pub.publish(json.dumps({"id": cmd_id, "status": "done"}))
                else:
                    rospy.logwarn("[VoiceNode] No audio generated.")
                    self.done_pub.publish(json.dumps({"id": cmd_id, "status": "done"}))
                
                self.is_speaking = False
                
            except queue.Empty:
                self.is_speaking = False
                continue
            except Exception as e:
                rospy.logerr(f"[VoiceNode] Error: {e}")
                self.is_speaking = False

    def status_loop(self):
        """定期汇报状态。"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            status = {
                "state": "speaking" if self.is_speaking else "idle",
                "queue_len": self.msg_queue.qsize()
            }
            self.status_pub.publish(json.dumps(status))
            rate.sleep()

if __name__ == "__main__":
    try:
        VoiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"[VoiceNode] Critical Error: {e}")
