import os
import sys

# 动态添加路径以导入同级模块
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

try:
    from voice_wav_only import VoiceWavOnly
    from wkn.src.nav_demo.vision_system.tasks.voice_wav_tts_bak import VoiceWavTTS
except ImportError as e:
    print(f"Error importing voice handlers: {e}")
    VoiceWavOnly = None
    VoiceWavTTS = None

class VoiceTaskDispatcher:
    """
    语音任务分发器
    根据任务类型将结果分发给不同的处理模块：
    1. VoiceWavOnly: 纯 WAV 拼接 (Stacking, Bikes, TrashBin)
    2. VoiceWavTTS:  WAV + TTS 混合 (Fire, CarPlate, OCR, Description, Custom)
    """
    def __init__(self):
        self.wav_handler = VoiceWavOnly() if VoiceWavOnly else None
        self.tts_handler = VoiceWavTTS() if VoiceWavTTS else None

    def dispatch(self, task_name, result):
        """
        分发任务
        :param task_name: 任务名称 (str)
        :param result: 任务结果数据 (dict or any)
        """
        print(f"📢 [VoiceDispatcher] Received task: {task_name}")

        if not task_name:
            return

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
            print(f"⚠️ [VoiceDispatcher] Unknown task: {task_name}")

if __name__ == "__main__":
    # 测试分发器
    dispatcher = VoiceTaskDispatcher()
    
    print("\n--- Testing Dispatcher (WAV Only) ---")
    dispatcher.dispatch("stacking", {"n1": 1, "n2": 2, "n3": 3, "n4": 4, "n5": 5})
    
    print("\n--- Testing Dispatcher (TTS) ---")
    dispatcher.dispatch("fire", {"text": "发现火情", "count": 2})
    dispatcher.dispatch("car_plate", {"parking_id": 2, "plate_chn": "沪", "plate_eng": "B12345"})
