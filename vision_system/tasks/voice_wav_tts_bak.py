import os
import sys
import time

# 动态添加路径以导入 voice_models 模块
current_dir = os.path.dirname(os.path.abspath(__file__))
voice_models_path = os.path.join(current_dir, "..", "voice_models")
if voice_models_path not in sys.path:
    sys.path.append(voice_models_path)

try:
    from audio_manager import AudioManager
    import sound_config
except ImportError:
    print("Error: Could not import audio_manager or sound_config. Check paths.")
    AudioManager = None

class VoiceWavTTS:
    """
    第二类：WAV + 实时 TTS 混合播报任务
    """
    def __init__(self):
        if AudioManager:
            self.player = AudioManager()
        else:
            self.player = None
            print("[VoiceWavTTS] Warning: AudioManager not initialized.")

    def _play_sequence(self, sequence, interval=None):
        """
        播放混合序列
        sequence: list of strings (WAV keys or TTS text) or numbers (delays)
        """
        if not self.player:
            print("[VoiceWavTTS] Player not available.")
            return

        print(f"🔊 [VoiceWavTTS] Playing sequence: {sequence}")
        self.player.broadcast_sequence(sequence, interval=interval)

    def _number_to_keys(self, number):
        """
        将数字转换为对应的音频 Key 列表 (支持 0-99 的中文读法)
        Example: 15 -> num_10, num_5
        Example: 25 -> num_2, num_10, num_5
        """
        if not isinstance(number, int):
            try:
                number = int(number)
            except:
                return []

        keys = []
        if number == 0:
             keys.append("0") # Use TTS for 0
        elif 1 <= number <= 10:
            keys.append(f"num_{number}")
        elif 11 <= number < 20:
            keys.append("num_10")
            keys.append(f"num_{number % 10}")
        elif 20 <= number <= 99:
            tens = number // 10
            units = number % 10
            keys.append(f"num_{tens}")
            keys.append("num_10")
            if units > 0:
                keys.append(f"num_{units}")
        else:
            # 超过 99 或负数，降级为按位读
            for digit in str(number):
                keys.append(f"num_{digit}")
        
        return keys

    # ==========================================
    # 具体任务播报逻辑
    # ==========================================

    def task_fire(self, result):
        """
        大楼火灾识别播报
        Sequence: Text(TTS) -> fire -> Num(WAV) -> fire2
        Input result example: {"text": "发现火情", "count": 1}
        """
        text = result.get("text", "发现火情")
        count = result.get("count", 0)

        sequence = []
        
        # 1. TTS Text
        sequence.append(text)
        
        # 2. fire
        sequence.append("fire")
        
        # 3. Num
        sequence.extend(self._number_to_keys(count))
        
        # 4. fire2
        sequence.append("fire2")

        self._play_sequence(sequence)

    def task_car_plate(self, result):
        """
        车牌识别区域播报
        Sequence: car1/car2/car3 -> Chinese(TTS) -> Letter(WAV)
        Input result example: {"parking_id": 1, "plate_chn": "京", "plate_eng": "A88888"}
        """
        parking_id = result.get("parking_id", 1)
        plate_chn = result.get("plate_chn", "")
        plate_eng = result.get("plate_eng", "")

        sequence = []
        
        # 1. carX
        sequence.append(f"car{parking_id}")
        
        # 2. Chinese (TTS)
        if plate_chn:
            sequence.append(plate_chn)
            
        # 3. Letter (WAV) - Split string into chars and map to keys
        for char in plate_eng:
            if char.isalpha():
                # Map 'A' -> 'char_a'
                key = f"char_{char.lower()}"
                sequence.append(key)
            elif char.isdigit():
                if char == '0':
                    sequence.append("0") # Use TTS for 0
                else:
                    # Map '1' -> 'num_1'
                    key = f"num_{char}"
                    sequence.append(key)
            else:
                # Fallback for other chars, might use TTS if not handled
                sequence.append(char)

        self._play_sequence(sequence, interval=0.05)

    def task_ocr(self, text):
        """
        通用 OCR 播报
        """
        if not text:
            return
        
        sequence = [
            "ocr_intro", 
            0.5,
            text
        ]
        self._play_sequence(sequence)

    def task_description(self, description):
        """
        场景描述播报
        """
        if not description:
            return
            
        sequence = [
            "desc_intro",
            0.5,
            description
        ]
        self._play_sequence(sequence)

    def task_custom_tts(self, text):
        """
        纯 TTS 播报
        """
        self._play_sequence([text])
