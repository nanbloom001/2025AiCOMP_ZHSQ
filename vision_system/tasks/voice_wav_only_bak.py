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

class VoiceWavOnly:
    """
    第一类：纯 WAV 文件拼接播报任务
    """
    def __init__(self):
        if AudioManager:
            self.player = AudioManager()
        else:
            self.player = None
            print("Warning: AudioManager not initialized, voice tasks will be silent.")

    def _play_sequence(self, keys):
        """按顺序播放 Key 列表"""
        if not self.player:
            return
        
        print(f"🔊 [VoiceWavOnly] Playing sequence: {keys}")
        self.player.broadcast_sequence(keys)

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
        if 1 <= number <= 10:
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

    def task_stacking(self, result):
        """
        街区任务播报
        Sequence: stack1 -> Num -> stack -> Num -> stack -> Num -> stack4 -> Num -> stack5_a -> Num -> stack5_b -> stack6
        Input result example: {"n1": 1, "n2": 2, "n3": 3, "n4": 4, "n5": 5}
        """
        n1 = result.get("n1", 0)
        n2 = result.get("n2", 0)
        n3 = result.get("n3", 0)
        n4 = result.get("n4", 0)
        n5 = result.get("n5", 0)

        sequence = []
        
        # stack1 -> Num
        sequence.append("stack1") 
        sequence.extend(self._number_to_keys(n1))
        
        # stack -> Num
        sequence.append("stack")
        sequence.extend(self._number_to_keys(n2))
        
        # stack -> Num
        sequence.append("stack")
        sequence.extend(self._number_to_keys(n3))

        # stack4 -> Num
        sequence.append("stack4")
        sequence.extend(self._number_to_keys(n4))

        # stack5_a -> Num
        sequence.append("stack5_a")
        sequence.extend(self._number_to_keys(n5))

        # stack5_b -> stack6
        sequence.append("stack5_b")
        sequence.append("stack6")

        self._play_sequence(sequence)

    def task_trash_bin(self, result):
        """
        垃圾桶任务播报 (支持单个或多个垃圾桶列表)
        Sequence per bin: bin_{type} -> bin_{action} -> bin_bridge -> trash_36_class -> bin_{check}
        Input result example (Single): {"type": "recycle", "action": "open", "trash_name": "apple", "check": "right"}
        Input result example (List): [
            {"type": "recycle", "action": "open", "trash_name": "orange", "check": "wrong"},
            {"type": "kitchen", "action": "open", "trash_name": "sugarcane", "check": "right"}
        ]
        """
        # 统一处理为列表
        items = result if isinstance(result, list) else [result]
        
        full_sequence = []
        
        for item in items:
            bin_type = item.get("type") # e.g., "recycle", "harmful", "kitchen", "other"
            action = item.get("action") # e.g., "open", "close"
            trash_name = item.get("trash_name") # e.g., "apple" or "08_orange"
            check = item.get("check")   # e.g., "right", "wrong"

            # 1. bin_{type}
            if bin_type:
                full_sequence.append(f"bin_{bin_type}")
            
            # 2. bin_{action}
            if action:
                full_sequence.append(f"bin_{action}")

            # 3. bin_bridge
            full_sequence.append("bin_bridge")

            # 4. trash_36_class
            if trash_name:
                # 尝试自动映射简单名称到 sound_config 中的 key
                mapped_key = trash_name
                if sound_config and hasattr(sound_config, 'SOUND_MAP'):
                    if trash_name not in sound_config.SOUND_MAP:
                        # 简单的模糊匹配
                        for key in sound_config.SOUND_MAP.keys():
                            if key.endswith(f"_{trash_name}") or key == trash_name:
                                mapped_key = key
                                break
                full_sequence.append(mapped_key)

            # 5. bin_{check}
            if check:
                full_sequence.append(f"bin_{check}")
            
            # 两个垃圾桶播报之间增加一点停顿
            full_sequence.append(0.5)

        self._play_sequence(full_sequence)

    def task_bikes(self, result):
        """
        电动车任务播报
        Sequence: 
          [bike_a1 + Num] OR [bike_a2]
          [bike_b1 + Num] OR [bike_b2]
          Num -> bike2 -> bike4 -> Num -> bike5 -> Num -> bike6
        Input result example: 
          {"illegal_a": 1, "illegal_b": 0, "n1": 10, "n2": 5, "n3": 3}
        """
        illegal_a = result.get("illegal_a", 0)
        illegal_b = result.get("illegal_b", 0)
        n1 = result.get("n1", 0) # First Num
        n2 = result.get("n2", 0) # Second Num (after bike4)
        n3 = result.get("n3", 0) # Third Num (after bike5)
        
        sequence = []

        # Part A
        if illegal_a > 0:
            sequence.append("bike_a1")
            sequence.extend(self._number_to_keys(illegal_a))
        else:
            sequence.append("bike_a2")

        # Part B
        if illegal_b > 0:
            sequence.append("bike_b1")
            sequence.extend(self._number_to_keys(illegal_b))
        else:
            sequence.append("bike_b2")
            
        # Remaining Sequence
        sequence.extend(self._number_to_keys(n1))
        sequence.append("bike2")
        sequence.append("bike4")
        sequence.extend(self._number_to_keys(n2))
        sequence.append("bike5")
        sequence.extend(self._number_to_keys(n3))
        sequence.append("bike6")

        self._play_sequence(sequence)

    def task_system(self, command):
        """系统指令播报"""
        if command == "stop":
            self._play_sequence(["stop"])
