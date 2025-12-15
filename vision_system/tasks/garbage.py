from .base_task import BaseTask
import time
import cv2
import rospy
import json
from std_msgs.msg import String
try:
    from config import BIN_CLASS_NAMES, SPECIFIC_TRASH_NAMES, TRASH_COMMON_NAMES, TRASH_MAPPING
except ImportError:
    from ..config import BIN_CLASS_NAMES, SPECIFIC_TRASH_NAMES, TRASH_COMMON_NAMES, TRASH_MAPPING

class GarbageTask(BaseTask):
    """
    垃圾分类任务 (Slave Side)。
    
    功能：
    1. 加载多个模型：垃圾桶检测 (best_bin)、垃圾检测 (best_trash)、分类模型 (best_classify)。
    2. 实时检测视野中的垃圾桶和垃圾。
    3. 判定垃圾类型（可回收、厨余、有害、其他）以及投放是否正确。
    4. 直接发布语音指令进行播报。
    """
    def __init__(self, model_loader, data_manager):
        super().__init__(model_loader, data_manager)
        # 加载模型
        self.bin_model = self.model_loader.load_model("best_bin")
        self.trash_model = self.model_loader.load_model("best_trash")
        self.cls_model = self.model_loader.load_model("best_classify")
        
        # 显示映射表 (中文 -> 英文)
        self.DISPLAY_MAP = {
            '可回收': 'Recycle', '厨余': 'Kitchen', '有害': 'Harmful', '其他': 'Other',
            '打开': 'Open', '关闭': 'Close', '正确': 'CORRECT', '错误': 'WRONG',
            '未知': 'Unknown', '未检测到垃圾': 'Empty', '无': 'None'
        }
        
        # 语音发布者
        self.voice_pub = rospy.Publisher("/vision/driver/voice/cmd", String, queue_size=1)
        
        self.reset_state()

    def start(self, seq_id):
        """启动任务。防止重复启动。"""
        # Prevent resetting if already active (debounce)
        if self.active:
            rospy.logwarn(f"[GarbageTask] Received start command but task is already active. Ignoring reset.")
            return
            
        super().start(seq_id)
        self.reset_state()
        rospy.loginfo("[GarbageTask] State reset. Starting observation...")

    def reset_state(self):
        """重置内部状态变量。"""
        self.start_time = None
        self.last_process_time = 0
        self.last_bin_detect_time = 0
        self.cached_bins = []
        self.locked = False
        self.best_scene = []
        self.max_items_count = 0
        self.observation_duration = 3.0
        self.voice_sent = False # Re-added this flag
        self.voice_sent_count = 0 
        
        # History for smart exit and mode calculation
        self.scene_history = [] 
        self.consistency_count = 0
        self.last_scene_signature = None

    def calculate_best_scene_from_history(self):
        if not self.scene_history: return
        from collections import Counter
        
        # Filter for complete scenes (4 bins)
        complete_scenes = [h for h in self.scene_history if len(h['scene']) == 4]
        target_list = complete_scenes if complete_scenes else self.scene_history
        
        if not target_list: return

        # Find mode of signatures
        counts = Counter(h['sig'] for h in target_list)
        if not counts: return
        
        most_common_sig = counts.most_common(1)[0][0]
        
        # Retrieve scene data
        for h in target_list:
            if h['sig'] == most_common_sig:
                self.best_scene = h['scene']
                break

    def get_display_text(self, cn_text):
        return self.DISPLAY_MAP.get(cn_text, "Obj")

    def parse_bin_info(self, class_name):
        parts = class_name.split('_')
        if len(parts) != 2: return "未知", "未知", (128,128,128)
        color_map = {'blue': '可回收', 'green': '厨余', 'red': '有害', 'grey': '其他'}
        state_map = {'open': '打开', 'close': '关闭'}
        # BGR 颜色
        cv_colors = {'blue': (255, 0, 0), 'green': (0, 255, 0), 'red': (0, 0, 255), 'grey': (128, 128, 128)}
        return color_map.get(parts[0], "未知"), state_map.get(parts[1], "未知"), cv_colors.get(parts[0], (0,255,255))

    def process(self, image):
        if not self.active:
            self.reset_state()
            return image, None
            
        if self.start_time is None:
            self.start_time = time.time()
            rospy.loginfo(f"[GarbageTask] First frame processed. Latency check.")
            
        current_time = time.time()
        
        # If locked, draw best scene and return result
        if self.locked:
            res_img = image.copy()
            self.draw_scene(res_img, self.best_scene)
            cv2.rectangle(res_img, (0, 0), (res_img.shape[1], 60), (0, 100, 0), -1)
            cv2.putText(res_img, "RESULT LOCKED", (20, 40), 0, 0.8, (255,255,255), 2)
            
            # Prepare result string (re-calculate or cache? Re-calc is safer for now as best_scene is fixed)
            voice_list = []
            target_scene = self.best_scene if self.best_scene else []
            for pair in target_scene:
                msg = f"{pair['bin_cat']}垃圾桶状态为{pair['bin_state']}"
                if pair['trash_name'] != "无":
                    msg += f"，垃圾为{pair['trash_name']}，投放{pair['judgement']}"
                voice_list.append(msg)
            result_str = "识别完成，" + "，".join(voice_list) if voice_list else "未检测到有效组合"

            # Send voice command multiple times (e.g., 5 frames) to ensure delivery
            if self.voice_sent_count < 5:
                self.voice_sent_count += 1
                self.voice_sent = True # Mark as sent
                rospy.loginfo(f"[GarbageTask] Sending voice (Attempt {self.voice_sent_count}/5): {result_str}")
                return res_img, f"voice:{result_str}"
            
            # Send DONE signal once after voice commands
            if self.voice_sent_count == 5:
                self.voice_sent_count += 1
                rospy.loginfo(f"[GarbageTask] Sending DONE signal: {result_str}")
                return res_img, f"done:{result_str}"
            
            return res_img, None

        # Observation phase
        elapsed = current_time - self.start_time
        remaining = max(0.0, self.observation_duration - elapsed)
        
        # --- Optimization 1: Frame Skipping (Max 5 FPS) ---
        if current_time - self.last_process_time < 0.2:
            # Skip inference, just draw previous result if available? 
            # Or just return original image.
            # For smooth visualization, we might want to draw the LAST known scene.
            if self.best_scene:
                res_img = image.copy()
                self.draw_scene(res_img, self.best_scene) # Draw best scene so far
                cv2.rectangle(res_img, (0, 0), (res_img.shape[1], 50), (50, 50, 50), -1)
                cv2.putText(res_img, f"SCANNING... {remaining:.1f}s (Skipped)", (20, 35), 0, 1, (0, 255, 255), 2)
                return res_img, None
            return image, None
            
        self.last_process_time = current_time

        # --- Optimization 2: Bin Detection Throttling (Every 1.0s) ---
        raw_bins = []
        if current_time - self.last_bin_detect_time > 1.0 or not self.cached_bins:
            # Run Bin Detection
            _, _, (bin_boxes, bin_scores, bin_class_ids) = self.bin_model.infer(image)
            for box, score, cls_id in zip(bin_boxes, bin_scores, bin_class_ids):
                raw_bins.append({
                    "cls": int(cls_id), 
                    "box": box, 
                    "name": self.bin_model.class_names[int(cls_id)]
                })
            raw_bins.sort(key=lambda x: x['box'][0]) # Sort by x1
            self.cached_bins = raw_bins
            self.last_bin_detect_time = current_time
        else:
            # Use Cached Bins
            raw_bins = self.cached_bins

        # 2. Detect Trash (Always run)
        _, _, (trash_boxes, trash_scores, trash_class_ids) = self.trash_model.infer(image)
        raw_trashes = []
        for box in trash_boxes:
            x1, y1, x2, y2 = map(int, box)
            # Ensure crop is within image bounds
            h, w = image.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            
            crop = image[y1:y2, x1:x2]
            if crop.size == 0: continue
            
            pred_idx = self.cls_model.infer(crop)
            spec_name = SPECIFIC_TRASH_NAMES[pred_idx]
            raw_trashes.append({
                "name": spec_name, 
                "cat_full": TRASH_MAPPING.get(spec_name, "未知"), 
                "box": [x1, y1, x2, y2]
            })

        # 3. Match Logic
        current_scene_pairs = []
        for bin_obj in raw_bins:
            b_cat, b_state, b_color = self.parse_bin_info(bin_obj["name"])
            
            matched_trash = None
            for t in raw_trashes:
                t_cx = (t["box"][0] + t["box"][2]) / 2
                t_cy = (t["box"][1] + t["box"][3]) / 2
                b_x1, b_y1, b_x2, b_y2 = bin_obj["box"]
                
                if b_x1 < t_cx < b_x2 and b_y1 < t_cy < b_y2:
                    matched_trash = t
                    break
            
            pair_data = {
                "bin_box": bin_obj["box"],
                "bin_cat": b_cat,
                "bin_state": b_state,
                "bin_color": b_color,
                "trash_box": matched_trash["box"] if matched_trash else None,
                "trash_name": TRASH_COMMON_NAMES.get(matched_trash["name"], matched_trash["name"]) if matched_trash else "无",
                "judgement": "未知"
            }

            if matched_trash:
                if b_cat in matched_trash["cat_full"]:
                    pair_data["judgement"] = "正确"
                else:
                    pair_data["judgement"] = "错误"
            else:
                pair_data["judgement"] = "未检测到垃圾"

            current_scene_pairs.append(pair_data)

        # 4. Update History & Check Consistency (Smart Exit)
        # Signature: (bin_cat, bin_state, trash_name, judgement) for each pair
        sig = tuple((p['bin_cat'], p['bin_state'], p['trash_name'], p['judgement']) for p in current_scene_pairs)
        self.scene_history.append({'sig': sig, 'scene': current_scene_pairs})
        
        # Strict Smart Exit Condition: 
        # 1. Must have 4 bins
        # 2. Each bin must have trash (trash_name != "无")
        bins_count = len(current_scene_pairs)
        trash_count = sum(1 for p in current_scene_pairs if p['trash_name'] != "无")
        
        is_complete = (bins_count == 4 and trash_count == 4)
        
        if is_complete:
            if sig == self.last_scene_signature:
                self.consistency_count += 1
            else:
                self.consistency_count = 1
            self.last_scene_signature = sig
            
            # Early Exit: 2 consecutive consistent frames with 4 bins
            if self.consistency_count >= 2:
                rospy.loginfo(f"[GarbageTask] Smart Exit: Consistent result found (Count: {self.consistency_count})")
                rospy.loginfo(f"[GarbageTask] Exit Signature: {sig}")
                self.best_scene = current_scene_pairs
                remaining = 0.0 # Trigger lock
        else:
            if bins_count == 4:
                 # Debug: Found 4 bins but signature changed
                 pass
            self.consistency_count = 0
            self.last_scene_signature = None

        # 5. Draw
        res_img = image.copy()
        self.draw_scene(res_img, current_scene_pairs)
        
        # UI
        cv2.rectangle(res_img, (0, 0), (res_img.shape[1], 50), (50, 50, 50), -1)
        cv2.putText(res_img, f"SCANNING... {remaining:.1f}s | Bins: {len(raw_bins)} | Trash: {trash_count}", (20, 35), 0, 1, (0, 255, 255), 2)

        if remaining == 0:
            self.locked = True
            
            # If not set by early exit, calculate mode from history
            if not self.best_scene:
                self.calculate_best_scene_from_history()
            
            # Prepare result string
            voice_list = []
            # Use best_scene if available, otherwise use current (fallback)
            target_scene = self.best_scene if self.best_scene else current_scene_pairs
            
            for pair in target_scene:
                # Always report bin status
                msg = f"{pair['bin_cat']}垃圾桶状态为{pair['bin_state']}"
                # If trash detected, append trash info
                if pair['trash_name'] != "无":
                    msg += f"，垃圾为{pair['trash_name']}，投放{pair['judgement']}"
                voice_list.append(msg)
            
            result_str = "识别完成，" + "，".join(voice_list) if voice_list else "未检测到有效组合"

            # Send voice command multiple times (e.g., 5 frames) to ensure delivery
            if self.voice_sent_count < 5:
                self.voice_sent_count += 1
                self.voice_sent = True
                rospy.loginfo(f"[GarbageTask] Sending voice (Attempt {self.voice_sent_count}/5): {result_str}")
                
                # Direct publish to Voice Node - Only send ONCE to avoid repetition
                if self.voice_sent_count == 1:
                    cmd = {
                        "action": "say",
                        "text": result_str,
                        "id": "garbage_direct",
                        "speed": 2
                    }
                    self.voice_pub.publish(json.dumps(cmd))
                
                return res_img, f"voice:{result_str}"
            
            # Send DONE signal once after voice commands to notify Master
            if self.voice_sent_count == 5:
                self.voice_sent_count += 1
                rospy.loginfo(f"[GarbageTask] Sending DONE signal: {result_str}")
                return res_img, f"done:{result_str}"
            
            return res_img, None
            
        return res_img, None

    def draw_scene(self, frame, pairs):
        for pair in pairs:
            bx1, by1, bx2, by2 = map(int, pair["bin_box"])
            color = pair["bin_color"]
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), color, 2)
            
            bin_label = f"{self.get_display_text(pair['bin_cat'])}-{self.get_display_text(pair['bin_state'])}"
            cv2.putText(frame, bin_label, (bx1, by1-10), 0, 0.6, color, 2)

            if pair["trash_box"] is not None:
                tx1, ty1, tx2, ty2 = map(int, pair["trash_box"])
                cv2.rectangle(frame, (tx1, ty1), (tx2, ty2), (0, 255, 255), 2)
                
                b_cx, b_cy = (bx1+bx2)//2, (by1+by2)//2
                t_cx, t_cy = (tx1+tx2)//2, (ty1+ty2)//2
                cv2.line(frame, (t_cx, t_cy), (b_cx, b_cy), (255,255,255), 1)

                res_eng = self.get_display_text(pair["judgement"])
                res_color = (0, 255, 0) if res_eng == 'CORRECT' else (0, 0, 255)
                
                cv2.putText(frame, res_eng, (tx1, ty1-5), 0, 0.8, res_color, 2)

