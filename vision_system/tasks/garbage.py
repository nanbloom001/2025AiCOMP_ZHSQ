from .base_task import BaseTask
import time
import cv2
import rospy
import json
from std_msgs.msg import String
try:
    from config import BIN_CLASS_NAMES, SPECIFIC_TRASH_NAMES, TRASH_COMMON_NAMES, TRASH_MAPPING, PRINT_TIMING_INFO
except ImportError:
    from ..config import BIN_CLASS_NAMES, SPECIFIC_TRASH_NAMES, TRASH_COMMON_NAMES, TRASH_MAPPING, PRINT_TIMING_INFO

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
        self.done = False

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
            
        if self.done:
            return image, None

        rospy.loginfo("[GarbageTask] Processing single frame...")
        t0 = time.time()
        
        # 1. Detect Bins
        _, _, (bin_boxes, bin_scores, bin_class_ids) = self.bin_model.infer(image)
        t1 = time.time()
        
        raw_bins = []
        for box, score, cls_id in zip(bin_boxes, bin_scores, bin_class_ids):
            raw_bins.append({
                "cls": int(cls_id), 
                "box": box, 
                "name": self.bin_model.class_names[int(cls_id)]
            })
        raw_bins.sort(key=lambda x: x['box'][0]) # Sort by x1

        # 2. Detect Trash
        _, _, (trash_boxes, trash_scores, trash_class_ids) = self.trash_model.infer(image)
        t2 = time.time()
        
        # 调试日志：打印原始检测数量
        rospy.loginfo(f"[GarbageTask] Raw trash detection count: {len(trash_boxes)}")
        
        raw_trashes = []
        
        # 优化：按置信度排序并限制数量，防止误检过多导致超时
        # 即使只有4个真实垃圾，模型可能会检测出几十个低置信度的噪点
        # 如果不对数量和置信度做限制，会对每一个噪点进行分类推理，导致总耗时激增
        
        # 1. 组合数据
        trash_items = []
        if len(trash_boxes) > 0:
            for i in range(len(trash_boxes)):
                trash_items.append({
                    "box": trash_boxes[i],
                    "score": trash_scores[i] if len(trash_scores) > i else 0,
                    "cls": trash_class_ids[i] if len(trash_class_ids) > i else 0
                })
        
        # 2. 按置信度降序排序
        trash_items.sort(key=lambda x: x["score"], reverse=True)
        
        # 3. 截取前 N 个 (例如 6 个，略多于实际数量以防漏检)
        MAX_ITEMS = 6
        processed_count = 0
        inference_count = 0
        
        rospy.loginfo(f"[GarbageTask] Candidates before filter: {len(trash_items)}")
        
        valid_crops = []
        valid_items_meta = []

        for item in trash_items:
            if processed_count >= MAX_ITEMS:
                break
                
            # 4. 过滤低置信度 (例如 0.4)
            if item["score"] < 0.4:
                continue
                
            box = item["box"]
            x1, y1, x2, y2 = map(int, box)
            h, w = image.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            
            crop = image[y1:y2, x1:x2]
            if crop.size == 0: continue
            
            valid_crops.append(crop)
            valid_items_meta.append({
                "box": [x1, y1, x2, y2]
            })
            processed_count += 1

        # 批量推理
        if valid_crops:
            t_infer_start = time.time()
            # Check if infer_batch exists (backward compatibility)
            if hasattr(self.cls_model, 'infer_batch'):
                pred_indices = self.cls_model.infer_batch(valid_crops)
            else:
                # Fallback if not updated
                pred_indices = [self.cls_model.infer(c) for c in valid_crops]
            t_infer_end = time.time()
            
            inference_count = len(valid_crops)
            if PRINT_TIMING_INFO:
                print(f"[\033[33mGarbageTask\033[0m] Batch Classify {inference_count} items: {(t_infer_end - t_infer_start)*1000:.2f}ms")

            for i, pred_idx in enumerate(pred_indices):
                spec_name = SPECIFIC_TRASH_NAMES[pred_idx]
                meta = valid_items_meta[i]
                raw_trashes.append({
                    "name": spec_name, 
                    "cat_full": TRASH_MAPPING.get(spec_name, "未知"), 
                    "box": meta["box"]
                })
            
        t3 = time.time()
        rospy.loginfo(f"[GarbageTask] Processed trash items: {processed_count}, Inferences: {inference_count}")

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
                "trash_key": matched_trash["name"] if matched_trash else None,
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

        # 4. Draw Result
        res_img = image.copy()
        self.draw_scene(res_img, current_scene_pairs)
        
        # 5. Generate Result String
        voice_list = []
        for pair in current_scene_pairs:
            msg = f"{pair['bin_cat']}垃圾桶状态为{pair['bin_state']}"
            if pair['trash_name'] != "无":
                msg += f"，垃圾为{pair['trash_name']}，投放{pair['judgement']}"
            voice_list.append(msg)
        
        result_str = "识别完成，" + "，".join(voice_list) if voice_list else "未检测到有效组合"
        
        t4 = time.time()
        timing_info = {
            "bin_det": round((t1 - t0) * 1000, 2),
            "trash_det": round((t2 - t1) * 1000, 2),
            "classify": round((t3 - t2) * 1000, 2),
            "post": round((t4 - t3) * 1000, 2),
            "total": round((t4 - t0) * 1000, 2)
        }
        
        if PRINT_TIMING_INFO:
            print(f"[\033[33mGarbageTask Detail\033[0m] {json.dumps(timing_info)} (ms)")
        
        # 6. Send Voice Command
        rospy.loginfo(f"[GarbageTask] Sending voice: {result_str}")
        
        # 使用 VoiceTaskDispatcher 格式的数据
        # 构造符合 voice_wav_only.py 中 task_trash_bin 要求的列表数据
        voice_data_list = []
        for pair in current_scene_pairs:
            voice_data_list.append({
                "type": self.DISPLAY_MAP.get(pair['bin_cat'], 'other').lower(), # recycle, kitchen, harmful, other
                "action": self.DISPLAY_MAP.get(pair['bin_state'], 'close').lower(), # open, close
                "trash_name": pair['trash_key'] if pair['trash_key'] else None, # Use KEY (e.g. 00_clamp) not Chinese name
                "check": self.DISPLAY_MAP.get(pair['judgement'], 'wrong').lower() # correct -> right, wrong
            })
            # 修正 correct -> right 映射
            if voice_data_list[-1]["check"] == "correct":
                voice_data_list[-1]["check"] = "right"

        cmd = {
            "action": "dispatch",
            "task": "trash_bin",
            "data": voice_data_list
        }
        self.voice_pub.publish(json.dumps(cmd))
        
        # 7. Mark as Done
        self.done = True
        rospy.loginfo(f"[GarbageTask] Single inference done. Result: {result_str}")
        
        return res_img, f"done:{result_str} | timing={json.dumps(timing_info)}"

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

