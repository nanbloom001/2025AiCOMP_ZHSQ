import rospy
import numpy as np
import cv2

class OCRModel:
    def __init__(self):
        # Import PaddleOCR
        try:
            from paddleocr import PaddleOCR 
        except ImportError as e:
            rospy.logerr(f"Failed to import PaddleOCR: {e}")
            rospy.logerr("Please ensure 'paddleocr' and 'paddlepaddle' are installed in the 'ocr' conda environment.")
            raise

        rospy.loginfo("Initializing PaddleOCR...")
        try:
            self.ocr = PaddleOCR(
                use_doc_orientation_classify=False,
                use_doc_unwarping=False,
                use_textline_orientation=False,
                device='cpu',
                enable_mkldnn=True,
                cpu_threads=4,
                text_det_limit_side_len=640,
                text_det_thresh=0.3,
                text_det_box_thresh=0.5,
                lang='ch',
                text_detection_model_name='PP-OCRv5_mobile_det',
                text_recognition_model_name='PP-OCRv5_mobile_rec',
                # show_log=False # Removed as it causes ValueError in newer PaddleOCR versions
            )
            rospy.loginfo("PaddleOCR initialized.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize PaddleOCR: {e}")
            raise

    def infer(self, image):
        import time
        t0 = time.time()
        
        # Logic adapted from working ocr_camera_inference.py
        h, w = image.shape[:2]
        # Optimize: Reduce target_width to improve speed (480 vs 640)
        # If detection is poor, increase this value.
        target_width = 480 
        scale = 1.0
        
        if w > target_width:
            scale = target_width / w
            new_h = int(h * scale)
            image_resized = cv2.resize(image, (target_width, new_h))
        else:
            image_resized = image

        t1 = time.time()
        # Use predict() as in the working code
        result_list = self.ocr.predict(image_resized)
        t2 = time.time()
        
        texts = []
        boxes = []
        scores = []
        
        if result_list and result_list[0] is not None:
            result_item = result_list[0]
            
            # Handle dict format
            if isinstance(result_item, dict) and 'rec_polys' in result_item:
                 polys = result_item['rec_polys']
                 rec_texts = result_item['rec_texts']
                 rec_scores = result_item['rec_scores']
                 
                 for poly, text, score in zip(polys, rec_texts, rec_scores):
                     if score >= 0.5: 
                         original_poly = (np.array(poly) / scale).astype(np.int32)
                         boxes.append(original_poly)
                         texts.append(text)
                         scores.append(float(score))
                         
            # Handle list format
            elif isinstance(result_item, list):
                for line in result_item:
                    # line[0] is box, line[1] is (text, score)
                    try:
                        poly = np.array(line[0])
                        text, score = line[1]
                        if score >= 0.5:
                            original_box = (poly / scale).astype(np.int32)
                            boxes.append(original_box)
                            texts.append(text)
                            scores.append(float(score))
                    except Exception as e:
                        rospy.logwarn(f"OCR parse error: {e}")
        
        t3 = time.time()
        timing = {
            "pre": round((t1-t0)*1000, 2),
            "infer": round((t2-t1)*1000, 2),
            "post": round((t3-t2)*1000, 2),
            "total": round((t3-t0)*1000, 2)
        }
        return texts, boxes, scores, timing

class OCRLoader:
    def __init__(self, config):
        self.config = config
        self.ocr_model = None

    def load_model(self, name="ocr"):
        # OCR model doesn't really use the config name the same way, but we keep the interface similar
        if self.ocr_model:
            return self.ocr_model
            
        try:
            self.ocr_model = OCRModel()
            return self.ocr_model
        except Exception as e:
            rospy.logerr(f"Failed to load OCR model: {e}")
            return None
