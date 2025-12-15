#!/usr/bin/env python3
import rospy
import cv2
import os
import threading
import sys
import time
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String

from core.ocr_loader import OCRLoader
from core.data_manager import DataManager
from core.image_processor import ImageProcessor
from core.utils import setup_display_env, draw_text_with_pil
import config

class OCRNode:
    def __init__(self):
        rospy.init_node("ocr_node", anonymous=False)
        
        self.data_manager = DataManager()
        self.model_loader = OCRLoader(config)
        
        # Eager-load OCR model at startup
        self.model = None
        self.ensure_model_loaded()
        
        # ROS setup
        self.image_topic = rospy.get_param("~image_topic", config.DEFAULT_CONFIG["image_topic"])
        headless_param = rospy.get_param("~headless", config.DEFAULT_CONFIG["headless"])
        
        # Setup display environment
        self.use_gui = setup_display_env(headless_param)
        
        # Driver Interface
        # cmd: "start" / "stop"
        self.driver_cmd_sub = rospy.Subscriber("/vision/driver/ocr/cmd", String, self.driver_cmd_cb)
        # result: JSON string {"texts": [...], "boxes": [...]}
        self.driver_res_pub = rospy.Publisher("/vision/driver/ocr/result", String, queue_size=1)
        
        self.img_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.current_frame_id = 0
        self.last_processed_id = -1
        
        # Modes: 0=Freeze, 1=Pause(Raw), 2=Run(Driver Enabled)
        self.mode = 2
        self.driver_enabled = False # Controlled by ROS topic or Manual
        self.is_ros_triggered = False
        self.best_score = -1
        self.task_session_id = 0
        
        self.start_console()
        
        # FPS calculation
        self.fps_start_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0.0
        
        if self.use_gui:
            try:
                cv2.namedWindow("OCR View", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("OCR View", 640, 480)
            except Exception as e:
                rospy.logwarn(f"GUI init failed: {e}")
                self.use_gui = False

    def ensure_model_loaded(self) -> bool:
        if self.model is not None:
            return True
        self.model = self.model_loader.load_model()
        if self.model is None:
            rospy.logerr("[OCR Driver] OCR model load failed; disabling OCR.")
            self.driver_enabled = False
            return False
        return True

    def start_console(self):
        def input_loop():
            print("\n" + "="*40)
            print("       OCR Node (Driver Mode)")
            print("="*40)
            print(" [0] Freeze (Stop updates)")
            print(" [1] Pause (Raw Image)")
            print(" [2] Auto (Listen to ROS)")
            print(" [3] Force Start OCR")
            print(" [4] Force Stop OCR")
            print("-" * 20)
            print("="*40)
            
            while not rospy.is_shutdown():
                try:
                    line = sys.stdin.readline()
                    if not line: break
                    line = line.strip()
                    if not line: continue
                    
                    try:
                        idx = int(line)
                        if idx == 0:
                            self.mode = 0
                            rospy.loginfo(">>> Mode: FREEZE")
                        elif idx == 1:
                            self.mode = 1
                            rospy.loginfo(">>> Mode: PAUSE (Raw Image)")
                        elif idx == 2:
                            self.mode = 2
                            rospy.loginfo(">>> Mode: AUTO (Listen to ROS)")
                        elif idx == 3:
                            self.mode = 2
                            self.driver_enabled = True
                            self.is_ros_triggered = False
                            rospy.loginfo(">>> Manual: Force Start OCR")
                        elif idx == 4:
                            self.mode = 2
                            self.driver_enabled = False
                            rospy.loginfo(">>> Manual: Force Stop OCR")
                        else:
                            print("Invalid selection")
                    except ValueError:
                        print("Please enter a number")
                except Exception:
                    break
        
        t = threading.Thread(target=input_loop)
        t.daemon = True
        t.start()

    def driver_cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            if self.ensure_model_loaded():
                self.driver_enabled = True
                self.is_ros_triggered = True
                self.best_score = -1
                self.task_session_id = int(time.time() * 1000)
                rospy.loginfo("[OCR Driver] Enabled via ROS")
        elif cmd == "stop":
            self.driver_enabled = False
            rospy.loginfo("[OCR Driver] Disabled via ROS")

    def image_callback(self, msg):
        img = ImageProcessor.imgmsg_to_cv2(msg)
        if img is not None:
            with self.frame_lock:
                self.current_frame = img
                self.current_frame_id += 1

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.mode == 0:
                rate.sleep()
                continue

            frame = None
            with self.frame_lock:
                if self.current_frame is not None and self.current_frame_id > self.last_processed_id:
                    frame = self.current_frame.copy()
                    self.last_processed_id = self.current_frame_id
            
            if frame is not None:
                display_img = frame.copy()
                inference_time = 0.0
                
                # Run Inference if Enabled
                if self.mode == 2 and self.driver_enabled:
                    if not self.ensure_model_loaded():
                        rate.sleep()
                        continue
                    t_start = time.time()
                    boxes, texts = self.model.infer(frame)
                    inference_time = (time.time() - t_start) * 1000
                    
                    # Publish Result
                    if texts:
                        # Ensure boxes are JSON serializable (convert numpy arrays to lists)
                        serializable_boxes = [b.tolist() if hasattr(b, 'tolist') else b for b in boxes]
                        res_data = {
                            "texts": texts,
                            "boxes": serializable_boxes
                        }
                        self.driver_res_pub.publish(json.dumps(res_data))

                    # Visualization
                    for i, box in enumerate(boxes):
                        cv2.polylines(display_img, [box], True, (0, 255, 0), 2)
                        if i < len(texts):
                            text = texts[i]
                            text_x = min([p[0] for p in box])
                            text_y = max([p[1] for p in box])
                            text_position = (text_x, text_y + 5)
                            # Use larger font size (default updated in utils.py or pass explicitly)
                            display_img = draw_text_with_pil(display_img, text, text_position, font_size=40)

                    # Save Best Result Image (With Boxes and Text)
                    if config.SAVE_RESULT_IMAGES and self.is_ros_triggered and len(boxes) > 0:
                        # Score = number of detected text blocks
                        current_score = len(boxes)
                        
                        if current_score > self.best_score:
                            self.best_score = current_score
                            try:
                                if not os.path.exists(config.SAVE_RESULT_DIR):
                                    os.makedirs(config.SAVE_RESULT_DIR)
                                
                                # Save/Overwrite best image
                                filename = f"ocr_{self.task_session_id}_best.jpg"
                                filepath = os.path.join(config.SAVE_RESULT_DIR, filename)
                                cv2.imwrite(filepath, display_img)
                            except Exception as e:
                                rospy.logwarn(f"Failed to save image: {e}")
                
                # Calculate FPS
                self.fps_frame_count += 1
                if time.time() - self.fps_start_time > 1.0:
                    self.current_fps = self.fps_frame_count / (time.time() - self.fps_start_time)
                    self.fps_frame_count = 0
                    self.fps_start_time = time.time()
                
                # Draw Info Overlay
                mode_str = ["Freeze", "Pause", "Auto"][self.mode]
                status_str = "RUNNING" if self.driver_enabled else "IDLE"
                info_text = f"FPS: {self.current_fps:.1f} | Latency: {inference_time:.1f}ms | {mode_str} | {status_str}"
                cv2.putText(display_img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                if self.use_gui:
                    cv2.imshow("OCR View", display_img)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
            
            rate.sleep()
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    OCRNode().run()
