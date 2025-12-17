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
    """
    OCR 驱动节点 (OCR Driver Node)
    
    功能:
    1. 接收图像流并进行 OCR 文字识别。
    2. 响应 Vision Master 的控制指令 (start/stop)。
    3. 发布识别结果 (文本和边界框)。
    4. 提供调试用的 GUI 和控制台交互。
    """
    def __init__(self):
        rospy.init_node("ocr_node", anonymous=False)
        
        self.data_manager = DataManager()
        self.model_loader = OCRLoader(config)
        
        # 启动时预加载 OCR 模型 (Eager-load)
        self.model = None
        self.ensure_model_loaded()
        
        # ROS 初始化设置
        self.image_topic = rospy.get_param("~image_topic", config.DEFAULT_CONFIG["image_topic"])
        headless_param = rospy.get_param("~headless", config.DEFAULT_CONFIG["headless"])
        
        # 设置显示环境 (GUI)
        # 如果 headless=True，则不显示 OpenCV 窗口
        self.use_gui = setup_display_env(headless_param)
        
        # 驱动接口 (Driver Interface)
        # 订阅指令: "start" (启动识别) / "stop" (停止识别)
        self.driver_cmd_sub = rospy.Subscriber("/vision/driver/ocr/cmd", String, self.driver_cmd_cb)
        # 发布结果: JSON 字符串 {"texts": ["text1", ...], "boxes": [[[x1,y1],...], ...]}
        self.driver_res_pub = rospy.Publisher("/vision/driver/ocr/result", String, queue_size=1)
        
        self.img_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.snapshot_sub = rospy.Subscriber("/vision/snapshot", Image, self.snapshot_callback, queue_size=1, buff_size=2**24)
        
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.snapshot_frame = None
        self.current_frame_id = 0
        self.last_processed_id = -1
        
        # 运行模式: 
        # 0 = 冻结 (Freeze, 停止更新)
        # 1 = 暂停 (Pause, 显示原图但不处理)
        # 2 = 运行 (Run, 启用驱动逻辑)
        # 3 = 快照 (Snapshot)
        self.mode = 2
        
        # 驱动启用状态: 由 ROS 话题或手动控制台控制
        self.driver_enabled = False 
        self.is_ros_triggered = False
        self.best_score = -1
        self.task_session_id = 0
        
        self.start_console()
        
        # FPS 计算相关变量
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
        """确保模型已加载。如果未加载则尝试加载。"""
        if self.model is not None:
            return True
        self.model = self.model_loader.load_model()
        if self.model is None:
            rospy.logerr("[OCR Driver] OCR model load failed; disabling OCR.")
            self.driver_enabled = False
            return False
        return True

    def start_console(self):
        """启动控制台输入监听线程，用于手动调试。"""
        def input_loop():
            print("\n" + "="*40)
            print("       OCR Node (Driver Mode)")
            print("="*40)
            print(" [0] Freeze (冻结画面)")
            print(" [1] Pause (暂停处理，显示原图)")
            print(" [2] Auto (自动模式，监听 ROS 指令)")
            print(" [3] Force Start OCR (强制启动 OCR)")
            print(" [4] Force Stop OCR (强制停止 OCR)")
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
        """处理来自 Vision Master 的控制指令。"""
        cmd = msg.data.strip().lower()
        if cmd == "start":
            if self.ensure_model_loaded():
                self.driver_enabled = True
                self.is_ros_triggered = True
                self.mode = 2
                self.best_score = -1
                self.task_session_id = int(time.time() * 1000)
                rospy.loginfo("[OCR Driver] Enabled via ROS")
        elif cmd == "snapshot":
            if self.ensure_model_loaded():
                self.driver_enabled = True
                self.is_ros_triggered = True
                self.mode = 3
                # self.snapshot_frame = None # REMOVED: Do not clear snapshot here to avoid race condition
                rospy.loginfo("[OCR Driver] Snapshot mode armed")
        elif cmd == "stop":
            self.driver_enabled = False
            rospy.loginfo("[OCR Driver] Disabled via ROS")

    def image_callback(self, msg):
        """接收图像回调，将 ROS 图像转换为 OpenCV 格式。"""
        img = ImageProcessor.imgmsg_to_cv2(msg)
        if img is not None:
            with self.frame_lock:
                self.current_frame = img
                self.current_frame_id += 1

    def snapshot_callback(self, msg):
        """接收快照图像回调"""
        img = ImageProcessor.imgmsg_to_cv2(msg)
        if img is not None:
            with self.frame_lock:
                self.snapshot_frame = img
                rospy.loginfo("[OCR Driver] Snapshot received")

    def run(self):
        """主循环：处理图像、运行推理、发布结果、显示界面。"""
        rate = rospy.Rate(10) # OCR 不需要太高帧率
        while not rospy.is_shutdown():
            if self.mode == 0:
                rate.sleep()
                continue

            frame = None
            
            # Snapshot Mode
            if self.mode == 3:
                with self.frame_lock:
                    if self.snapshot_frame is not None:
                        frame = self.snapshot_frame.copy()
                        self.snapshot_frame = None
                    else:
                        rate.sleep()
                        continue
            else:
                # Normal Mode
                with self.frame_lock:
                    if self.current_frame is not None and self.current_frame_id > self.last_processed_id:
                        frame = self.current_frame.copy()
                        self.last_processed_id = self.current_frame_id
            
            if not self.use_gui and not (self.driver_enabled):
                rate.sleep()
                continue

            if frame is not None:
                display_img = frame.copy()
                inference_time = 0.0
                
                if self.driver_enabled:
                    t_start = time.time()
                    
                    # 执行 OCR 推理
                    # infer 返回: (texts, boxes, scores, timing_dict)
                    texts, boxes, scores, timing_dict = self.model.infer(frame)
                    inference_time = timing_dict.get('total', 0)
                    
                    # 发布结果
                    # Snapshot 模式下总是发布结果
                    if len(texts) > 0 or self.mode == 3:
                        # 确保数据可序列化 (numpy -> list)
                        serializable_boxes = [b.tolist() if hasattr(b, 'tolist') else b for b in boxes]
                        
                        result_data = {
                            "texts": texts,
                            "boxes": serializable_boxes,
                            "scores": scores,
                            "timing": timing_dict,
                            "width": frame.shape[1],
                            "height": frame.shape[0],
                            "timestamp": time.time()
                        }
                        self.driver_res_pub.publish(json.dumps(result_data))
                    
                    # 绘制结果
                    if self.use_gui or (config.SAVE_RESULT_IMAGES and self.is_ros_triggered):
                        display_img = ImageProcessor.draw_ocr_results(frame, texts, boxes, scores)
                        
                        # 保存最佳结果图片
                        if config.SAVE_RESULT_IMAGES and self.is_ros_triggered:
                            current_score = len(texts)
                            if self.mode == 3 or current_score > self.best_score:
                                self.best_score = current_score
                                try:
                                    if not os.path.exists(config.SAVE_RESULT_DIR):
                                        os.makedirs(config.SAVE_RESULT_DIR)
                                    
                                    filename = f"ocr_{self.task_session_id}_best.jpg"
                                    filepath = os.path.join(config.SAVE_RESULT_DIR, filename)
                                    cv2.imwrite(filepath, display_img)
                                    rospy.loginfo(f"Saved result image: {filepath}")
                                except Exception as e:
                                    rospy.logwarn(f"Failed to save image: {e}")
                
                # Snapshot 自动复位
                if self.mode == 3:
                    rospy.loginfo("[OCR Driver] Snapshot processed. Returning to Idle.")
                    self.mode = 0
                    self.driver_enabled = False

                # FPS 计算
                self.fps_frame_count += 1
                if time.time() - self.fps_start_time > 1.0:
                    self.current_fps = self.fps_frame_count / (time.time() - self.fps_start_time)
                    self.fps_frame_count = 0
                    self.fps_start_time = time.time()
                
                if self.use_gui:
                    mode_str = ["Freeze", "Pause", "Auto", "Snapshot"][self.mode]
                    status_str = "RUNNING" if self.driver_enabled else "STOPPED"
                    info_text = f"FPS: {self.current_fps:.1f} | {mode_str} | {status_str}"
                    cv2.putText(display_img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    cv2.imshow("OCR View", display_img)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
            
            rate.sleep()
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    OCRNode().run()
