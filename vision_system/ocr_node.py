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
        
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.current_frame_id = 0
        self.last_processed_id = -1
        
        # 运行模式: 
        # 0 = 冻结 (Freeze, 停止更新)
        # 1 = 暂停 (Pause, 显示原图但不处理)
        # 2 = 运行 (Run, 启用驱动逻辑)
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
                self.best_score = -1
                self.task_session_id = int(time.time() * 1000)
                rospy.loginfo("[OCR Driver] Enabled via ROS")
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

    def run(self):
        """主循环：处理图像、运行推理、发布结果、显示界面。"""
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
                
                # 如果启用驱动且在运行模式，则执行推理
                if self.mode == 2 and self.driver_enabled:
                    if not self.ensure_model_loaded():
                        rate.sleep()
                        continue
                    t_start = time.time()
                    boxes, texts = self.model.infer(frame)
                    inference_time = (time.time() - t_start) * 1000
                    
                    # 发布结果
                    if texts:
                        # 确保 boxes 可 JSON 序列化 (numpy array -> list)
                        serializable_boxes = [b.tolist() if hasattr(b, 'tolist') else b for b in boxes]
                        res_data = {
                            "texts": texts,
                            "boxes": serializable_boxes
                        }
                        self.driver_res_pub.publish(json.dumps(res_data))

                    # 可视化绘制
                    for i, box in enumerate(boxes):
                        cv2.polylines(display_img, [box], True, (0, 255, 0), 2)
                        if i < len(texts):
                            text = texts[i]
                            text_x = min([p[0] for p in box])
                            text_y = max([p[1] for p in box])
                            text_position = (text_x, text_y + 5)
                            # 使用 PIL 绘制中文文本 (OpenCV 不支持中文)
                            display_img = draw_text_with_pil(display_img, text, text_position, font_size=40)

                    # 保存最佳结果图片 (仅在 ROS 触发时)
                    if config.SAVE_RESULT_IMAGES and self.is_ros_triggered and len(boxes) > 0:
                        # 评分标准: 检测到的文本块数量
                        current_score = len(boxes)
                        
                        if current_score > self.best_score:
                            self.best_score = current_score
                            try:
                                if not os.path.exists(config.SAVE_RESULT_DIR):
                                    os.makedirs(config.SAVE_RESULT_DIR)
                                
                                # 保存/覆盖最佳图片
                                filename = f"ocr_{self.task_session_id}_best.jpg"
                                filepath = os.path.join(config.SAVE_RESULT_DIR, filename)
                                cv2.imwrite(filepath, display_img)
                            except Exception as e:
                                rospy.logwarn(f"Failed to save image: {e}")
                
                # 计算 FPS
                self.fps_frame_count += 1
                if time.time() - self.fps_start_time > 1.0:
                    self.current_fps = self.fps_frame_count / (time.time() - self.fps_start_time)
                    self.fps_frame_count = 0
                    self.fps_start_time = time.time()
                
                # 绘制信息叠加层
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
