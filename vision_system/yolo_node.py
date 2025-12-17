#!/usr/bin/env python3
# 关键修复：onnxruntime 必须在 cv2 之前导入，以启用 OpenVINO 加速
import onnxruntime
import rospy
import cv2
import os
import threading
import sys
import time
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String

from core.yolo_loader import YOLOLoader
from core.data_manager import DataManager
from core.image_processor import ImageProcessor
from core.utils import setup_display_env
import config
from tasks.garbage import GarbageTask
from tasks.fallen_vehicle import FallenVehicleTask

class YOLONode:
    """
    YOLO 驱动节点 (YOLO Driver Node)
    
    功能:
    1. 加载并管理多个 YOLO 模型 (红绿灯, 娃娃, 火焰等)。
    2. 响应 Vision Master 的控制指令，动态切换当前活跃的模型。
    3. 接收图像流并进行推理。
    4. 发布检测结果 (类别, 置信度, 边界框)。
    """
    def __init__(self):
        rospy.init_node("yolo_node", anonymous=False)
        
        self.data_manager = DataManager()
        self.model_loader = YOLOLoader(config)
        
        # 加载所有 YOLO 模型 (驱动模式)
        # 我们预加载所有模型，但根据指令一次只运行一个，以节省资源并避免冲突
        # 将任务名称映射到配置中的模型名称
        self.models = {
            "traffic_light": self.model_loader.load_yolo_model("lights"),
            "doll": self.model_loader.load_yolo_model("people_v3_s"),
            "fire": self.model_loader.load_yolo_model("fire"),
            "garbage": GarbageTask(self.model_loader, self.data_manager), # 使用 Task 类封装复杂逻辑
            "fallen_vehicle": FallenVehicleTask(self.model_loader, self.data_manager),
        }
        
        self.active_model_name = None
        self.active_model = None
        
        # ROS 初始化设置
        self.image_topic = rospy.get_param("~image_topic", config.DEFAULT_CONFIG["image_topic"])
        headless_param = rospy.get_param("~headless", config.DEFAULT_CONFIG["headless"])
        
        # 设置显示环境 (GUI)
        self.use_gui = setup_display_env(headless_param)
        
        # 驱动接口 (Driver Interface)
        # 订阅指令: "start <model_name>" (启动指定模型) / "stop" (停止当前模型)
        self.driver_cmd_sub = rospy.Subscriber("/vision/driver/yolo/cmd", String, self.driver_cmd_cb)
        # 发布结果: JSON 字符串 {"class_ids": [...], "scores": [...], "boxes": [...]}
        self.driver_res_pub = rospy.Publisher("/vision/driver/yolo/result", String, queue_size=1)
        
        self.img_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.current_frame_id = 0
        self.last_processed_id = -1
        
        # 运行模式: 
        # 0 = 冻结 (Freeze)
        # 1 = 暂停 (Pause, 显示原图)
        # 2 = 运行 (Run, 启用驱动逻辑)
        self.mode = 2
        
        # 标记是否由 ROS 触发 (用于区分手动调试和自动运行)
        self.is_ros_triggered = False 
        self.best_score = -1.0
        self.task_session_id = 0
        
        self.start_console()
        
        # FPS 计算
        self.fps_start_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0.0
        
        if self.use_gui:
            try:
                cv2.namedWindow("YOLO View", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("YOLO View", 640, 480)
            except Exception as e:
                rospy.logwarn(f"GUI init failed: {e}")
                self.use_gui = False

    def _stop_active_model(self):
        """安全停止当前活跃的模型或任务。"""
        if self.active_model is not None and hasattr(self.active_model, "stop"):
            try:
                self.active_model.stop()
            except Exception as e:
                rospy.logwarn(f"Failed to stop active model: {e}")
        self.active_model = None
        self.active_model_name = None

    def _start_model_by_name(self, model_name: str, is_ros=False):
        """根据名称启动模型。"""
        model = self.models.get(model_name)
        if model is None:
            rospy.logwarn(f"[YOLO Driver] Model not loaded or unknown: {model_name}")
            return

        # 如果切换了模型，先停止前一个
        if self.active_model_name != model_name:
            self._stop_active_model()

        self.active_model_name = model_name
        self.active_model = model
        self.is_ros_triggered = is_ros
        self.best_score = -1.0
        self.task_session_id = int(time.time() * 1000)
        
        # 如果是 Task (BaseTask 子类)，调用 start()
        if hasattr(model, "start"):
            try:
                model.start(seq_id=0)
            except TypeError:
                # 兼容旧接口 start(seq_id)
                model.start(0)
        rospy.loginfo(f"[YOLO Driver] Started model: {model_name} (ROS Triggered: {is_ros})")

    def start_console(self):
        """启动控制台输入监听线程，用于手动调试。"""
        def input_loop():
            print("\n" + "="*40)
            print("       YOLO Node (Driver Mode)")
            print("="*40)
            print(" [0] Freeze (冻结画面)")
            print(" [1] Pause (暂停处理，显示原图)")
            print(" [2] Auto (自动模式，监听 ROS 指令)")
            print("-" * 20)
            
            model_keys = list(self.models.keys())
            for i, key in enumerate(model_keys):
                print(f" [{i+3}] Switch Model: {key}")
            print(" [99] Stop Model")
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
                        elif idx == 99:
                            self._stop_active_model()
                            rospy.loginfo(">>> Manual: Stop Model")
                        elif 3 <= idx < 3 + len(model_keys):
                            model_name = model_keys[idx-3]
                            self._start_model_by_name(model_name, is_ros=False)
                            self.mode = 2
                            rospy.loginfo(f">>> Manual: Switch Model to {model_name}")
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
        cmd = msg.data.strip()
        parts = cmd.split()
        action = parts[0].lower()
        
        if action == "start":
            if len(parts) > 1:
                model_name = parts[1]
                self._start_model_by_name(model_name, is_ros=True)
        elif action == "stop":
            self._stop_active_model()
            rospy.loginfo("[YOLO Driver] Stopped")

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
                
                # 如果有活跃模型且在运行模式，则执行推理
                # 使用局部引用避免与回调发生竞争条件
                current_model = self.active_model
                if self.mode == 2 and current_model:
                    t_start = time.time()
                    
                    # 特殊处理 Task 类 (GarbageTask, FallenVehicleTask)
                    # 检查是否有 'process' 方法或是否为 BaseTask 实例
                    if hasattr(current_model, 'process') and hasattr(current_model, 'stop'):
                        # Task.process 返回 (res_img, result_str)
                        res_img, result_str = current_model.process(frame)
                        display_img = res_img
                        inference_time = (time.time() - t_start) * 1000
                        
                        if result_str:
                            self.driver_res_pub.publish(result_str)

                        # 保存结果图片 (完整可视化)
                        if config.SAVE_RESULT_IMAGES and result_str:
                            try:
                                if not os.path.exists(config.SAVE_RESULT_DIR):
                                    os.makedirs(config.SAVE_RESULT_DIR)
                                
                                timestamp = int(time.time() * 1000)
                                filename = f"task_{self.active_model_name}_{timestamp}.jpg"
                                filepath = os.path.join(config.SAVE_RESULT_DIR, filename)
                                cv2.imwrite(filepath, res_img)
                            except Exception as e:
                                rospy.logwarn(f"Failed to save image: {e}")
                            
                    else:
                        # 标准 YOLO 模型
                        # infer 返回: res_img, timing_dict, (boxes, scores, class_ids)
                        _, timing_dict, (boxes, scores, class_ids) = current_model.infer(frame)
                        inference_time = timing_dict.get('total', 0)
                        
                        # 发布结果
                        if len(boxes) > 0:
                            res_data = {
                                "model": self.active_model_name,
                                "boxes": [b.tolist() for b in boxes] if hasattr(boxes, 'tolist') else boxes,
                                "scores": [float(s) for s in scores] if hasattr(scores, 'tolist') else scores,
                                "class_ids": [int(c) for c in class_ids] if hasattr(class_ids, 'tolist') else class_ids,
                                "timing": timing_dict
                            }
                            self.driver_res_pub.publish(json.dumps(res_data))
                            
                            if config.PRINT_TIMING_INFO:
                                print(f"[\033[35mYOLO Driver\033[0m] {self.active_model_name} timing: {json.dumps(timing_dict)} (ms)")
                        
                        # 可视化绘制
                        for i, box in enumerate(boxes):
                            class_id = int(class_ids[i])
                            score = float(scores[i])
                            
                            # 默认颜色
                            color = (0, 255, 0)
                            
                            # 针对娃娃/人物的特殊逻辑
                            if self.active_model_name == "doll":
                                # 假设有 class_names 属性
                                if hasattr(current_model, "class_names") and 0 <= class_id < len(current_model.class_names):
                                    class_name = current_model.class_names[class_id]
                                    # 逻辑: 社区人员 vs 非社区人员
                                    # 假设 'bad' 前缀表示非社区人员 (红色)，其他为社区人员 (绿色)
                                    if class_name.startswith("bad"):
                                        color = (0, 0, 255) # 红色
                                    else:
                                        color = (0, 255, 0) # 绿色
                                else:
                                    # 如果没有 class_names，默认绿色
                                    color = (0, 255, 0)
                            
                            cv2.rectangle(
                                display_img,
                                (int(box[0]), int(box[1])),
                                (int(box[2]), int(box[3])),
                                color,
                                3, # 粗线条
                            )
                            
                            if hasattr(current_model, "class_names") and 0 <= class_id < len(current_model.class_names):
                                class_name = current_model.class_names[class_id]
                            else:
                                class_name = str(class_id)
                            label = f"{class_name}: {score:.2f}"
                            
                            # 更大的字体
                            cv2.putText(
                                display_img,
                                label,
                                (int(box[0]), int(box[1] - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1.0, # 字体缩放
                                color,
                                2,
                            )

                        # 保存最佳结果图片 (仅在 ROS 触发时)
                        if config.SAVE_RESULT_IMAGES and self.is_ros_triggered and len(boxes) > 0:
                            # 计算当前帧得分 (例如最大置信度)
                            current_score = max(scores) if len(scores) > 0 else 0
                            
                            if current_score > self.best_score:
                                self.best_score = current_score
                                try:
                                    if not os.path.exists(config.SAVE_RESULT_DIR):
                                        os.makedirs(config.SAVE_RESULT_DIR)
                                    
                                    # 保存/覆盖本次会话的最佳图片
                                    filename = f"yolo_{self.active_model_name}_{self.task_session_id}_best.jpg"
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
                model_str = self.active_model_name if self.active_model_name else "NONE"
                info_text = f"FPS: {self.current_fps:.1f} | Latency: {inference_time:.1f}ms | {mode_str} | Model: {model_str}"
                cv2.putText(display_img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                if self.use_gui:
                    cv2.imshow("YOLO View", display_img)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
            
            rate.sleep()
        
        cv2.destroyAllWindows()

if __name__ == "__main__":
    YOLONode().run()
