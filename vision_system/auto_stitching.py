# -*- coding: utf-8 -*-
import cv2
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
from stitching import Stitcher
import os
import numpy as np
import rospy
from sensor_msgs.msg import Image as RosImage
import tf
from geometry_msgs.msg import Twist
import math

class AutoStitchingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("自动全景拼接工具 (ROS TF版)")
        self.root.geometry("1100x750")
        
        # === 路径配置 ===
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.results_dir = os.path.join(self.script_dir, "results")
        try:
            os.makedirs(self.results_dir, exist_ok=True)
            print(f"图片保存路径: {self.results_dir}")
        except Exception as e:
            messagebox.showerror("Error", f"无法创建保存目录: {e}")

        # === 状态变量 ===
        self.is_running = True
        self.current_frame = None
        self.captured_images = []
        self.captured_paths = []
        self.max_images = 3
        self.lock = threading.Lock()
        self.capture_event = threading.Event()
        
        # 新增：往返模式方向状态 (1: 正向/逆时针, -1: 反向/顺时针)
        self.next_direction = 1
        
        # === ROS 初始化 ===
        try:
            if rospy.get_node_uri() is None:
                rospy.init_node('auto_stitching_node', anonymous=True)
            
            self.sub = rospy.Subscriber("/camera/color/image_raw", RosImage, self.image_callback)
            
            # 新增：TF监听器和速度发布者
            self.tf_listener = tf.TransformListener()
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            
            print("ROS 节点已初始化")
        except Exception as e:
            messagebox.showerror("ROS Error", f"ROS 初始化失败: {e}")

        # === 布局 ===
        self._setup_ui()
        
        # === 启动显示刷新循环 ===
        self.update_ui_loop()

    def image_callback(self, msg):
        try:
            cv_image = self.imgmsg_to_cv2(msg)
            if cv_image is not None:
                with self.lock:
                    self.current_frame = cv_image
        except Exception as e:
            print(f"图像转换错误: {e}")

    @staticmethod
    def imgmsg_to_cv2(img_msg):
        dtype = np.uint8
        n_channels = 3
        if img_msg.encoding == "mono8":
            n_channels = 1
        
        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')

        try:
            if n_channels == 3:
                img = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                                 dtype=dtype, buffer=img_msg.data)
            else:
                img = np.ndarray(shape=(img_msg.height, img_msg.width),
                                 dtype=dtype, buffer=img_msg.data)
        except Exception as e:
            return None

        if img_msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def _setup_ui(self):
        # 1. 参数控制区
        control_frame = tk.LabelFrame(self.root, text="参数配置", padx=10, pady=10)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        tk.Label(control_frame, text="检测器:").grid(row=0, column=0, sticky="e")
        self.combo_detector = ttk.Combobox(control_frame, values=["orb", "sift", "brisk", "akaze"])
        self.combo_detector.current(0)
        self.combo_detector.grid(row=0, column=1, padx=5)

        tk.Label(control_frame, text="投影方式:").grid(row=0, column=2, sticky="e")
        self.combo_warper = ttk.Combobox(control_frame, values=["plane", "spherical", "cylindrical", "fisheye"])
        self.combo_warper.current(2) # 默认 cylindrical
        self.combo_warper.grid(row=0, column=3, padx=5)

        # 参数：搜索范围
        tk.Label(control_frame, text="顺序优化(Range):").grid(row=0, column=4, sticky="e")
        self.combo_range = ttk.Combobox(control_frame, values=["1 (相邻比对/最快)", "-1 (全员比对/最慢)"])
        self.combo_range.current(0) 
        self.combo_range.grid(row=0, column=5, padx=5)
        
        # 参数：置信度
        tk.Label(control_frame, text="匹配阈值(Conf):").grid(row=0, column=6, sticky="e")
        self.entry_conf = tk.Entry(control_frame, width=5)
        self.entry_conf.insert(0, "0.1")
        self.entry_conf.grid(row=0, column=7, padx=5)

        # 参数：裁剪黑边
        self.var_crop = tk.BooleanVar(value=False)
        tk.Checkbutton(control_frame, text="裁剪黑边(Crop)", variable=self.var_crop).grid(row=0, column=8, padx=5)

        # Row 1: 自动控制参数
        tk.Label(control_frame, text="旋转角度(Deg):").grid(row=1, column=0, sticky="e", pady=5)
        self.entry_angle = tk.Entry(control_frame, width=5)
        self.entry_angle.insert(0, "20")
        self.entry_angle.grid(row=1, column=1, padx=5, pady=5)

        tk.Label(control_frame, text="旋转速度(Rad/s):").grid(row=1, column=2, sticky="e", pady=5)
        self.entry_speed = tk.Entry(control_frame, width=5)
        self.entry_speed.insert(0, "1")
        self.entry_speed.grid(row=1, column=3, padx=5, pady=5)

        # 参数：往返模式
        self.var_auto_reverse = tk.BooleanVar(value=True)
        tk.Checkbutton(control_frame, text="往返模式", variable=self.var_auto_reverse).grid(row=1, column=4, padx=5, pady=5)

        # Row 2: 性能优化参数
        tk.Label(control_frame, text="融合算法(Blend):").grid(row=2, column=0, sticky="e", pady=5)
        self.combo_blender = ttk.Combobox(control_frame, values=["multiband (最佳/慢)", "feather (均衡/快)", "no (最快/有缝)"])
        self.combo_blender.current(1) # 默认改为 feather 以提升速度
        self.combo_blender.grid(row=2, column=1, padx=5, pady=5)

        self.var_gpu = tk.BooleanVar(value=False)
        tk.Checkbutton(control_frame, text="尝试GPU加速", variable=self.var_gpu).grid(row=2, column=2, padx=5, pady=5)

        # 2. 图像显示区
        display_frame = tk.Frame(self.root)
        display_frame.pack(expand=True, fill=tk.BOTH, padx=10)

        self.lbl_video = tk.Label(display_frame, text="等待 ROS 图像...", bg="black", fg="white")
        self.lbl_video.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=5)

        gallery_frame = tk.Frame(display_frame, width=200, bg="#dddddd")
        gallery_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)
        
        self.thumb_labels = []
        for i in range(3):
            lbl = tk.Label(gallery_frame, text=f"空位 {i+1}", width=20, height=8, bg="white", relief="sunken")
            lbl.pack(pady=10, padx=10)
            self.thumb_labels.append(lbl)

        # 3. 操作按钮
        btn_frame = tk.Frame(self.root, pady=10)
        btn_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # 移除字体设置，使用系统默认，解决个别汉字不显示问题
        self.btn_capture = tk.Button(btn_frame, text="手动抓拍", bg="#e1f5fe", command=self.capture_frame)
        self.btn_capture.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=10)

        # 新增自动控制按钮
        self.btn_auto = tk.Button(btn_frame, text="自动旋转采集", bg="#fff9c4", command=self.start_auto_sequence)
        self.btn_auto.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=10)

        self.btn_stitch = tk.Button(btn_frame, text="开始拼接", bg="#e8f5e9", state=tk.DISABLED, command=self.start_stitching_thread)
        self.btn_stitch.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=10)
        
        self.btn_reset = tk.Button(btn_frame, text="重置", bg="#ffebee", command=self.reset_images)
        self.btn_reset.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=10)

    def update_ui_loop(self):
        if not self.is_running:
            return
        with self.lock:
            frame = self.current_frame
        if frame is not None:
            cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_h, img_w = cv_img.shape[:2]
            scale = 640 / img_w
            dim = (640, int(img_h * scale))
            resized = cv2.resize(cv_img, dim)
            img_pil = Image.fromarray(resized)
            imgtk = ImageTk.PhotoImage(image=img_pil)
            self.lbl_video.config(image=imgtk)
            self.lbl_video.image = imgtk
        self.root.after(30, self.update_ui_loop)

    def capture_frame(self):
        try:
            if len(self.captured_images) >= self.max_images:
                return
            with self.lock:
                if self.current_frame is None:
                    print("未接收到图像数据")
                    return
                frame = self.current_frame.copy()
                
                # 打印分辨率确认
                h, w = frame.shape[:2]
                print(f"已捕获图像 {len(self.captured_images)+1}/{self.max_images}, 分辨率: {w}x{h}")

            idx = len(self.captured_images)
            filename = os.path.join(self.results_dir, f"temp_capture_{idx}.jpg")
            cv2.imwrite(filename, frame)
            
            self.captured_images.append(frame)
            self.captured_paths.append(filename)
            
            cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(cv_img, (150, 100))
            img_pil = Image.fromarray(resized)
            imgtk = ImageTk.PhotoImage(image=img_pil)
            
            self.thumb_labels[idx].config(image=imgtk, text="")
            self.thumb_labels[idx].image = imgtk
            
            if len(self.captured_images) == self.max_images:
                self.btn_stitch.config(state=tk.NORMAL, bg="#4caf50", fg="white")
        finally:
            if hasattr(self, 'capture_event'):
                self.capture_event.set()

    def reset_images(self):
        self.captured_images = []
        self.captured_paths = []
        for lbl in self.thumb_labels:
            lbl.config(image="", text=f"空位", width=20, height=8)
        self.btn_stitch.config(state=tk.DISABLED, bg="#e8f5e9", fg="black")

    # === 自动控制逻辑 ===
    def get_yaw(self):
        """查询 odom 到 base_link 的变换获取当前偏航角 (使用 odom 防止 map 跳变)"""
        try:
            # 修改：将 /map 改为 /odom
            (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            return yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # 如果 odom 不存在，尝试回退到 map (兼容性)
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                _, _, yaw = tf.transformations.euler_from_quaternion(rot)
                return yaw
            except Exception as e2:
                print(f"TF Error: {e}")
                return None

    def rotate_robot(self, angle_deg, speed):
        """控制机器人旋转指定角度"""
        target_rad = math.radians(angle_deg)
        start_yaw = self.get_yaw()
        
        if start_yaw is None:
            print("无法获取初始位姿，尝试使用开环控制或等待...")
            # 如果无法获取TF，这里简单等待一下再试，或者直接返回失败
            return False

        print(f"开始旋转: 目标 {angle_deg} 度, 速度 {speed}")
        twist = Twist()
        twist.angular.z = speed if angle_deg > 0 else -speed
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
            current_yaw = self.get_yaw()
            if current_yaw is None: continue
            
            delta_yaw = current_yaw - start_yaw
            # 角度归一化处理
            while delta_yaw > math.pi: delta_yaw -= 2*math.pi
            while delta_yaw < -math.pi: delta_yaw += 2*math.pi
            
            if abs(delta_yaw) >= abs(target_rad):
                break
        
        # 停止
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5) # 等待停稳
        
        # 打印实际角度
        end_yaw = self.get_yaw()
        if end_yaw is not None:
            real_delta = end_yaw - start_yaw
            while real_delta > math.pi: real_delta -= 2*math.pi
            while real_delta < -math.pi: real_delta += 2*math.pi
            print(f"旋转结束: 实际旋转 {math.degrees(real_delta):.2f} 度")

        return True

    def start_auto_sequence(self):
        self.btn_auto.config(state=tk.DISABLED, text="运行中...")
        threading.Thread(target=self.run_auto_sequence).start()

    def run_auto_sequence(self):
        # 获取参数
        try:
            angle = float(self.entry_angle.get())
            speed = float(self.entry_speed.get())
        except ValueError:
            print("参数错误，使用默认值")
            angle = 30
            speed = 0.5

        # 往返模式逻辑
        direction = 1
        if self.var_auto_reverse.get():
            direction = self.next_direction
            print(f"往返模式: 本次方向 {'逆时针(+)' if direction > 0 else '顺时针(-)'}")
        
        effective_angle = angle * direction

        # 先重置
        self.root.after(0, self.reset_images)
        time.sleep(0.5)
        
        for i in range(3):
            if not self.is_running: break
            
            print(f"=== 步骤 {i+1}/3 ===")
            # 1. 旋转 (修改：第一个点不旋转)
            if i > 0:
                success = self.rotate_robot(effective_angle, speed)
                if not success:
                    print("旋转失败，终止自动流程")
                    break
                # time.sleep(1.0) # 等待图像稳定
            
            # 2. 拍照
            print("拍照...")
            self.capture_event.clear()
            self.root.after(0, self.capture_frame)
            self.capture_event.wait()
            # time.sleep(1.0) # 等待拍照完成
            
        # 序列结束后翻转方向
        if self.var_auto_reverse.get():
            self.next_direction *= -1

        print("自动采集完成，开始拼接...")
        self.root.after(0, self.start_stitching_thread)
        self.root.after(0, lambda: self.btn_auto.config(state=tk.NORMAL, text="自动旋转采集"))

    # === 拼接逻辑 ===
    def start_stitching_thread(self):
        self.btn_stitch.config(text="处理中...", state=tk.DISABLED)
        threading.Thread(target=self.run_stitch).start()

    def run_stitch(self):
        try:
            t_start = time.time()
            det_val = self.combo_detector.get()
            warp_val = self.combo_warper.get()
            range_str = self.combo_range.get()
            conf_val = float(self.entry_conf.get())
            crop_val = self.var_crop.get()
            
            # 解析融合算法
            blend_str = self.combo_blender.get()
            if "multiband" in blend_str: blender = "multiband"
            elif "feather" in blend_str: blender = "feather"
            else: blender = "no"

            range_val = 1 if "1" in range_str and "-1" not in range_str else -1

            settings = {
                "detector": det_val,
                "confidence_threshold": conf_val,
                "range_width": range_val,
                "warper_type": warp_val,
                "blender_type": blender,  
                "compensator": "gain_blocks", # 保持光照补偿，影响较小但对效果重要
                "crop": crop_val,
                "try_use_gpu": self.var_gpu.get(),
                # 优化：降低特征匹配分辨率以提速 (0.6 -> 0.4)，不影响最终画质
                "medium_megapix": 0.4,  
                "final_megapix": -1,    # 保持全分辨率输出
            }

            print("开始拼接...", settings)
            
            t_init = time.time()
            stitcher = Stitcher(**settings)
            print(f"[性能] 初始化耗时: {time.time() - t_init:.4f}s")
            
            t_stitch = time.time()
            panorama = stitcher.stitch(self.captured_paths)
            print(f"[性能] 核心拼接耗时: {time.time() - t_stitch:.4f}s")

            if panorama is not None:
                t_save = time.time()
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                output_file = os.path.join(self.results_dir, f"result_{timestamp}.jpg")
                cv2.imwrite(output_file, panorama)
                print(f"[性能] 保存图片耗时: {time.time() - t_save:.4f}s")
                
                self.root.after(0, lambda: self.show_result(panorama, output_file))
            else:
                self.root.after(0, lambda: messagebox.showerror("失败", "拼接失败！"))
            
            print(f"[性能] 总流程耗时: {time.time() - t_start:.4f}s")

        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("错误", f"程序出错: {str(e)}"))
        finally:
            self.root.after(0, lambda: self.btn_stitch.config(text="开始拼接", state=tk.NORMAL))

    def show_result(self, img, filepath):
        try:
            top = tk.Toplevel(self.root)
            top.title(f"拼接结果")
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            h, w = img_rgb.shape[:2]
            scale = min(1000/w, 700/h, 1.0)
            new_w, new_h = int(w * scale), int(h * scale)
            img_resized = cv2.resize(img_rgb, (new_w, new_h))
            img_pil = Image.fromarray(img_resized)
            img_tk = ImageTk.PhotoImage(img_pil)
            lbl = tk.Label(top, image=img_tk)
            lbl.image = img_tk
            lbl.pack(padx=10, pady=10)
            tk.Label(top, text=f"已保存: {filepath}").pack()
        except Exception as e:
            print(e)

    def on_close(self):
        self.is_running = False
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = AutoStitchingApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
