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

class StitchingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("OpenStitching GUI 工具 (N100 优化版 - ROS)")
        self.root.geometry("1100x700")
        
        # === 路径配置 ===
        # 获取当前脚本所在目录，并创建 results 子目录
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
        self.captured_images = []  # 存储内存中的图像数据
        self.captured_paths = []   # 存储临时文件路径
        self.max_images = 3
        self.lock = threading.Lock()
        
        # === ROS 初始化 ===
        try:
            # 检查节点是否已经初始化，避免重复初始化
            if rospy.get_node_uri() is None:
                rospy.init_node('stitching_gui_node', anonymous=True)
            
            self.sub = rospy.Subscriber("/camera/color/image_raw", RosImage, self.image_callback)
            print("ROS 节点已初始化，正在订阅 /camera/color/image_raw")
        except Exception as e:
            messagebox.showerror("ROS Error", f"ROS 初始化失败: {e}")

        # === 布局 ===
        self._setup_ui()
        
        # === 启动显示刷新循环 ===
        self.update_ui_loop()

    def image_callback(self, msg):
        """ROS 图像回调"""
        try:
            cv_image = self.imgmsg_to_cv2(msg)
            if cv_image is not None:
                with self.lock:
                    self.current_frame = cv_image
        except Exception as e:
            print(f"图像转换错误: {e}")

    @staticmethod
    def imgmsg_to_cv2(img_msg):
        """ 
        参考 image_processor.py 的实现 
        将 ROS Image 消息转换为 OpenCV 图像 (numpy array)
        """
        dtype = np.uint8
        n_channels = 3
        
        if img_msg.encoding == "bgr8":
            pass
        elif img_msg.encoding == "rgb8":
            pass
        elif img_msg.encoding == "mono8":
            n_channels = 1
        else:
            pass

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
        # 1. 顶部：参数控制区
        control_frame = tk.LabelFrame(self.root, text="参数配置 (悬停查看说明)", padx=10, pady=10)
        control_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        # 参数：特征检测器
        tk.Label(control_frame, text="检测器(Detector):").grid(row=0, column=0, sticky="e")
        self.combo_detector = ttk.Combobox(control_frame, values=["orb", "sift", "brisk", "akaze"])
        self.combo_detector.current(0) # 默认 ORB (最快)
        self.combo_detector.grid(row=0, column=1, padx=5)

        # 参数：投影方式
        tk.Label(control_frame, text="投影方式(Warper):").grid(row=0, column=2, sticky="e")
        self.combo_warper = ttk.Combobox(control_frame, values=["plane", "spherical", "cylindrical", "fisheye"])
        self.combo_warper.current(0) # 默认 plane (平面)
        self.combo_warper.grid(row=0, column=3, padx=5)

        # 参数：搜索范围 (Range Width) - 核心优化
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

        # 2. 中部：图像显示区
        display_frame = tk.Frame(self.root)
        display_frame.pack(expand=True, fill=tk.BOTH, padx=10)

        # 左侧：摄像头实时画面
        self.lbl_video = tk.Label(display_frame, text="等待 ROS 图像...", bg="black", fg="white")
        self.lbl_video.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=5)

        # 右侧：已抓拍的缩略图
        gallery_frame = tk.Frame(display_frame, width=200, bg="#dddddd")
        gallery_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)
        
        tk.Label(gallery_frame, text="已抓拍队列", bg="#dddddd", font=("Arial", 12, "bold")).pack(pady=5)
        
        self.thumb_labels = []
        for i in range(3):
            lbl = tk.Label(gallery_frame, text=f"空位 {i+1}", width=20, height=8, bg="white", relief="sunken")
            lbl.pack(pady=10, padx=10)
            self.thumb_labels.append(lbl)

        # 3. 底部：操作按钮
        btn_frame = tk.Frame(self.root, pady=10)
        btn_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # 使用系统默认字体，避免 Arial 在部分 Linux 上中文乱码
        self.btn_capture = tk.Button(btn_frame, text="📸 抓拍 (0/3)", font=("", 14), bg="#e1f5fe", command=self.capture_frame)
        self.btn_capture.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=20)

        self.btn_stitch = tk.Button(btn_frame, text="🧩 开始拼接", font=("", 14), bg="#e8f5e9", state=tk.DISABLED, command=self.start_stitching_thread)
        self.btn_stitch.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=20)
        
        self.btn_reset = tk.Button(btn_frame, text="🗑️ 重置", font=("", 14), bg="#ffebee", command=self.reset_images)
        self.btn_reset.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=20)

    def update_ui_loop(self):
        """定时刷新界面显示"""
        if not self.is_running:
            return

        with self.lock:
            frame = self.current_frame
        
        if frame is not None:
            # 转为 RGB 供 tkinter 显示
            cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # 缩放以适应 GUI
            img_h, img_w = cv_img.shape[:2]
            scale = 640 / img_w
            dim = (640, int(img_h * scale))
            resized = cv2.resize(cv_img, dim)
            
            img_pil = Image.fromarray(resized)
            imgtk = ImageTk.PhotoImage(image=img_pil)
            
            self.lbl_video.config(image=imgtk)
            self.lbl_video.image = imgtk
        
        # 30ms 后再次调用 (约 33fps)
        self.root.after(30, self.update_ui_loop)

    def capture_frame(self):
        """抓拍当前帧"""
        if len(self.captured_images) >= self.max_images:
            return
            
        with self.lock:
            if self.current_frame is None:
                messagebox.showwarning("警告", "未接收到图像数据")
                return
            frame = self.current_frame.copy()

        # 1. 保存全分辨率图像
        idx = len(self.captured_images)
        filename = os.path.join(self.results_dir, f"temp_capture_{idx}.jpg")
        cv2.imwrite(filename, frame)
        
        self.captured_images.append(frame)
        self.captured_paths.append(filename)
        
        # 2. 更新右侧缩略图
        cv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(cv_img, (150, 100))
        img_pil = Image.fromarray(resized)
        imgtk = ImageTk.PhotoImage(image=img_pil)
        
        self.thumb_labels[idx].config(image=imgtk, text="")
        self.thumb_labels[idx].image = imgtk
        
        # 3. 更新按钮状态
        count = len(self.captured_images)
        self.btn_capture.config(text=f"📸 抓拍 ({count}/{self.max_images})")
        
        if count == self.max_images:
            self.btn_capture.config(state=tk.DISABLED)
            self.btn_stitch.config(state=tk.NORMAL, bg="#4caf50", fg="white")

    def reset_images(self):
        """清空重来"""
        self.captured_images = []
        self.captured_paths = []
        for lbl in self.thumb_labels:
            lbl.config(image="", text="空位", width=20, height=8)
        self.btn_capture.config(state=tk.NORMAL, text="📸 抓拍 (0/3)")
        self.btn_stitch.config(state=tk.DISABLED, bg="#e8f5e9", fg="black")

    def start_stitching_thread(self):
        """在独立线程中运行拼接，避免卡死 GUI"""
        self.btn_stitch.config(text="⏳ 处理中...", state=tk.DISABLED)
        threading.Thread(target=self.run_stitch).start()

    def run_stitch(self):
        try:
            # ==========================================
            # 核心部分：构建参数字典
            # ==========================================
            
            det_val = self.combo_detector.get()
            warp_val = self.combo_warper.get()
            range_str = self.combo_range.get()
            conf_val = float(self.entry_conf.get())
            crop_val = self.var_crop.get()
            
            range_val = 1 if "1" in range_str and "-1" not in range_str else -1

            settings = {
                "detector": det_val,
                "confidence_threshold": conf_val,
                "range_width": range_val,
                "warper_type": warp_val,
                "blender_type": "multiband",  
                "compensator": "gain_blocks",
                "crop": crop_val,
                "try_use_gpu": False,
            }

            print("开始拼接，参数配置:", settings)
            stitcher = Stitcher(**settings)
            
            panorama = stitcher.stitch(self.captured_paths)

            # 显示结果
            if panorama is not None:
                # 检查是否全黑
                if np.max(panorama) == 0:
                    self.root.after(0, lambda: messagebox.showwarning("警告", "拼接结果为全黑图像，可能是参数设置不当或图片无重叠。"))
                
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                output_file = os.path.join(self.results_dir, f"result_{timestamp}.jpg")
                cv2.imwrite(output_file, panorama)
                
                # 在主线程显示结果 (使用 Tkinter Toplevel)
                self.root.after(0, lambda: self.show_result(panorama, output_file))
            else:
                self.root.after(0, lambda: messagebox.showerror("失败", "拼接失败！\n请确保图片之间有足够的重叠区域，且纹理丰富。"))

        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("错误", f"程序出错: {str(e)}"))
        finally:
            self.root.after(0, lambda: self.btn_stitch.config(text="🧩 开始拼接", state=tk.NORMAL))

    def show_result(self, img, filepath):
        """使用 Tkinter Toplevel 显示结果，解决 OpenCV imshow 黑屏和乱码问题"""
        try:
            top = tk.Toplevel(self.root)
            top.title(f"拼接结果")
            
            # 转换颜色 BGR -> RGB
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            h, w = img_rgb.shape[:2]
            
            # 智能缩放，适应屏幕 (假设最大显示 1200x800)
            max_w, max_h = 1200, 800
            scale = min(max_w/w, max_h/h, 1.0)
            
            new_w = int(w * scale)
            new_h = int(h * scale)
            
            img_resized = cv2.resize(img_rgb, (new_w, new_h))
            img_pil = Image.fromarray(img_resized)
            img_tk = ImageTk.PhotoImage(img_pil)
            
            # 图片标签
            lbl_img = tk.Label(top, image=img_tk)
            lbl_img.image = img_tk # 保持引用防止被回收
            lbl_img.pack(padx=10, pady=10)
            
            # 信息标签
            info_text = f"保存路径: {filepath}\n原始尺寸: {w}x{h}"
            tk.Label(top, text=info_text, font=("Arial", 12), wraplength=600).pack(pady=10)
            
            # 打开文件夹按钮
            def open_folder():
                folder = os.path.dirname(filepath)
                os.system(f"xdg-open '{folder}'")
                
            tk.Button(top, text="📂 打开所在文件夹", command=open_folder, bg="#e1f5fe", font=("Arial", 12)).pack(pady=10)
            
        except Exception as e:
            messagebox.showerror("显示错误", f"无法显示结果: {e}")

    def on_close(self):
        self.is_running = False
        # 清理临时文件
        for f in self.captured_paths:
            if os.path.exists(f):
                try:
                    os.remove(f)
                except:
                    pass
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StitchingApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
