import os
import rospy
import subprocess
import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont

def draw_text_with_pil(image, text, position, font_path="/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", font_size=40, color=(0, 0, 255)):
    """使用PIL在OpenCV图像上安全地绘制中文文本。"""
    try:
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(rgb_image)
        draw = ImageDraw.Draw(pil_image)
        try:
            font = ImageFont.truetype(font_path, font_size)
        except IOError:
            # Fallback to default if specific font not found
            font = ImageFont.load_default()
            
        draw.text(position, text, font=font, fill=(color[2], color[1], color[0]))
        return cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
    except Exception as e:
        rospy.logwarn_once(f"使用PIL绘制文本时出错: {e}")
        return image

def setup_display_env(headless_param=False):
    """
    自动检测并设置 DISPLAY 环境变量。
    返回 use_gui (bool)
    """
    use_gui = True
    
    # 如果环境变量中没有 DISPLAY，尝试从 /tmp/.X11-unix 中自动检测并验证
    if "DISPLAY" not in os.environ:
        valid_display = None
        try:
            if os.path.exists("/tmp/.X11-unix"):
                # 查找所有 X socket 文件 (如 X0, X1003)
                x_sockets = [f for f in os.listdir("/tmp/.X11-unix") if f.startswith("X")]
                
                # 排序：优先尝试大号显示器 (如 NoMachine/VNC 的 :1000+)，最后尝试 :0
                # 这样可以确保远程连接的用户能看到窗口
                x_sockets.sort(key=lambda x: int(x.replace("X", "")) if x.replace("X", "").isdigit() else -1, reverse=True)
                
                rospy.loginfo(f"检测到潜在的 X11 socket (优先尝试远程): {x_sockets}，将依次尝试连接...")

                for socket_name in x_sockets:
                    display_num = socket_name.replace("X", "")
                    test_display = f":{display_num}"
                    
                    # 使用 xset -q 测试显示器连接是否有效
                    try:
                        test_env = os.environ.copy()
                        test_env["DISPLAY"] = test_display
                        ret = subprocess.call(["xset", "-q"], env=test_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        if ret == 0:
                            valid_display = test_display
                            rospy.loginfo(f"成功连接到显示器: {valid_display}")
                            break
                        else:
                            rospy.logwarn(f"尝试连接显示器 {test_display} 失败 (无法建立连接)")
                    except Exception as e:
                        rospy.logwarn(f"测试显示器 {test_display} 时发生错误: {e}")

        except Exception as e:
            rospy.logwarn(f"显示器自动检测流程出错: {e}")

        if valid_display:
            os.environ["DISPLAY"] = valid_display
            rospy.loginfo(f"已自动设置 DISPLAY={valid_display}")
        else:
            rospy.logwarn("未找到有效的显示器连接，将强制禁用 GUI (Headless Mode) 以防止崩溃。")
            use_gui = False
            os.environ["DISPLAY"] = ":0" 

    # 允许通过参数强制禁用 GUI
    if headless_param:
        use_gui = False
        rospy.loginfo("Headless 模式已启用 (参数控制)，将不显示 GUI 窗口。")
        
    return use_gui
