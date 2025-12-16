import os
import sys
import time
import random

# --- 环境设置 ---
# 将 tasks 目录加入路径，以便导入 VoiceTaskDispatcher
current_dir = os.path.dirname(os.path.abspath(__file__))
tasks_dir = os.path.join(current_dir, "tasks")
if tasks_dir not in sys.path:
    sys.path.append(tasks_dir)

try:
    from voice_task import VoiceTaskDispatcher
except ImportError as e:
    print(f"❌ 无法导入 VoiceTaskDispatcher: {e}")
    sys.exit(1)

# --- 模拟类定义 ---

class MockRobot:
    def __init__(self):
        self.current_pos = (0, 0)
        print("🤖 [Robot] 系统初始化完成")

    def navigate_to(self, x, y):
        print(f"\n🚀 [Robot] 正在前往目标点 ({x}, {y})...")
        # 模拟移动耗时
        time.sleep(1.5) 
        self.current_pos = (x, y)
        print(f"📍 [Robot] 到达目标点 ({x}, {y})")
        return True

class MockVisionSystem:
    def __init__(self):
        print("👁️ [Vision] 视觉系统初始化完成")

    def capture_image(self, image_path):
        """模拟拍照"""
        print(f"📸 [Vision] 正在采集图像...")
        if os.path.exists(image_path):
            print(f"   -> 读取文件: {image_path}")
            return True
        else:
            print(f"   -> ⚠️ 文件不存在: {image_path} (使用模拟数据继续)")
            return False

    def process_image(self, task_type, image_path):
        """
        模拟图像处理算法
        根据任务类型返回预设的模拟结果
        """
        print(f"⚙️ [Vision] 正在分析图像 (任务: {task_type})...")
        time.sleep(1.0) # 模拟计算耗时

        # --- 这里定义不同任务的模拟返回结果 ---
        
        if task_type == "stacking":
            # 模拟街区人数识别结果
            return {
                "n1": 15, # 测试 10+5 组合读法
                "n2": 8,
                "n3": 22, # 测试 20+2 组合读法
                "n4": 5,
                "n5": 105 # 测试 >99 读法
            }
            
        elif task_type == "fire":
            # 模拟火灾识别
            return {
                "text": "智慧大厦",
                "count": 3
            }
            
        elif task_type == "car_plate":
            # 模拟车牌识别
            return {
                "parking_id": 2,
                "plate_chn": "苏",
                "plate_eng": "E88888"
            }
            
        elif task_type == "trash_bin":
            # 模拟垃圾桶识别 (多桶顺序播报)
            return [
                {"type": "recycle", "action": "open", "trash_name": "orange", "check": "wrong"},
                {"type": "kitchen", "action": "open", "trash_name": "sugarcane", "check": "right"},
                {"type": "harmful", "action": "close", "trash_name": "nailpolish", "check": "right"},
                {"type": "other",   "action": "open", "trash_name": "brush", "check": "right"}
            ]
            
        elif task_type == "bikes":
            # 模拟电动车识别
            return {
                "illegal_a": 12,
                "illegal_b": 0,
                "n1": 5,
                "n2": 3,
                "n3": 8
            }
            
        return None

# --- 主流程 ---

def run_simulation():
    # 1. 初始化模块
    robot = MockRobot()
    vision = MockVisionSystem()
    dispatcher = VoiceTaskDispatcher()

    # 2. 定义任务列表 (模拟巡检路线)
    # 格式: (X坐标, Y坐标, 任务类型, 模拟图片路径)
    mission_list = [

        (2.0, 6.0, "car_plate",  "/home/mowen/wkn/src/nav_demo/vision_system/frame_0002_png.rf.2a48b3dc0a7d2e31bb82ef2c23d3642c.jpg"),
    
    ]

    print("\n" + "="*40)
    print("   开始全流程模拟测试")
    print("="*40)

    for x, y, task_type, img_path in mission_list:
        # A. 导航
        robot.navigate_to(x, y)

        # B. 视觉识别
        vision.capture_image(img_path)
        result = vision.process_image(task_type, img_path)
        
        print(f"📊 [Result] 识别结果: {result}")

        # C. 语音播报
        print(f"🎤 [Voice] 请求播报...")
        dispatcher.dispatch(task_type, result)
        
        print("-" * 30)
        time.sleep(2) # 任务间稍作停顿

    print("\n✅ 模拟测试结束")

if __name__ == "__main__":
    run_simulation()
