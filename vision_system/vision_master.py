#!/usr/bin/env python3
"""
视觉主控节点 (Vision Master Node)
功能：
1. 启动并管理 YOLO 和 OCR 子进程（在各自 Conda 环境中）。
2. 作为 Nav 与视觉节点的中介，转发指令与结果。
3. 提供鲁棒性检查（子进程存活监控）。
4. 托管业务逻辑任务 (Logic Tasks)，调度底层驱动 (Drivers)。

话题接口：
- 订阅 (来自 Nav): /vision/cmd (String) -> "trigger <task> <seq>" 或 "stop <task>"
- 发布 (给 Nav): /vision/done (String) -> "done_<task> <result>"
- 发布 (给 Nav): /vision/status (String) -> "status_<task> <info>"

内部接口 (与子节点 Drivers):
- 发布: /vision/driver/yolo/cmd
- 订阅: /vision/driver/yolo/result
- 发布: /vision/driver/ocr/cmd
- 订阅: /vision/driver/ocr/result
"""

import os
import signal
import subprocess
import sys
import time
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Dict

import rospy
from std_msgs.msg import String
import config

# Import Logic Tasks
# Note: These tasks must be refactored to be Logic Controllers, not Image Processors
from tasks.plate import PlateTask
from tasks.fire import FireTask
from tasks.traffic_light import TrafficLightTask
from tasks.doll import DollTask
from tasks.garbage_controller import GarbageController
from tasks.fallen_vehicle_controller import FallenVehicleController
from tasks.sign import SignTask

# Absolute paths to node scripts
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
YOLO_SCRIPT = os.path.join(CURRENT_DIR, "yolo_node.py")
OCR_SCRIPT = os.path.join(CURRENT_DIR, "ocr_node.py")
VOICE_SCRIPT = os.path.join(CURRENT_DIR, "voice_node.py")

# Conda 环境名
YOLO_ENV = "yolo"
OCR_ENV = "ocr"
VOICE_ENV = "speak"

# 是否打开新终端
USE_TERMINAL = True

def detect_terminal_cmd():
    """
    检测系统可用的终端模拟器。
    如果找到 gnome-terminal 或 xterm，则返回相应的启动命令模板。
    否则回落到当前 shell 执行。
    """
    if not USE_TERMINAL:
        return ["bash", "-lc", "{cmd}"]

    candidates = [
        ["gnome-terminal", "--", "bash", "-lc", "{cmd}"],
        ["xterm", "-e", "bash", "-lc", "{cmd}"],
    ]
    for tmpl in candidates:
        try:
            if subprocess.call([tmpl[0], "--version"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
                return tmpl
        except Exception:
            continue
    print("[master] No GUI terminal found, running children inline in current shell.")
    return ["bash", "-lc", "{cmd}"]

TERM_CMD = detect_terminal_cmd()

@dataclass
class ProcSpec:
    """
    子进程规范类 (Process Specification)
    用于定义和管理一个子进程 (如 YOLO 节点, OCR 节点)。
    """
    name: str
    env_name: str
    script_path: str
    extra_args: List[str] = field(default_factory=list)
    process: Optional[subprocess.Popen] = None

    def build_command(self) -> List[str]:
        """构建启动命令，包含 Conda 环境激活。"""
        conda_setup = (
            "if [ -f \"$HOME/miniconda3/etc/profile.d/conda.sh\" ]; then "
            "source $HOME/miniconda3/etc/profile.d/conda.sh; "
            "elif [ -f \"$HOME/anaconda3/etc/profile.d/conda.sh\" ]; then "
            "source $HOME/anaconda3/etc/profile.d/conda.sh; "
            "else source ~/.bashrc; fi"
        )
        tail = ""
        # 如果使用 GUI 终端，添加 read line 以便在进程退出后保持窗口打开，方便查看日志
        if USE_TERMINAL:
            tail = "; echo 'Process exited. Press Enter to close...'; read line"
        cmd = (
            f"{conda_setup} && conda activate {self.env_name} "
            f"&& python {self.script_path} {' '.join(self.extra_args)}{tail}"
        )
        return [p.format(cmd=cmd) if "{cmd}" in p else p for p in TERM_CMD]

    def is_running(self) -> bool:
        """检查子进程是否正在运行。"""
        if self.process is None:
            return False
        if USE_TERMINAL:
            # 如果是新开终端，无法直接通过 Popen 对象判断 (因为 Popen 启动的是终端本身)
            # 这里简单返回 True，实际生产环境可能需要更复杂的 PID 检查
            return True 
        return self.process.poll() is None

    def start(self):
        """启动子进程。"""
        cmd = self.build_command()
        rospy.loginfo(f"[master] Starting {self.name} with env '{self.env_name}'")
        self.process = subprocess.Popen(cmd, start_new_session=True)

    def stop(self):
        """停止子进程。"""
        if self.process:
            rospy.loginfo(f"[master] Stopping {self.name}")
            try:
                pgid = os.getpgid(self.process.pid)
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                pass
            self.process = None

class VisionMasterNode:
    """
    视觉主控节点类。
    负责协调所有视觉相关的子系统和业务逻辑。
    """
    def __init__(self):
        rospy.init_node("vision_master", anonymous=False)
        
        # 1. 外部接口 (Nav <-> Master)
        # 接收导航节点的指令
        self.nav_cmd_sub = rospy.Subscriber("/vision/cmd", String, self.nav_cmd_cb)
        # 发送任务完成结果给导航节点
        self.nav_done_pub = rospy.Publisher("/vision/done", String, queue_size=5)
        # 发送任务状态给导航节点
        self.nav_status_pub = rospy.Publisher("/vision/status", String, queue_size=5)
        
        # 2. 业务逻辑任务池 (Logic Tasks)
        # 这些任务类是纯逻辑控制器，负责调度底层的 Driver (YOLO/OCR)
        self.logic_tasks = {
            "plate": PlateTask(),
            "fire": FireTask(),
            "traffic_light": TrafficLightTask(config.MODEL_LIST),
            "doll": DollTask(),
            "garbage": GarbageController(), 
            "fallen_vehicle": FallenVehicleController(),
            "sign": SignTask(),
        }
        self.active_task_name = None
        
        # 3. 子进程管理
        # 定义需要启动的子节点及其环境
        self.specs = {
            "yolo": ProcSpec(name="yolo", env_name=YOLO_ENV, script_path=YOLO_SCRIPT),
            "ocr": ProcSpec(name="ocr", env_name=OCR_ENV, script_path=OCR_SCRIPT),
            "voice": ProcSpec(name="voice", env_name=VOICE_ENV, script_path=VOICE_SCRIPT),
        }
        
        # 启动所有子进程
        for spec in self.specs.values():
            spec.start()
            time.sleep(1.0)

        # 启动监控线程，定期检查子进程是否存活
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        # 启动控制台，允许手动触发任务
        self.start_console()
        
        rospy.loginfo("[master] Vision Master Initialized.")

    def start_console(self):
        """启动控制台输入监听线程，用于手动调试。"""
        def input_loop():
            print("\n" + "="*40)
            print("       Vision Master Console")
            print("="*40)
            task_keys = list(self.logic_tasks.keys())
            for i, key in enumerate(task_keys):
                print(f" [{i+1}] Trigger Task: {key}")
            print(" [99] Stop Current Task")
            print("="*40)
            
            while not rospy.is_shutdown():
                try:
                    line = sys.stdin.readline()
                    if not line: break
                    line = line.strip()
                    if not line: continue
                    
                    try:
                        parts = line.split()
                        if not parts: continue

                        # Support typing task name directly
                        if parts[0] in self.logic_tasks:
                            task_name = parts[0]
                            seq_id = int(parts[1]) if len(parts) > 1 else 0
                            self.trigger_task(task_name, seq_id)
                            rospy.loginfo(f">>> Manual: Triggered {task_name} seq={seq_id}")
                            print(f"[Simulated ROS Cmd] /vision/cmd -> \"trigger {task_name} {seq_id}\"")
                            continue

                        idx = int(parts[0])
                        seq_id = int(parts[1]) if len(parts) > 1 else 0

                        if idx == 99:
                            if self.active_task_name:
                                print(f"[Simulated ROS Cmd] /vision/cmd -> \"stop {self.active_task_name}\"")
                                self.stop_task(self.active_task_name)
                                rospy.loginfo(">>> Manual: Stopped Task")
                        elif 1 <= idx <= len(task_keys):
                            task_name = task_keys[idx-1]
                            self.trigger_task(task_name, seq_id)
                            rospy.loginfo(f">>> Manual: Triggered {task_name} seq={seq_id}")
                            print(f"[Simulated ROS Cmd] /vision/cmd -> \"trigger {task_name} {seq_id}\"")
                        else:
                            print("Invalid selection")
                    except ValueError:
                        print("Please enter a number (e.g. '1' or '1 1') or valid task name")
                except Exception:
                    break
        
        t = threading.Thread(target=input_loop)
        t.daemon = True
        t.start()

    def nav_cmd_cb(self, msg):
        """处理来自 Nav 的指令。"""
        cmd_str = msg.data.strip()
        parts = cmd_str.split()
        if not parts: return
        
        action = parts[0] # trigger / stop
        
        if action == "trigger":
            if len(parts) < 2: return
            task_type = parts[1]
            seq_id = int(parts[2]) if len(parts) > 2 else 0
            self.trigger_task(task_type, seq_id)

        elif action == "stop":
            if len(parts) < 2: return
            task_type = parts[1]
            self.stop_task(task_type)

    def trigger_task(self, task_name, seq_id):
        """触发指定的逻辑任务。"""
        if task_name in self.logic_tasks:
            # 如果有正在运行的任务，先停止它
            if self.active_task_name and self.active_task_name != task_name:
                self.stop_task(self.active_task_name)
            
            self.active_task_name = task_name
            rospy.loginfo(f"[master] Starting Logic Task: {task_name} seq={seq_id}")
            # 启动任务，并传入回调函数用于汇报结果和状态
            self.logic_tasks[task_name].start(self.report_success, self.report_status, seq_id)
        else:
            rospy.logwarn(f"[master] Unknown task: {task_name}")

    def stop_task(self, task_name):
        """停止指定的逻辑任务。"""
        if task_name in self.logic_tasks:
            rospy.loginfo(f"[master] Stopping Logic Task: {task_name}")
            self.logic_tasks[task_name].stop()
            if self.active_task_name == task_name:
                self.active_task_name = None

    def report_success(self, result_str):
        """逻辑任务完成时的回调。"""
        rospy.loginfo(f"[master] Task Finished: {result_str}")
        self.nav_done_pub.publish(result_str)
        # 任务完成后是否自动停止？通常逻辑任务内部会停止自己，但我们需要在这里清除活跃状态
        if self.active_task_name:
             # 理想情况下，我们应该检查 result_str 是否匹配当前活跃任务
             pass

    def report_status(self, status_str):
        """逻辑任务状态汇报回调。"""
        # rospy.loginfo_throttle(1.0, f"[master] Task Status: {status_str}")
        self.nav_status_pub.publish(status_str)

    def monitor_loop(self):
        """监控子进程状态的循环。"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            for name, spec in self.specs.items():
                if not spec.is_running():
                    rospy.logwarn_throttle(5, f"[master] Child process {name} is not running!")
            rate.sleep()

    def run(self):
        """主运行循环。"""
        rospy.spin()
        # 节点关闭时，停止所有子进程
        for spec in self.specs.values():
            spec.stop()

if __name__ == "__main__":
    # 检查脚本是否存在
    for path in [YOLO_SCRIPT, OCR_SCRIPT, VOICE_SCRIPT]:
        if not os.path.exists(path):
            print(f"[master] Missing script: {path}")
            sys.exit(1)
            
    node = VisionMasterNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
