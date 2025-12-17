"""导航管理器的配置和状态原语。"""

from dataclasses import dataclass
from typing import Optional
import json
import rospy


@dataclass
class Task:
    """
    单个任务定义。
    
    Attributes:
        type (str): 任务类型 (例如: 'doll', 'fire', 'plate').
        seq_id (int): 任务序列号 (例如: 人偶计数累加).
    """
    type: str
    seq_id: int = 0


@dataclass
class Goal:
    """
    导航目标点数据结构。
    
    Attributes:
        x (float): 目标点的 x 坐标。
        y (float): 目标点的 y 坐标。
        yaw (float): 目标点的偏航角（弧度）。
        tasks (List[Task]): 到达目标点后触发的任务列表。
        nav_timeout (Optional[float]): 该目标点的特定导航超时时间（秒）。如果为 None，则使用默认配置。
    """
    x: float
    y: float
    yaw: float
    tasks: list = None
    nav_timeout: Optional[float] = None

    def __post_init__(self):
        if self.tasks is None:
            self.tasks = [Task("none", 0)]

    @property
    def task_type(self) -> str:
        """Primary task type for logging/backward-compat calls."""
        if self.tasks and len(self.tasks) > 0 and isinstance(self.tasks[0], Task):
            return self.tasks[0].type
        return "none"


@dataclass
class NavConfig:
    """
    导航配置参数。
    
    Attributes:
        default_nav_timeout (float): 默认导航超时时间（秒）。
        default_task_timeout (float): 默认任务等待超时时间（秒）。
        post_task_pause (float): 任务完成后的暂停时间（秒）。
        startup_delay (float): 启动延迟时间（秒）。
        keyboard_enabled (bool): 是否启用键盘控制。
    """
    default_nav_timeout: float = 60.0
    default_task_timeout: float = 15.0
    post_task_pause: float = 0.0
    startup_delay: float = 5.0
    keyboard_enabled: bool = True


@dataclass
class NavState:
    """
    导航管理器的运行时状态。
    
    Attributes:
        current_goal_index (int): 当前正在执行的目标点索引。
        paused (bool): 是否处于暂停状态。
        skip_requested (bool): 是否请求跳过当前目标/任务。
        shutdown_requested (bool): 是否请求关闭程序。
        executing_nav (bool): 是否正在执行导航动作。
        task_completed (bool): 当前任务是否已完成（由回调设置）。
        task_result (Optional[str]): 任务完成后的结果数据。
    """
    current_goal_index: int = 0
    paused: bool = False
    skip_requested: bool = False
    shutdown_requested: bool = False
    executing_nav: bool = False
    current_task: str = "none"
    task_completed: bool = False
    task_result: Optional[str] = None


def load_config_from_ros() -> NavConfig:
    """从 ROS 私有命名空间读取配置参数。"""
    return NavConfig(
        default_nav_timeout=rospy.get_param("~default_nav_timeout", 25.0),
        default_task_timeout=rospy.get_param("~default_task_timeout", 15.0),
        post_task_pause=rospy.get_param("~post_task_pause", 0.0),
        startup_delay=rospy.get_param("~startup_delay", 5.0),
        keyboard_enabled=rospy.get_param("~keyboard_enabled", True),
    )


def apply_param_update(cfg: NavConfig, raw: str) -> None:
    """通过来自 /nav_ctrl/param_update 的 JSON 字符串更新配置。"""
    data = json.loads(raw)
    for key, val in data.items():
        if key == "default_nav_timeout":
            cfg.default_nav_timeout = float(val)
        elif key == "default_task_timeout":
            cfg.default_task_timeout = float(val)
        elif key == "post_task_pause":
            cfg.post_task_pause = float(val)
        elif key == "startup_delay":
            cfg.startup_delay = float(val)
        elif key == "keyboard_enabled":
            cfg.keyboard_enabled = bool(val)
        else:
            rospy.set_param(key, val)
    rospy.loginfo("[nav_manager] params updated: %s" % json.dumps(data))
