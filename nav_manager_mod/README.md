# Navigation Manager Module (导航管理模块)

`nav_manager_mod` 是本项目的核心调度模块，实现了基于状态机的自主导航与任务协同功能。它负责按照预定序列指挥机器人移动 (`move_base`)，并在到达指定位置后触发并等待视觉任务 (`vision_system`) 完成。

## 1. 模块概述 (Overview)

本模块设计目标是实现**导航**与**业务逻辑**的解耦。
*   **导航层**: 封装了 ROS `move_base` Action Client，提供简单的“前往某点”接口，并处理超时和失败重试。
*   **业务层**: 通过 `goals.py` 定义任务序列，通过 ROS 话题与视觉主控 (`vision_master`) 握手。
*   **交互层**: 提供键盘控制和状态上报，方便调试和比赛现场干预。

## 2. 核心架构 (Core Architecture)

### 2.1 状态机流程
系统维护一个主循环，在以下状态间流转：

1.  **IDLE (空闲/准备)**: 等待初始化完成或暂停状态。
2.  **NAVIGATING (导航中)**: 发送目标点给 `move_base`，阻塞等待结果。
    *   *异常处理*: 若导航超时或失败，记录日志并根据配置决定是否重试或跳过。
3.  **TASK_TRIGGER (任务触发)**: 到达目标点，向 `/vision/cmd` 发送指令。
4.  **WAITING_TASK (等待任务)**: 阻塞等待 `/vision/done` 反馈。
    *   *特殊逻辑*: 对于 `traffic_light`，会持续监听 `/vision/status` 直到绿灯。
5.  **NEXT_GOAL (切换目标)**: 索引自增，加载下一个目标。

### 2.2 模块依赖
*   **Upstream**: `move_base` (ROS Navigation Stack)
*   **Downstream**: `vision_master` (本项目视觉系统)

## 3. 文件详解与开发指南 (Development Guide)

### 3.1 配置管理 (`config.py`)
定义了系统运行所需的关键数据结构。若需修改默认超时时间或容差，请修改 `NavConfig` 类。

*   **`NavConfig`**: 全局配置参数。
    *   `default_nav_timeout`: 导航最大时长 (默认 60s)。
    *   `default_task_timeout`: 视觉任务最大等待时长 (默认 30s)。
*   **`Goal`**: 目标点定义 `(x, y, yaw, tasks=[...])`。
*   **`Task`**: 任务定义 `(type, seq_id)`。

### 3.2 目标点序列 (`goals.py`)
**这是比赛或测试时最常修改的文件。**
`GOALS` 列表定义了机器人的巡逻路径。

**如何添加新目标点**:
```python
# 示例: 在坐标 (1.0, 2.0) 处停下，执行 "fire" 检测任务
Goal(1.0, 2.0, 1.57, [Task("fire", 1)])
```

### 3.3 导航执行器 (`nav_executor.py`)
封装了 `SimpleActionClient`。
*   **关键方法**: `navigate(goal, cfg, state)`
*   **修改建议**: 如果需要更换导航栈 (如 `move_base_flex`) 或修改底盘控制逻辑，请修改此类。

### 3.4 任务处理器 (`task_handler.py`)
负责与视觉系统的握手协议。
*   **关键方法**: `wait_for_completion(...)`
*   **扩展**: 如果引入了新的交互模式 (例如需要双向确认)，请在此扩展逻辑。

## 4. ROS 接口定义 (ROS Interface)

### 4.1 订阅 (Subscribed)
| 话题 | 类型 | 作用 |
| :--- | :--- | :--- |
| `/vision/done` | `std_msgs/String` | **任务完成信号**。<br>格式: `done_<task_type> <result_data>`<br>例: `done_plate A88888` |
| `/vision/status` | `std_msgs/String` | **实时状态流** (非阻塞)。<br>用于红绿灯等需要持续监测的任务。<br>格式: `status_<task> <val>` |
| `/nav_ctrl/param_update` | `std_msgs/String` | **动态调参**。<br>接收 JSON 字符串更新 `NavConfig`。<br>例: `{"default_nav_timeout": 100.0}` |

### 4.2 发布 (Published)
| 话题 | 类型 | 作用 |
| :--- | :--- | :--- |
| `/vision/cmd` | `std_msgs/String` | **视觉指令**。<br>格式: `trigger <task> <seq>` 或 `stop <task>` |
| `/nav_ctrl/status` | `std_msgs/String` | **系统状态监控**。<br>以 JSON 格式发布当前目标索引、状态、剩余时间等。 |

## 5. 操作手册 (Operation Manual)

### 5.1 启动
```bash
# 确保已启动 roscore, move_base 和 vision_master
rosrun nav_demo nav_manager_node.py
```

### 5.2 键盘控制 (Runtime Control)
程序启动后，终端会监听按键输入（无需回车）：

*   **`p` (Pause)**: **暂停**。
    *   导航中: 立即取消当前 `move_base` 目标，机器人急停。
    *   任务中: 忽略视觉结果，保持原地等待。
*   **`r` (Resume)**: **恢复**。
    *   从暂停处继续执行当前逻辑（重新发送导航目标或继续等待视觉）。
*   **`s` (Skip)**: **跳过**。
    *   **强制结束**当前阶段（无论是导航还是视觉任务），直接索引+1，跳转到下一个目标点。
    *   *场景*: 机器人卡死、视觉识别超时但不想等待时使用。
*   **`q` (Quit)**: **退出**节点。


