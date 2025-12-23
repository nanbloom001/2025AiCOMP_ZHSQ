# 本项目为2025全球校园人工智能算法精英大赛-智慧社区赛道-国赛开源代码
## 架构仅供参考，所依赖的conda环境已由.yml文件列出


# Nav Demo Project (导航演示项目)

本项目是一个集成了 ROS 导航 (`move_base`) 与深度学习视觉系统 (`YOLO`, `PaddleOCR`) 的综合演示项目。机器人按照预设的路径点巡逻，并在特定位置执行视觉识别任务（如红绿灯检测、人偶计数、车牌识别、火焰检测等）。

## 1. 系统架构 (System Architecture)

本系统采用分层架构设计，分为 **导航层 (Navigation Layer)** 和 **视觉层 (Vision Layer)**，通过 ROS 话题进行异步通信。

### 1.1 拓扑结构图

```text
+-----------------------------------------------------------------------------------+
|                                  Hardware Layer                                   |
|  +-------------+      +-------------+      +-------------+      +--------------+  |
|  |   Lidar     |      |   Camera    |      |  Base/Motor |      |  Microphone  |  |
|  +------+------+      +------+------+      +------+------+      +-------+------+  |
|         | /scan              | /usb_cam           | /cmd_vel            |         |
+---------|--------------------|--------------------|---------------------|---------+
          |                    |                    |                     |
+---------v--------------------v--------------------v---------------------v---------+
|                                    ROS Core                                       |
+-----------------------------------------------------------------------------------+
          ^                    ^                    ^                     ^
          | /scan              | /usb_cam/image_raw | /cmd_vel            |
+---------|--------------------|--------------------|---------------------|---------+
|         |                    |                    |                     |         |
|  +------+------+      +------+------+      +------+------+      +-------+------+  |
|  |  move_base  |<-----| NavManager  |<---->| VisionMaster|----->|  VoiceNode   |  |
|  | (Navigation)|      | (Scheduler) |      | (Controller)|      | (TTS Engine) |  |
|  +-------------+      +------+------+      +------+------+      +--------------+  |
|                              ^                    |                               |
|                              | /vision/cmd        | /vision/driver/yolo/cmd       |
|                              | (trigger/stop)     | (start/stop)                  |
|                              |                    v                               |
|                              |             +------+------+                        |
|                              |             |  YOLO Node  |                        |
|                              |             |  (Detect)   |                        |
|                              |             +------+------+                        |
|                              |                    |                               |
|                              | /vision/done       | /vision/driver/yolo/result    |
|                              | (result)           | (json)                        |
|                              |                    v                               |
|                              |             +------+------+                        |
|                              +-------------|  OCR Node   |                        |
|                                            | (Recognize) |                        |
|                                            +-------------+                        |
+-----------------------------------------------------------------------------------+
```

### 1.2 核心节点交互

1.  **NavManager (`nav_manager_node.py`)**:
    *   **角色**: 任务调度器。
    *   **输入**: 目标点列表 (`goals.py`)、视觉结果 (`/vision/done`)、视觉状态 (`/vision/status`)。
    *   **输出**: 导航目标 (`move_base/goal`)、视觉指令 (`/vision/cmd`)。
    *   **逻辑**: 维护一个状态机，控制“导航 -> 到达 -> 触发视觉 -> 等待结果 -> 下一目标”的循环。

2.  **VisionMaster (`vision_master.py`)**:
    *   **角色**: 视觉中控。
    *   **输入**: 导航指令 (`/vision/cmd`)、底层驱动结果 (`/vision/driver/*/result`)。
    *   **输出**: 任务结果 (`/vision/done`)、底层驱动指令 (`/vision/driver/*/cmd`)。
    *   **逻辑**: 接收 `trigger` 指令后，实例化对应的 **Logic Task** (如 `FireTask`)，该 Task 负责协调 YOLO 和 OCR 的启停与数据处理。

3.  **底层驱动 (Drivers)**:
    *   **YOLO Node**: 运行 YOLOv5/v8/v11 模型，支持动态切换模型权重 (如 `traffic_light.pt`, `fire.pt`)。
    *   **OCR Node**: 运行 PaddleOCR，识别图像中的文字。
    *   **Voice Node**: 运行 Sherpa-ONNX，提供离线语音合成播报。

## 2. 导航与任务流程详解 (Workflow)

系统启动后，`NavManager` 会依次加载 `goals.py` 中的目标点，并执行以下流程。

### 阶段一：出发与交通灯 (Start & Traffic Light)
*   **Goal 1**: 到达路口起始点。
    *   **任务**: `traffic_light` (Seq 1)
    *   **代码逻辑**:
        1.  `NavManager` 发送 `trigger traffic_light 1`。
        2.  `VisionMaster` 启动 `TrafficLightTask`，进而发送 `start traffic_light` 给 YOLO 节点。
        3.  YOLO 节点加载 `lights` 模型，识别红/绿/黄灯。
        4.  `TrafficLightTask` 解析 YOLO 结果，通过 `/vision/status` 持续发布 `status_traffic_light red_light` 或 `green_light`。
        5.  `NavManager` 的 `TaskHandler` 监听到 `green_light` 后，结束等待，继续导航。

### 阶段二：A区巡逻 (Block A Patrol)
*   **Goal 3, 8, 13**: 人偶观测点 (Seq 1, 2, 3)。
    *   **任务**: `doll`
    *   **代码逻辑**:
        1.  `NavManager` 发送 `trigger doll <seq>`。
        2.  `VisionMaster` 启动 `DollTask`，YOLO 加载 `people_v2` 模型。
        3.  `DollTask` 连续采集 5 帧图像，统计每帧识别到的职业类别 (如 `doctor`, `teacher`)。
        4.  **判定**: 如果某个类别在 5 帧中出现至少 3 次，则确认为有效识别。
        5.  结果根据 Seq ID 分类存入 A 区或 B 区列表。
        6.  任务自动结束并返回 `done_doll`。

### 阶段三：B区巡逻与火焰检测 (Block B & Fire)
*   **Goal 14, 19**: B 区人偶观测点 (Seq 4, 5)。
    *   **逻辑**: 同上，结果计入 B 区。
*   **Goal 16, 20, 34**: 建筑火焰检测 (Seq 1, 2, 3)。
    *   **任务**: `fire`
    *   **代码逻辑**:
        1.  **Step 1 (YOLO)**: 启动 YOLO (`fire` 模型) 检测火焰。
        2.  **Step 2 (Switch)**: 如果检测到火焰 (box count > 0)，`FireTask` 立即停止 YOLO，并启动 OCR。
        3.  **Step 3 (OCR)**: OCR 节点识别火焰附近的文字信息 (如楼层号)。
        4.  **Step 4 (Report)**: 识别到文字后，通过 `VoiceNode` 播报“发现火情”，并返回结果 `done_fire <text>`。

### 阶段四：车牌与垃圾分类 (Plate & Garbage)
*   **Goal 23, 26, 32**: 车位检测。
    *   **任务**: `plate`
    *   **代码逻辑**:
        1.  直接启动 OCR 节点。
        2.  `PlateTask` 监听 OCR 结果，一旦识别到非空文本字符串，即视为车牌号。
        3.  返回 `done_plate <text>`。
*   **Goal 29**: 垃圾桶检测。
    *   **任务**: `garbage`
    *   **代码逻辑**:
        1.  启动 YOLO (`garbage` 模型)。
        2.  YOLO 节点内部集成了分类逻辑，识别到垃圾后，会直接返回带有 `voice:` 前缀的指令。
        3.  `GarbageController` 收到 `voice:` 指令后，触发语音播报，并延迟 3 秒等待播报结束。
        4.  返回任务完成信号。

## 3. 快速开始 (Quick Start)

### 3.1 启动硬件与底层驱动
首先启动底盘、雷达、摄像头等硬件驱动：
```bash
roslaunch src/NAV.launch
```
*(注: 该脚本会包含 `nav777.launch` 并延迟启动 `newt.py`)*

### 3.2 启动业务系统
建议在两个新的终端窗口中分别运行：

**终端 1: 视觉主控**
```bash
# 启动后会自动拉起 YOLO, OCR, Voice 子进程
rosrun nav_demo vision_master.py
```

**终端 2: 导航管理器**
```bash
# 启动后开始执行 goals.py 中的任务序列
rosrun nav_demo nav_manager_node.py
```

### 3.3 运行时控制
在 `nav_manager_node.py` 的终端窗口中，支持键盘交互：
*   `p`: **暂停 (Pause)** - 机器人停止移动，保持当前状态。
*   `r`: **恢复 (Resume)** - 继续执行当前目标。
*   `s`: **跳过 (Skip)** - 放弃当前目标或正在执行的视觉任务，直接前往下一个目标。


