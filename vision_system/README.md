# Vision System (视觉系统)

本目录包含机器人视觉系统的核心代码。该系统采用 **Master-Slave (主从)** 架构，由一个主控节点 (`vision_master.py`) 和多个底层驱动节点 (`yolo_node.py`, `ocr_node.py`, `voice_node.py`) 组成。

## 1. 系统架构 (System Architecture)

视觉系统设计为独立于导航系统的模块，通过 ROS 话题进行异步通信。

*   **Vision Master (`vision_master.py`)**:
    *   **角色**: 系统的“大脑”。
    *   **职责**:
        1.  **进程管理**: 启动并监控底层驱动子进程 (YOLO, OCR, Voice)。每个子进程运行在独立的 Conda 环境中，避免依赖冲突。
        2.  **任务调度**: 接收来自导航节点的指令 (`trigger`), 加载对应的 **逻辑任务控制器 (Logic Task Controller)**。
        3.  **状态中转**: 将底层驱动的识别结果汇总，处理后反馈给导航节点。

*   **Drivers (底层驱动)**:
    *   **YOLO Node (`yolo_node.py`)**: 负责目标检测。支持动态切换模型 (如 `traffic_light`, `doll`, `garbage` 等)。
    *   **OCR Node (`ocr_node.py`)**: 负责文字识别 (PaddleOCR)。
    *   **Voice Node (`voice_node.py`)**: 负责语音合成与播报 (Sherpa-ONNX)。

## 2. 运行逻辑 (Running Logic)

### 2.1 启动流程
1.  运行 `vision_master.py`。
2.  Master 自动检测并激活对应的 Conda 环境 (`yolo`, `ocr`, `speak`)。
3.  Master 启动子进程：
    *   `yolo_node.py` (等待指令)
    *   `ocr_node.py` (等待指令)
    *   `voice_node.py` (准备就绪)
4.  Master 进入空闲状态，监听 `/vision/cmd`。

### 2.2 任务执行流程
当收到导航指令 `trigger <task_name> <seq_id>` 时：
1.  Master 查找对应的 **Logic Task** 类 (例如 `FireTask`)。
2.  调用 Task 的 `start()` 方法。
3.  **Logic Task** 根据业务逻辑，向底层驱动发送指令 (例如 `/vision/driver/yolo/cmd` -> `start fire`)。
4.  **Logic Task** 订阅底层驱动的结果话题，处理数据 (例如过滤置信度、计数、逻辑判断)。
5.  任务完成后，Task 调用回调函数，Master 将结果发布到 `/vision/done`。

## 3. 子任务逻辑详解 (Sub-tasks)

所有逻辑任务代码位于 `tasks/` 目录下。

| 任务名称 | 对应类 | 逻辑描述 | 成功条件 |
| :--- | :--- | :--- | :--- |
| **plate** | `PlateTask` | 1. 启动 OCR 驱动。<br>2. 监听识别到的文本。 | OCR 返回任意非空文本。 |
| **fire** | `FireTask` | **阶段一**: 启动 YOLO (`fire` 模型)。<br>**阶段二**: 检测到火焰后，关闭 YOLO，启动 OCR。<br>**阶段三**: 识别火焰附近的文字信息。 | OCR 识别到文本信息。 |
| **traffic_light** | `TrafficLightTask` | 启动 YOLO (`lights` 模型)。持续发布红绿灯状态到 `/vision/status`。 | (无终止条件，由导航层决定何时停止) |
| **doll** | `DollTask` | 启动 YOLO (`people_v2` 模型)。统计特定类别的数量。 | 识别并计数完成 (具体逻辑由 `DollTask` 内部维护)。 |
| **garbage** | `GarbageController` | 启动 YOLO (`garbage` 模型)。YOLO 节点内部处理分类并生成语音文本。 | 收到 YOLO 返回的 `voice:...` 信号，并等待语音播报完成。 |
| **fallen_vehicle** | `FallenVehicleController` | 启动 YOLO (`fallen_vehicle` 模型)。 | 收到 YOLO 返回的 `done_fallen_vehicle` 信号。 |

## 4. ROS 话题交互 (ROS Topics)

### 4.1 外部接口 (Nav <-> Master)

| 话题 | 方向 | 类型 | 格式/示例 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| `/vision/cmd` | Sub | `std_msgs/String` | `trigger <task> <seq>`<br>`stop <task>` | 导航发送的控制指令。<br>例: `trigger fire 1` |
| `/vision/done` | Pub | `std_msgs/String` | `done_<task> <result>` | 任务完成信号。<br>例: `done_plate A1234` |
| `/vision/status` | Pub | `std_msgs/String` | `status_<task> <info>` | 实时状态流 (用于红绿灯等)。<br>例: `status_traffic_light green` |

### 4.2 内部接口 (Master <-> Drivers)

这些话题通常不需要手动操作，由 Master 自动管理。

| 话题 | 节点 | 说明 |
| :--- | :--- | :--- |
| `/vision/driver/yolo/cmd` | Master -> YOLO | 发送 `start <model_name>` 或 `stop`。 |
| `/vision/driver/yolo/result` | YOLO -> Master | JSON 格式的检测结果或特定状态字符串。 |
| `/vision/driver/ocr/cmd` | Master -> OCR | 发送 `start` 或 `stop`。 |
| `/vision/driver/ocr/result` | OCR -> Master | JSON 格式: `{"texts": ["abc", "123"]}`。 |
| `/vision/driver/voice/cmd` | Master -> Voice | JSON 格式: `{"text": "你好", "action": "say"}`。 |

## 5. 开发与调试

*   **手动触发任务**:
    `vision_master.py` 启动后会在终端显示控制台菜单，输入数字即可模拟发送 `/vision/cmd` 指令，方便单独测试视觉模块。

*   **添加新任务**:
    1. 在 `tasks/` 下创建新的 Python 文件 (继承 `BaseTask` 或参考现有 Task)。
    2. 实现 `start()`, `stop()` 和数据处理逻辑。
    3. 在 `vision_master.py` 的 `self.logic_tasks` 字典中注册新任务。

## 6. 内部拓扑图 (Internal Topology)

下图展示了 Vision Master 内部逻辑任务控制器与底层驱动节点之间的详细交互关系。

```text
                                     +----------------------+
                                     |      NavManager      |
                                     +----------+-----------+
                                                |
                                                | /vision/cmd (trigger/stop)
                                                v
+-----------------------------------------------------------------------------------------+
|                                 VisionMaster (Main Node)                                |
|                                                                                         |
|   +----------------+      +---------------------------------------------------------+   |
|   |                |      |                 Logic Task Controllers                  |   |
|   |   Dispatcher   |----->|  (Instantiated Classes in vision_master.py)             |   |
|   |                |      |                                                         |   |
|   |                |      |  +-----------+   +----------+   +-------------------+   |   |
|   |                |<-----|  | PlateTask |   | FireTask |   | TrafficLightTask  |   |   |
|   +-------+--------+      |  +-----------+   +----------+   +-------------------+   |   |
|           |               |  +----------+    +-------------+  +-------------------+ |   |
|           | /vision/done  |  | DollTask |    | GarbageCtrl |  | FallenVehicleCtrl | |   |
|           v               |  +----------+    +-------------+  +-------------------+ |   |
|                           +--------+---------------+------------------+-------------+   |
|                                    |               |                  |                 |
|                                    | Pub: cmd      | Pub: cmd         | Pub: cmd        |
|                                    | Sub: result   | Sub: result      |                 |
+------------------------------------|---------------|------------------|-----------------+
                                     |               |                  |
        +----------------------------+               |                  |
        |                            +---------------+                  |
        v                            v                                  v
+----------------+          +----------------+                  +----------------+
|   YOLO Node    |          |    OCR Node    |                  |   Voice Node   |
| (Subprocess)   |          |  (Subprocess)  |                  |  (Subprocess)  |
|                |          |                |                  |                |
| Topics:        |          | Topics:        |                  | Topics:        |
| Sub: .../cmd   |          | Sub: .../cmd   |                  | Sub: .../cmd   |
| Pub: .../result|          | Pub: .../result|                  |                |
+----------------+          +----------------+                  +----------------+
```
