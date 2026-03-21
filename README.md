# 2025 全球校园人工智能算法精英大赛 智慧社区赛道开源代码

本仓库是智慧社区机器人项目的比赛代码与工程资料，ROS 包名为 `nav_demo`。当前实现以自主导航为主线，叠加视觉识别、语音播报、地图定位、采点与调试工具，适合作为比赛复现、工程接手和功能拆解的参考。

根文档以当前仓库中的真实实现为准，优先说明主线入口、实际依赖、现有协议和调试方式。子目录中的 README 仍可作为补充，但部分描述可能落后于当前代码。

## 项目概览

当前仓库可以概括为 5 条主线：

- 导航编排：`nav_manager_node.py` 与 `nav_manager_mod/` 负责状态机、目标点序列、任务触发和超时控制。
- 视觉主控：`vision_system/vision_master.py` 负责接收导航指令，并拉起 YOLO、OCR、Voice 子进程。
- 视觉驱动：`vision_system/yolo_node.py`、`vision_system/ocr_node.py`、`vision_system/voice_node.py` 分别负责检测、识别和播报。
- 地图与导航参数：`launch/`、`map/`、`param/`、`config/` 提供地图、AMCL、`move_base`、RViz 配置。
- 调试与辅助脚本：根目录和 `vision_system/` 中保留了采点、遥控、相机测试、工作流模拟和历史版本脚本。

项目当前的主线实现不是“导航到了点位后一直同步等待所有视觉识别结束”，而是混合了两种任务风格：

- 阻塞型任务：导航会等待任务真正结束，例如 `traffic_light`、`fallen_vehicle`。
- 抓拍即返回型任务：导航侧收到“已抓拍”后立即继续，识别与播报在后台完成，例如 `doll`、`fire`、`plate`、`garbage`、`sign`。

## 系统架构

整体上可以理解为“导航层驱动流程，视觉层提供能力，ROS 话题做解耦”：

```text
硬件/底盘/雷达/相机
        |
        v
  roslaunch nav_demo nav777.launch
        |
        +-- map_server
        +-- AMCL
        +-- move_base
        +-- 雷达/里程计/IMU/EKF/RViz
        |
        v
  nav_manager_node.py
        |
        |  /vision/cmd
        v
  vision_system/vision_master.py
        |
        +-- yolo_node.py   (conda env: yolo)
        +-- ocr_node.py    (conda env: ocr)
        +-- voice_node.py  (conda env: speak)
        |
        +-- /vision/done
        +-- /vision/status
```

当前主线的几个关键入口文件如下：

- `launch/nav777.launch`：完整导航与硬件链路的集成入口。
- `vision_system/vision_master.py`：视觉主控入口，会自动拉起 YOLO、OCR、Voice 子进程。
- `nav_manager_node.py`：导航调度入口，负责“导航 -> 任务 -> 下一点”的总流程。
- `nav_manager_mod/goals.py`：当前比赛路线和任务序列定义。

## 仓库导览

### 导航主线

| 路径 | 作用 |
| --- | --- |
| `nav_manager_node.py` | 当前主线导航调度器，负责状态机、键盘控制、日志记录、任务握手。 |
| `nav_manager_mod/config.py` | `Goal`、`Task`、`NavConfig`、`NavState` 等核心数据结构。 |
| `nav_manager_mod/goals.py` | 当前启用的导航点与任务序列。 |
| `nav_manager_mod/nav_executor.py` | `move_base` Action Client 封装。 |
| `nav_manager_mod/task_handler.py` | 负责 `/vision/cmd` 触发和 `/vision/done`、`/vision/status` 等待逻辑。 |

### 视觉主线

| 路径 | 作用 |
| --- | --- |
| `vision_system/vision_master.py` | 视觉总控，统一接收导航触发、管理子进程、分发逻辑任务。 |
| `vision_system/yolo_node.py` | YOLO 驱动，支持多模型切换、快照模式、结果图保存。 |
| `vision_system/ocr_node.py` | OCR 驱动，支持持续模式和快照模式。 |
| `vision_system/voice_node.py` | 语音指令接收与播报分发。 |
| `vision_system/tasks/` | 逻辑任务控制器和部分复杂视觉任务实现。 |
| `vision_system/core/` | 模型加载、图像转换、结果管理等底层工具。 |
| `vision_system/models/` | 当前仓库内置的 ONNX/PTH 模型文件。 |

### Launch、参数、地图与可视化

| 路径 | 作用 |
| --- | --- |
| `launch/` | 地图、AMCL、`move_base`、整车启动和测试用 launch 文件。 |
| `param/` | `move_base`、DWA、TEB、costmap、global planner 等参数文件。 |
| `map/` | 当前使用地图 `mymap_zzz.yaml` 与 `mymap_zzz.pgm`。 |
| `config/` | RViz 配置和个别 IMU 参数。 |

### 采点、遥控、测试与历史脚本

| 路径 | 作用 |
| --- | --- |
| `teleop_collect.py` | 遥控 + AMCL 采点工具。 |
| `auto_collect_point.py` | 仅采点工具。 |
| `my_teleop.py` | 简单键盘遥控。 |
| `vision_system/camera_test.py` | 摄像头图像显示与检查。 |
| `vision_system/test_env.py` | 环境快速验证脚本。 |
| `vision_system/test_workflow_simulation.py` | 视觉工作流模拟测试。 |
| `vision_system/test_garbage_task.py` | 垃圾任务链路测试。 |
| `vision_system/test_stiching.py` | 拼接测试脚本。 |
| `vision_system/voice_node_test.py` | 语音节点测试脚本。 |
| `navigation_OT.py`、`navigation_only.py`、`nav_manager.py` | 历史导航版本或实验版本，不是当前主线。 |
| `fake_vision_stub.py` | 旧版假视觉桩节点，协议已不同于当前主线。 |
| `RGY_opencv.py` | 基于传统视觉的红绿灯检测实验代码。 |
| `amcl_pose.py` | AMCL 位姿监视工具。 |
| `tts.py` | 独立 TTS 实验脚本。 |

### 日志与资源

- `logs/`：导航过程日志样例，当前主线运行时也会写导航日志。
- `vision_system/task_save/`：视觉节点在运行时可能保存结果图，目录由程序按需创建。

## 当前主运行链路

当前仓库的完整主链路建议按下面理解：

1. `roslaunch nav_demo nav777.launch`
   - 加载地图 `map/mymap_zzz.yaml`
   - 启动激光雷达、机器人模型、IMU、轮式里程计、EKF、AMCL、`move_base` 和 RViz
2. `python3 vision_system/vision_master.py`
   - 自动检测并激活 `yolo`、`ocr`、`speak` 三个 conda 环境
   - 拉起 `yolo_node.py`、`ocr_node.py`、`voice_node.py`
   - 监听 `/vision/cmd`
3. `python3 nav_manager_node.py`
   - 加载 `nav_manager_mod/goals.py`
   - 依次调用 `move_base`
   - 到点后发送 `trigger <task> <seq>`
   - 根据任务类型等待 `/vision/done` 或 `/vision/status`

当前启用的 `goals.py` 共包含 33 个导航点，其中：

- `traffic_light` 2 次
- `doll` 5 次
- `sign` 1 次
- `fire` 3 次
- `plate` 3 次
- `garbage` 1 次
- `fallen_vehicle` 1 次
- `none` 17 次，用于转向、恢复姿态和路线过渡

从路线阶段上看，可以大致理解为：

1. 起点红绿灯放行
2. A/B 社区人偶识别与指示牌播报
3. 楼宇火情巡检与二次红绿灯
4. 车牌识别、垃圾分类、第三处火情、电动车倒伏检测
5. 收尾返回

## 任务与比赛流程

下面的说明以当前代码行为为准，而不是早期同步阻塞式文档。

| 任务 | 当前实现 | 导航侧是否阻塞 | 备注 |
| --- | --- | --- | --- |
| `traffic_light` | `TrafficLightTask` 持续启动 YOLO `traffic_light` 模型，向 `/vision/status` 发布红黄绿灯状态。 | 是 | `nav_manager` 主要依赖 `status_traffic_light` 判断是否连续检测到绿灯，并在超时或跳过时发送 `stop traffic_light`。 |
| `doll` | 到点后先抓拍一帧并立即回 `done_doll snapshot_taken`，后台再执行 YOLO 快照推理、累计 A/B 区统计，并在最后一次触发后语音汇总。 | 否 | 当前使用模型名为 `people_v3_s`。 |
| `sign` | 立即回 `done_sign success`，后台延时约 1 秒发语音任务。 | 否 | 当前逻辑更像“提示音/停字牌播报”，不是视觉识别。 |
| `fire` | 到点抓拍后立即回 `done_fire snapshot_taken`，后台执行 YOLO 火焰检测，必要时再触发 OCR 并播报。 | 否 | 当前导航不会等待 OCR 结束。 |
| `plate` | 到点抓拍后立即回 `done_plate snapshot_taken`，后台做 OCR 车牌识别并语音播报。 | 否 | 车牌文本会按正则规则过滤。 |
| `garbage` | 到点抓拍后立即回 `done_garbage snapshot_taken`，后台触发垃圾桶/垃圾识别与分类播报。 | 否 | 复杂分类逻辑主要在 YOLO 侧 `GarbageTask`。 |
| `fallen_vehicle` | 启动倒地电动车任务，YOLO 侧会旋转机器人、采图、拼接、检测并触发语音。 | 是 | `nav_manager` 会等待 `done_fallen_vehicle ...`，默认给更长超时。 |

### 任务真实语义的重点说明

- `doll`、`fire`、`plate`、`garbage`、`sign` 都属于“前台抓拍立即返回，后台继续识别/播报”的实现。
- `traffic_light` 和 `fallen_vehicle` 才是当前主线里真正会显著阻塞导航推进的任务。
- `traffic_light` 在视觉侧既可能发布 `done_traffic_light green_light`，也会持续发布 `status_traffic_light ...`；当前导航侧的主要控制依据是状态流。

## ROS 接口与任务协议

### 外部接口：导航层与视觉主控

| 话题 | 方向 | 类型 | 当前格式 |
| --- | --- | --- | --- |
| `/vision/cmd` | Nav -> Vision Master | `std_msgs/String` | `trigger <task> <seq>` 或 `stop <task>` |
| `/vision/done` | Vision Master -> Nav | `std_msgs/String` | `done_<task> <result>` |
| `/vision/status` | Vision Master -> Nav | `std_msgs/String` | `status_<task> <info>` |
| `/nav_ctrl/status` | Nav -> 外部监控 | `std_msgs/String` | JSON，包含当前点位、暂停状态、超时参数等 |
| `/nav_ctrl/param_update` | 外部 -> Nav | `std_msgs/String` | JSON，用于运行时更新 `NavConfig` |

常见消息示例：

```text
trigger fire 1
trigger doll 5
stop traffic_light
done_plate snapshot_taken
done_fallen_vehicle detected count=1 | timing=...
status_traffic_light green_light | timing=... | duration_ms=...
```

### 内部接口：Vision Master 与 Drivers

| 话题 | 方向 | 用途 |
| --- | --- | --- |
| `/vision/driver/yolo/cmd` | Master -> YOLO | 发送 `start <model>`、`snapshot <model>`、`stop` |
| `/vision/driver/yolo/result` | YOLO -> Master | JSON 结果或特定字符串结果 |
| `/vision/driver/ocr/cmd` | Master -> OCR | 发送 `start`、`snapshot`、`stop` |
| `/vision/driver/ocr/result` | OCR -> Master | JSON 文字识别结果 |
| `/vision/driver/voice/cmd` | Master/Task -> Voice | JSON 语音任务 |
| `/vision/driver/voice/status` | Voice -> Master/Task | JSON 回执状态 |
| `/vision/snapshot` | Master/Task -> Drivers | 快照图像传递 |

### 当前任务名

当前主线文档化的任务名如下：

- `traffic_light`
- `doll`
- `sign`
- `fire`
- `plate`
- `garbage`
- `fallen_vehicle`

这些任务名已经在 `goals.py`、`vision_master.py`、`task_handler.py` 的当前实现中使用，不建议只改 README 而不改代码。

## 环境与依赖

### ROS 与工作空间

本仓库是一个 catkin 包，包名为 `nav_demo`。运行完整链路前，默认需要：

- 可用的 ROS 运行环境与 catkin 工作空间
- 已 `source` 过 ROS 与工作空间的 `setup.bash`
- `move_base`、`amcl`、`map_server` 等导航栈组件

当前 `package.xml` 只声明了部分导航依赖，尚未完整声明所有运行时依赖，也没有把 Python 脚本完整安装进 catkin 运行路径，因此本文档更推荐直接用 `python3` 启动主脚本。

### Conda 环境

视觉链路依赖 3 个 conda 环境，仓库中已提供环境文件：

- `yolo.yml`：YOLO/ONNX/OpenCV 相关环境，当前为 Python 3.10
- `ocr.yml`：OCR/PaddleOCR 相关环境，当前为 Python 3.10
- `speak.yml`：语音环境，当前为 Python 3.8

建议在首次部署时创建环境：

```bash
conda env create -f yolo.yml
conda env create -f ocr.yml
conda env create -f speak.yml
```

`vision_master.py` 会尝试通过 `conda activate yolo|ocr|speak` 拉起子进程，所以环境名需要与 YAML 文件中的环境名保持一致。

### 默认图像、地图与模型

- 默认相机话题：`/camera/color/image_raw`
- 地图文件：`map/mymap_zzz.yaml`、`map/mymap_zzz.pgm`
- 视觉模型目录：`vision_system/models/`

当前 `vision_system/config.py` 中实际启用或配置过的模型包括：

- `lights`
- `fire`
- `people_v3_s`
- `best_bin`
- `best_trash`
- `best_classify`
- `eb`

### `launch/` 中涉及的外部 ROS 包

若使用当前主线 `nav777.launch`，还需要系统中存在相应 ROS 包或硬件驱动：

- `lslidar_driver`
- `mowen`
- `wit_ros_imu`
- `car_bringup`
- `sync_example`
- `robot_localization`

不同测试 launch 还可能依赖：

- `laser_scan_matcher`
- `depthimage_to_laserscan`
- `tf`

如果这些外部包、对应硬件驱动或消息源不存在，完整链路无法直接联调。

## 启动方式

### 1. 准备工作

建议先完成以下步骤：

1. 将仓库放入 catkin 工作空间的 `src/` 下，或保证 ROS 能通过包名 `nav_demo` 找到它。
2. `catkin_make` 或使用等价方式构建工作空间。
3. `source` ROS 与工作空间环境。
4. 创建并验证 `yolo`、`ocr`、`speak` 三个 conda 环境。
5. 确认相机、雷达、底盘、IMU 和外部 ROS 包可用。

### 2. 启动完整硬件链路

当前仓库推荐的完整启动入口是：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch nav_demo nav777.launch
```

`nav777.launch` 会启动地图服务、雷达、机器人模型、IMU、轮式里程计、EKF、AMCL、`move_base` 和 RViz，是当前最接近完整比赛链路的 launch。

补充说明：

- `nav_a.launch`、`nav_c.launch`、`nav_mm.launch`、`navtest.launch` 是不同定位/里程计组合的变体。
- `nav02_map_save.launch`、`nav03_map_server.launch`、`nav04_*.launch`、`nav05_path_dwa.launch` 是模块化拆开的 launch。
- 虽然仓库里保留了 `teb_local_planner_params.yaml`，但当前 `move_base_params.yaml` 实际激活的是 `dwa_local_planner/DWAPlannerROS`。

### 3. 启动视觉主控

从仓库根目录执行：

```bash
source ~/catkin_ws/devel/setup.bash
python3 vision_system/vision_master.py
```

之所以推荐 `python3` 而不是直接 `rosrun`，原因是：

- `vision_system/vision_master.py` 当前没有执行权限
- 当前 `CMakeLists.txt` 没有使用 `catkin_install_python`
- 直接执行脚本更符合当前仓库形态

视觉主控启动后会：

- 自动寻找 `conda.sh`
- 激活 `yolo`、`ocr`、`speak`
- 拉起 `yolo_node.py`、`ocr_node.py`、`voice_node.py`
- 打开一个控制台菜单，允许手动触发视觉任务

### 4. 启动导航调度器

同样从仓库根目录执行：

```bash
source ~/catkin_ws/devel/setup.bash
python3 nav_manager_node.py
```

`nav_manager_node.py` 当前有执行权限，因此在工作空间配置合适时也可能可以使用：

```bash
rosrun nav_demo nav_manager_node.py
```

但为了减少 catkin 安装路径和脚本权限差异带来的问题，本文档仍优先推荐 `python3` 启动。

### 5. 运行时控制

`nav_manager_node.py` 的终端支持以下按键：

- `p`：暂停
- `r`：恢复
- `s`：跳过当前目标或任务
- `q`：退出

视觉主控和驱动节点也各自带有控制台菜单，适合单独测试 YOLO、OCR 或任务控制逻辑。

## 调试与辅助脚本

### 主线调试入口

- `nav_manager_node.py`
  - 用于观察主状态机、导航日志、任务握手、超时和键盘控制
- `vision_system/vision_master.py`
  - 用于观察任务注册、子进程拉起情况和手动触发逻辑任务
- `vision_system/yolo_node.py`
  - 自带控制台，可手动切换模型、冻结画面、切换自动模式
- `vision_system/ocr_node.py`
  - 自带控制台，可强制启停 OCR 或进入快照模式

### 采点与遥控

- `teleop_collect.py`
  - 适合边遥控机器人边把当前 AMCL 位姿写入 `nav_point.json`
- `auto_collect_point.py`
  - 适合只做采点，不负责遥控
- `my_teleop.py`
  - 简单的键盘速度控制
- `amcl_pose.py`
  - 终端实时打印 AMCL 位姿

### 视觉与流程测试

- `vision_system/camera_test.py`
  - 检查相机话题是否正常
- `vision_system/test_env.py`
  - 快速验证视觉相关 Python 环境
- `vision_system/test_workflow_simulation.py`
  - 模拟视觉任务工作流
- `vision_system/test_garbage_task.py`
  - 单测垃圾分类任务链路
- `vision_system/test_stiching.py`
  - 验证拼接能力
- `vision_system/voice_node_test.py`
  - 验证语音节点

### 历史版本与非主线脚本

以下脚本可供参考，但不是当前主线：

- `nav_manager.py`
- `navigation_OT.py`
- `navigation_only.py`
- `fake_vision_stub.py`
- `RGY_opencv.py`
- `tts.py`

如果只是想理解项目演进，可以阅读这些文件；如果要跑当前比赛主链路，请优先看：

- `nav_manager_node.py`
- `nav_manager_mod/`
- `vision_system/vision_master.py`
- `vision_system/tasks/`
- `launch/nav777.launch`

## 已知问题与缺失项

以下问题在当前仓库中是真实存在的，接手时建议优先留意：

1. 语音节点引用的 `voice_models/` 目录当前不在仓库中。
   `vision_system/voice_node.py` 会尝试导入 `voice_models.voice_wav_only`、`voice_models.voice_wav_tts` 和 `voice_models.audio_manager`，但这些文件未被提交到当前仓库。

2. `package.xml` 与 `CMakeLists.txt` 没有完整声明或安装 Python 运行依赖。
   当前文档推荐直接使用 `python3` 启动主脚本，正是为了绕开这部分工程化缺口。

3. 完整联调依赖真实硬件和多个仓库外 ROS 包。
   没有雷达、相机、底盘、IMU，或缺少外部包时，无法直接复现完整比赛链路。

4. 部分逻辑带有现场化假设或硬编码路径。
   例如 `fallen_vehicle` 任务会尝试读取 `/home/mowen/Desktop/invild_c.txt` 作为统计补充数据。

5. 文档与子目录说明存在版本差异。
   根 README 以当前主线代码为准，`vision_system/README.md` 和 `nav_manager_mod/README.md` 更适合作为补充阅读。

## 建议的阅读顺序

如果你是第一次接手这个仓库，建议按下面顺序阅读：

1. `README.md`
2. `launch/nav777.launch`
3. `nav_manager_node.py`
4. `nav_manager_mod/goals.py`
5. `vision_system/vision_master.py`
6. `vision_system/tasks/`
7. `vision_system/config.py`

这样可以先理解“系统怎么跑”，再理解“任务怎么做”，最后再深入具体模型和参数。
