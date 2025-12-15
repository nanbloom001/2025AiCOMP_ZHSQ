# nav_manager_mod 模块说明

简要解释各文件的职责和协作关系，便于维护和修改。

- `nav_manager_node.py`（位于 `src/nav_demo/`）：主入口与调度器。初始化节点、加载配置与目标、订阅视觉/参数更新、启动键盘监听，主循环依次调用导航执行与任务处理，并发布状态。
- `config.py`：定义数据结构与参数更新。包含 `Goal`、`NavConfig`、`NavState` 三个 dataclass；`load_config_from_ros` 从 rosparam 读取默认值；`apply_param_update` 解析 `/nav_ctrl/param_update` 的 JSON 并更新配置。
- `goals.py`：维护当前启用的目标点列表 `GOALS`，并提供 `load_goals()` 返回可修改的副本。后续可改为从 YAML/rosparam 载入。
- `nav_executor.py`：封装与 `move_base` 的交互。`NavExecutor.navigate` 负责发送目标、处理暂停/跳过/超时、读取 action 状态码并返回是否成功。
- `task_handler.py`：视觉任务触发与等待。`trigger` 将任务类型映射到 `/navigation_task` 的触发字符串；`wait_for_completion` 按任务类型等待视觉结果或超时，包含 `traffic_light` 的特殊等待逻辑。
- `__init__.py`：标记为可导入的 Python 包。

运行方式：使用 `nav_manager_node.py` 作为节点入口（例如在 launch 中运行或直接 `rosrun nav_demo nav_manager_node.py`）。
注意：`nav_manager_node.py` 位于 `nav_manager_mod` 目录的上级目录中。