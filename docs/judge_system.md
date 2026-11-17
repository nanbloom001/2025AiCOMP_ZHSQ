# 裁判系统对接与两项新增任务

本文件对应 2026 年第八届总决赛的两处变化：

1. 现场取消语音播报，各队结果**直接提交给裁判系统**；
2. 新增站房仪表读取（14 分）与楼宇异常温度（10 分）两项任务。

## 1. 总体改动

| 模块 | 变化 |
| --- | --- |
| `vision_system/core/judge_client.py` | 新增。结果上报客户端，支持 UDP / HTTP / ROS / 本地文件四种通道 |
| `vision_system/tasks/meter.py` | 新增。站房仪表读取，抓拍即返回 |
| `vision_system/tasks/temperature.py` | 新增。楼宇异常温度楼层识别，抓拍即返回 |
| `vision_system/config.py` | 增加 `JUDGE_CONFIG`、`METER_TABLES`、`FLOOR_REGIONS` 等配置 |
| `vision_system/vision_master.py` | 在 `logic_tasks` 注册 `meter` 与 `temperature` |
| `nav_manager_mod/goals.py` | 预留站房与 D 楼路点（坐标待现场标定后取消注释） |

原有任务（doll、plate、fire、garbage、traffic_light、sign、fallen_vehicle）逻辑未改，
仍按 `/vision/cmd`、`/vision/done`、`/vision/status` 三条消息工作。

## 2. 上报报文

每条结果一条 JSON，`msg_id` 由（轮次、任务、序号、结果摘要）决定：

```json
{
  "schema": "aic.judge.v1",
  "msg_id": "AIC2026_20261117_093000-meter-1-3f2a9c11d0ab",
  "run_id": "AIC2026_20261117_093000",
  "team": "三人行必有我师焉",
  "node": "vision_master",
  "task": "meter",
  "seq": 1,
  "checkpoint": null,
  "ts": 1763345400.512,
  "text": "1 号表读数为 0.42 MPa",
  "payload": {
    "meter_id": 1, "kind": "analog", "value": 0.42, "unit": "MPa",
    "confidence": 0.75, "angle_deg": -32.4, "status": "OK"
  },
  "evidence": {"captured_at": 1763345392.31, "ocr_texts": ["0.42 MPa"]}
}
```

结果不变时 `msg_id` 不变，因此重发不会在裁判端产生第二条有效记录。

## 3. 通道配置

`config.py` 中的 `JUDGE_CONFIG`：

```python
JUDGE_CONFIG = {
    "enabled": True,
    "transport": ["udp", "file"],          # 可多选
    "udp": {"host": "255.255.255.255", "port": 9900},
    "http": {"url": "http://127.0.0.1:8080/api/report", "timeout": 0.8},
    "ros": {"topic": "/judge/report"},
    "file": {"path": "~/.aic_judge_outbox/reports.jsonl"},
}
```

现场不必改代码，用环境变量覆盖即可：

```bash
export AIC_JUDGE_HOST=192.168.1.50      # 裁判机地址
export AIC_JUDGE_PORT=9900
# 或改用 HTTP
export AIC_JUDGE_URL=http://192.168.1.50:8080/api/report
```

`file` 通道始终追加，路径为 `~/.aic_judge_outbox/reports.jsonl`，
断网时裁判端可据此补录；被丢弃或重试失败的记录带 `_tag` 字段（DROPPED / FAILED）。

## 4. 站房仪表

启动后由路点触发 `trigger meter <seq>`。处理流程：

1. 取一帧即回 `done_meter snapshot_taken`，导航继续；
2. 霍夫圆定位表盘，裁出 ROI；
3. **数字表**：ROI 送 OCR，按 `METER_TABLES[seq].unit` 校验并取数；
   **指针表**：沿半径采样 360 个方向，取平均亮度最低者为指针方向，
   再按 `theta0_deg / theta_max_deg / r0 / r_max` 线性映射为读数；
4. 结果上报裁判系统。

现场标定步骤：

1. 让表针指向量程起点，记录此时的指针方向角 `theta0_deg`；
2. 指向量程满度，记录 `theta_max_deg`；
3. 按表盘铭牌填 `r0`、`r_max` 与 `unit`；
4. 用已知读数复核，误差超过 `tolerance` 时微调两个角度。

约定：0 度指向图像 +x，逆时针为正，与 `_needle_angle` 的采样一致。

## 5. 楼宇异常温度

由路点触发 `trigger temperature <seq>`，同样抓拍即返回。判定顺序：

1. 有 `THERMAL_TOPIC` 时读热成像峰值，离群高温视为检出；
2. 否则在 HSV 空间找高饱和暖色块（红/橙），取面积大且靠近画面中心者；
3. 在标记附近裁一块送 OCR，解析“2F / 2 层 / 二楼”；
4. OCR 失败时用 `FLOOR_REGIONS[seq]` 的标定值；
5. 两者都没有时上报 `UNKNOWN_FLOOR`，不编造楼层。

状态字段：`OK`（有标记且有楼层）、`NO_MARKER`、`UNKNOWN_FLOOR`、`PARTIAL`、`ERROR`。

## 6. 联调自测（不依赖 ROS）

```python
import sys, time
sys.path.insert(0, "vision_system")
from core.judge_client import get_client, reset_client

cfg = {"transport": ["file"], "file": {"path": "/tmp/judge_selftest.jsonl"}}
c = get_client(config=cfg, node_name="selftest")
c.report("meter", {"meter_id": 1, "value": 0.42, "unit": "MPa"}, text="1 号表读数为 0.42 MPa", seq=1)
c.flush(2.0)
print(c.stats())      # {"sent": 1, ...}
reset_client()
```

看到 `sent: 1` 即通道正常；再用 `tail` 检查 jsonl 内容。

## 7. 已知边界

- 指针角度依赖表盘无遮挡、光斑不落在指针上；强反光时对比度不足会自动判为
  不可读（`value=None`），不会输出随机数。
- 温度任务的暖色标记检测是可见光近似，若现场物料采用其他表达方式，
  需要替换 `_find_marker` 的色域或改用训练好的检测模型。
- 两项任务的路点坐标必须在现场标定后填入 `goals.py`，否则不会触发。
