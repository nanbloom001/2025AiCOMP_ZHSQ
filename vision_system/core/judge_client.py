#!/usr/bin/env python3
# coding: utf-8
"""裁判系统上报客户端。

2026 年总决赛不再使用语音播报，各队的识别结果需直接提交给现场裁判系统。
本模块把一次业务的最终结果封装成一条 JSON 记录，按配置选择一个或多个通道发出：

    udp    —— 局域网广播/单播，最低延迟，适合现场同一交换机
    http   —— POST JSON，适合裁判系统提供 HTTP 接口的场合
    ros    —— 发布 std_msgs/String，适合裁判系统订阅 ROS 话题
    file   —— 追加 jsonl，永远开启，作为本地留档与断网重放

三条设计约束：

1. 不阻塞任务。report() 只入队，发送在网络线程完成；队列满时丢弃并计入统计，
   绝不因为网络问题卡住导航或识别。
2. 幂等。每条记录的 msg_id 由（轮次、任务、序号、结果摘要）决定，结果不变则
   msg_id 不变，重复发送不会在裁判端产生第二条有效记录。
3. 可降级。任何通道失败都不抛异常到调用方，只写本地文件并记入失败计数，
   现场可以据此在赛后补交。
"""

import hashlib
import json
import os
import queue
import socket
import threading
import time

try:
    import rospy
    from std_msgs.msg import String
except Exception:  # 允许脱离 ROS 单独调试
    rospy = None
    String = None


DEFAULT_CONFIG = {
    "enabled": True,
    # 可多选，例如 ["udp", "file"]；file 始终追加，用于留档
    "transport": ["udp", "file"],
    "udp": {"host": "255.255.255.255", "port": 9900},
    "http": {"url": "http://127.0.0.1:8080/api/report", "timeout": 0.8},
    "ros": {"topic": "/judge/report"},
    "file": {"path": "~/.aic_judge_outbox/reports.jsonl"},
    "team": "三人行必有我师焉",
    "run_id_prefix": "AIC2026",
    "retry": 3,
    "queue_size": 256,
}


def _expand(path):
    return os.path.abspath(os.path.expanduser(path))


def _merge(base, override):
    out = dict(base)
    for key, val in (override or {}).items():
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], val)
        else:
            out[key] = val
    return out


class JudgeClient(object):
    """线程安全的结果上报器，进程内单例（见 get_client）。"""

    def __init__(self, config=None, node_name="vision_master", run_id=None, logger=None):
        self.cfg = _merge(DEFAULT_CONFIG, config)
        self.node_name = node_name
        self._logger = logger
        self.run_id = run_id or self._make_run_id()
        self._queue = queue.Queue(maxsize=int(self.cfg["queue_size"]))
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._sent = 0
        self._failed = 0
        self._dropped = 0
        self._ros_pub = None
        self._udp_sock = None
        self._worker = threading.Thread(target=self._loop, name="judge-sender")
        self._worker.daemon = True
        self._worker.start()
        self._log("judge client ready, transports=%s run_id=%s"
                  % (self.cfg.get("transport"), self.run_id))

    # ---------------- public ----------------
    def report(self, task, payload, text=None, seq=0, checkpoint=None,
               evidence=None, blocking=False):
        """提交一条结果。

        task      任务名，与 /vision/cmd 中的任务名一致（meter、temperature 等）
        payload   结构化字段，直接给裁判系统解析
        text      面向人读的一句话（现场大屏/播报回落时使用）
        seq       任务序号，对应路点配置中的 seq_id
        evidence  证据索引，例如原图路径、耗时
        blocking  调试用：等待本条发送完成
        """
        record = self._build(task, payload, text, seq, checkpoint, evidence)
        if not self.cfg.get("enabled", True):
            self._write_file(record)
            return record["msg_id"]

        try:
            self._queue.put_nowait((record, time.time()))
        except queue.Full:
            with self._lock:
                self._dropped += 1
            self._log("queue full, drop %s" % record["msg_id"])
            self._write_file(record, tag="DROPPED")
            return record["msg_id"]

        if blocking:
            self.flush(timeout=3.0)
        return record["msg_id"]

    def flush(self, timeout=2.0):
        """等待队列排空，返回仍未发出的条数。"""
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            if self._queue.empty():
                return 0
            time.sleep(0.02)
        return self._queue.qsize()

    def stats(self):
        with self._lock:
            return {
                "sent": self._sent,
                "failed": self._failed,
                "dropped": self._dropped,
                "pending": self._queue.qsize(),
                "run_id": self.run_id,
            }

    def close(self):
        self._stop.set()
        self.flush(timeout=2.0)
        self._worker.join(timeout=2.0)

    # ---------------- internals ----------------
    def _log(self, msg):
        if self._logger is not None:
            try:
                self._logger(msg)
                return
            except Exception:
                pass
        print("[judge] %s" % msg)

    def _make_run_id(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return "%s_%s" % (self.cfg.get("run_id_prefix", "RUN"), stamp)

    def _build(self, task, payload, text, seq, checkpoint, evidence):
        payload = payload or {}
        # 结果摘要参与 msg_id：同一结果重复上报不产生新记录，结果变化才产生新记录
        digest = hashlib.sha1(
            json.dumps(payload, sort_keys=True, ensure_ascii=False).encode("utf-8")
        ).hexdigest()[:12]
        msg_id = "%s-%s-%s-%s" % (self.run_id, task, int(seq), digest)
        record = {
            "schema": "aic.judge.v1",
            "msg_id": msg_id,
            "run_id": self.run_id,
            "team": self.cfg.get("team", ""),
            "node": self.node_name,
            "task": task,
            "seq": int(seq),
            "checkpoint": checkpoint,
            "ts": round(time.time(), 3),
            "text": text or "",
            "payload": payload,
            "evidence": evidence or {},
        }
        return record

    def _loop(self):
        while not self._stop.is_set():
            try:
                record, created = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            ok = self._send_with_retry(record)
            with self._lock:
                if ok:
                    self._sent += 1
                else:
                    self._failed += 1
            if not ok:
                self._write_file(record, tag="FAILED")
            self._queue.task_done()

    def _send_with_retry(self, record):
        transports = list(self.cfg.get("transport") or [])
        if "file" not in transports:
            transports.append("file")
        attempts = max(1, int(self.cfg.get("retry", 3)))
        last_ok = False
        for transport in transports:
            for i in range(attempts):
                try:
                    if self._dispatch(transport, record):
                        last_ok = True
                        break
                except Exception as exc:
                    self._log("%s attempt %d failed: %s" % (transport, i + 1, exc))
                time.sleep(0.05 * (i + 1))
        return last_ok

    def _dispatch(self, transport, record):
        if transport == "file":
            self._write_file(record)
            return True
        if transport == "udp":
            return self._send_udp(record)
        if transport == "http":
            return self._send_http(record)
        if transport == "ros":
            return self._send_ros(record)
        self._log("unknown transport: %s" % transport)
        return False

    def _send_udp(self, record):
        cfg = self.cfg.get("udp", {})
        if self._udp_sock is None:
            self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        body = json.dumps(record, ensure_ascii=False).encode("utf-8")
        self._udp_sock.sendto(body, (cfg.get("host", "127.0.0.1"), int(cfg.get("port", 9900))))
        return True

    def _send_http(self, record):
        import urllib.request
        cfg = self.cfg.get("http", {})
        body = json.dumps(record, ensure_ascii=False).encode("utf-8")
        req = urllib.request.Request(
            cfg.get("url"), data=body,
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST")
        with urllib.request.urlopen(req, timeout=float(cfg.get("timeout", 0.8))) as resp:
            return 200 <= getattr(resp, "status", resp.getcode()) < 300

    def _send_ros(self, record):
        if rospy is None or String is None:
            return False
        if self._ros_pub is None:
            topic = self.cfg.get("ros", {}).get("topic", "/judge/report")
            self._ros_pub = rospy.Publisher(topic, String, queue_size=20)
            time.sleep(0.1)
        self._ros_pub.publish(String(json.dumps(record, ensure_ascii=False)))
        return True

    def _write_file(self, record, tag=None):
        cfg = self.cfg.get("file", {})
        path = _expand(cfg.get("path", DEFAULT_CONFIG["file"]["path"]))
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            line = dict(record)
            if tag:
                line["_tag"] = tag
            with open(path, "a", encoding="utf-8") as fp:
                fp.write(json.dumps(line, ensure_ascii=False) + "\n")
        except Exception as exc:
            self._log("write %s failed: %s" % (path, exc))


_CLIENT = None
_CLIENT_LOCK = threading.Lock()


def get_client(config=None, node_name="vision_master", run_id=None, logger=None):
    """进程内单例；任务模块直接调用即可，不必自己持有连接。"""
    global _CLIENT
    with _CLIENT_LOCK:
        if _CLIENT is None:
            _CLIENT = JudgeClient(config=config, node_name=node_name,
                                  run_id=run_id, logger=logger)
        return _CLIENT


def reset_client():
    """测试用：销毁单例。"""
    global _CLIENT
    with _CLIENT_LOCK:
        if _CLIENT is not None:
            _CLIENT.close()
        _CLIENT = None
