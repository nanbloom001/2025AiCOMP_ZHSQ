#!/usr/bin/env python3
# coding:utf-8
"""模块化导航管理器协调器 (Orchestrator)。"""

import json
import sys
import termios
import threading
import tty
import select

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_manager_mod.config import NavConfig, NavState, Goal, Task, load_config_from_ros, apply_param_update
from nav_manager_mod.goals import load_goals
from nav_manager_mod.nav_executor import NavExecutor
from nav_manager_mod.task_handler import TaskHandler


class NavigationManager:
    """
    导航管理器主类 (Navigation Manager)。
    
    功能:
    1. 协调整个比赛流程：导航 -> 视觉任务 -> 导航。
    2. 初始化各个子模块（配置、目标加载、导航执行器、任务处理器）。
    3. 订阅 ROS 话题，监听键盘输入，并运行主控制循环。
    """
    def __init__(self):
        rospy.init_node("navigation_manager", anonymous=False)

        self.cfg: NavConfig = load_config_from_ros()
        self.state: NavState = NavState()
        self.goals = load_goals()

        self.nav = NavExecutor()
        self.tasks = TaskHandler()

        self.status_pub = rospy.Publisher("/nav_ctrl/status", String, queue_size=5)

        rospy.Subscriber("/nav_ctrl/param_update", String, self.param_update_cb)
        # 统一使用 /vision/done 接收视觉任务结果
        rospy.Subscriber("/vision/done", String, self.visual_done_cb)
        
        # 订阅统一的状态话题 (如红绿灯状态)
        rospy.Subscriber("/vision/status", String, self.status_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)

        if self.cfg.keyboard_enabled:
            kb = threading.Thread(target=self.keyboard_listener, daemon=True)
            kb.start()

        rospy.loginfo("NavigationManager initialized, %d goals loaded." % len(self.goals))
        rospy.sleep(self.cfg.startup_delay)
        self.publish_status()

    # ---------------- callbacks ----------------
    def param_update_cb(self, msg):
        """处理参数更新回调。"""
        try:
            apply_param_update(self.cfg, msg.data)
            self.publish_status()
        except Exception as exc:
            rospy.logerr("param_update parse error: %s" % str(exc))

    def visual_done_cb(self, msg: String):
        """
        通用视觉任务完成回调。
        消息格式: done_<task_type> <result_data>
        例如: done_doll 3
        """
        raw = msg.data.strip()
        parts = raw.split(' ', 1)
        header = parts[0]
        result_data = parts[1] if len(parts) > 1 else ""

        if header.startswith("done_"):
            task_type = header[5:] # remove 'done_'
            # 检查是否是当前正在等待的任务
            if self.state.current_task == task_type:
                self.state.task_completed = True
                self.state.task_result = result_data
                rospy.loginfo("[nav_manager] Task '%s' done. Result: %s" % (task_type, result_data))
            else:
                rospy.logwarn("[nav_manager] Received done for '%s' but waiting for '%s'. Ignored." % (task_type, self.state.current_task))
        else:
            # 兼容旧格式或未知格式，直接作为结果
            self.state.task_completed = True
            self.state.task_result = raw
            rospy.loginfo("[nav_manager] Received raw done msg: %s" % raw)

    def status_cb(self, msg: String):
        """
        通用状态回调 (包括红绿灯等)。
        消息格式: status_<task> <info>
        例如: status_traffic_light green
        """
        raw = msg.data.strip()
        parts = raw.split(' ', 1)
        header = parts[0]
        info = parts[1] if len(parts) > 1 else ""
        
        if header == "status_traffic_light":
            # 兼容旧逻辑，将结果存入 task_result 供 task_handler 轮询
            self.state.task_result = info
            rospy.loginfo_throttle(1.0, "[nav_manager] traffic_light_status: %s" % info)
        else:
            # 其他状态信息暂不处理，可扩展
            pass

    def amcl_cb(self, msg):
        # 预留用于未来监控/分析的钩子
        pass

    # ---------------- keyboard control ----------------
    def keyboard_listener(self):
        """
        键盘监听线程函数。
        
        监听标准输入，处理以下按键：
        p: 暂停 (PAUSE)
        r: 恢复 (RESUME)
        s: 跳过当前目标 (SKIP)
        q: 退出程序 (QUIT)
        """
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        self.print_keyboard_help()
        try:
            while not rospy.is_shutdown() and not self.state.shutdown_requested:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch == "p":
                        self.request_pause()
                    elif ch == "r":
                        self.request_resume()
                    elif ch == "s":
                        self.request_skip()
                    elif ch == "q":
                        rospy.logwarn("Keyboard requested shutdown (q).")
                        self.state.shutdown_requested = True
                        self.state.paused = False
                        break
                rospy.sleep(0.01)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def print_keyboard_help(self):
        rospy.loginfo("Keyboard: p=PAUSE, r=RESUME, s=SKIP, q=QUIT")

    def request_pause(self):
        if not self.state.paused:
            rospy.logwarn("[nav_manager] PAUSE requested")
            self.state.paused = True
            try:
                self.nav.client.cancel_goal()
            except Exception:
                pass
            self.publish_status()

    def request_resume(self):
        if self.state.paused:
            rospy.loginfo("[nav_manager] RESUME requested")
            self.state.paused = False
            self.publish_status()

    def request_skip(self):
        rospy.logwarn("[nav_manager] SKIP requested")
        self.state.skip_requested = True
        try:
            self.nav.client.cancel_goal()
        except Exception:
            pass
        self.publish_status()

    # ---------------- helpers ----------------
    def publish_status(self):
        """发布当前状态到 /nav_ctrl/status 话题。"""
        st = {
            "paused": bool(self.state.paused),
            "current_goal_index": int(self.state.current_goal_index),
            "total_goals": len(self.goals),
            "executing_nav": bool(self.state.executing_nav),
            "default_nav_timeout": float(self.cfg.default_nav_timeout),
            "default_task_timeout": float(self.cfg.default_task_timeout),
            "post_task_pause": float(self.cfg.post_task_pause),
        }
        try:
            self.status_pub.publish(String(json.dumps(st)))
        except Exception:
            pass

    # ---------------- main loop ----------------
    def run(self):
        """
        主运行循环。
        
        依次遍历目标点列表，执行以下步骤：
        1. 导航到目标点。
        2. 触发相应的视觉任务。
        3. 等待任务完成。
        4. 移动到下一个目标点。
        """
        rospy.loginfo("NavigationManager running.")
        rospy.sleep(0.5)

        while (
            not rospy.is_shutdown()
            and not self.state.shutdown_requested
            and self.state.current_goal_index < len(self.goals)
        ):
            idx = self.state.current_goal_index
            goal: Goal = self.goals[idx]
            rospy.loginfo("=== Goal %d / %d ===" % (idx + 1, len(self.goals)))

            if self.state.paused:
                rospy.loginfo_throttle(2, "Paused before sending goal.")
                rospy.sleep(0.2)
                continue

            reached = self.nav.navigate(goal, self.cfg, self.state)
            
            # 获取任务列表，如果没有则默认为 [Task("none", 0)]
            task_list = goal.tasks if goal.tasks else [Task("none", 0)]
            task_timeout = None  # 如果需要，可在此处扩展每个目标的任务超时时间

            if reached:
                rospy.loginfo("Reached goal %d. Tasks: %s" % (idx, str(task_list)))
                
                all_tasks_ok = True
                for task_item in task_list:
                    if self.state.skip_requested:
                        rospy.logwarn("Skip requested, aborting remaining tasks at this goal.")
                        self.state.skip_requested = False
                        all_tasks_ok = False
                        break
                        
                    rospy.loginfo(">>> Triggering task: %s (SeqID: %d)" % (task_item.type, task_item.seq_id))
                    self.tasks.trigger(task_item.type, task_item.seq_id)
                    
                    task_ok = self.tasks.wait_for_completion(task_item.type, self.cfg, self.state, task_timeout)
                    if not task_ok:
                        rospy.logwarn("Task '%s' failed/timeout/skipped." % task_item.type)
                        all_tasks_ok = False
                        # 根据需求决定是否继续执行下一个任务，这里假设失败一个就跳过该点剩余任务
                        break
                    
                    rospy.loginfo("Task '%s' succeeded." % task_item.type)
                    rospy.sleep(0.5) # 任务间短暂间隔

                rospy.sleep(self.cfg.post_task_pause)
                self.state.current_goal_index += 1
                if all_tasks_ok:
                    rospy.loginfo("All tasks at goal %d succeeded. Move to next." % idx)
                else:
                    rospy.logwarn("Tasks at goal %d not fully completed. Moving to next." % idx)
            else:
                rospy.logwarn("Failed to reach goal %d. Skipping." % idx)
                self.state.current_goal_index += 1

            self.publish_status()

        rospy.loginfo("NavigationManager finished (all goals or shutdown).")
        self.publish_status()


def main():
    try:
        manager = NavigationManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted by ROS.")


if __name__ == "__main__":
    main()
