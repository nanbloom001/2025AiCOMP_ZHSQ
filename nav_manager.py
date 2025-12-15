#!/usr/bin/env python3
# coding:utf-8
"""
navigation_manager.py

功能要点：
 - 20 个目标点（包含 traffic_light / rog / ocr / none）
 - 到点后停车，触发视觉 /navigation_task（trigger_ocr / trigger_rog / trigger_traffic_light）
 - 等待视觉返回（/ocr_task_done, /yolo_task_done, /traffic_light_done）
    - OCR/ROG (yolo)：等待 task_timeout（可配置），超时自动跳过
    - traffic_light：等待 'green_light'（无默认超时），但可手动 skip
 - 键盘控制：p = pause, r = resume, s = skip 当前任务, q = 退出
 - 运行时可通过 /nav_ctrl/param_update (std_msgs/String, JSON) 更新参数
 - 发布 /nav_ctrl/status (std_msgs/String, JSON)
 - ROS Noetic (Python3) 兼容
"""

import rospy
import threading
import json
import sys
import termios
import tty
import select
from time import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler

class NavigationManager:
    def __init__(self):
        rospy.init_node('navigation_manager', anonymous=False)

        # ---- 可配置参数（rosparam / 运行时可更新）
        self.default_nav_timeout = rospy.get_param('~default_nav_timeout', 25.0)    # 导航超时 (s)
        self.default_task_timeout = rospy.get_param('~default_task_timeout', 15.0)  # OCR/ROG 超时 (s)
        self.post_task_pause = rospy.get_param('~post_task_pause', 1.0)            # 触发视觉后等待 (s)
        self.startup_delay = rospy.get_param('~startup_delay', 5.0)
        self.keyboard_enabled = rospy.get_param('~keyboard_enabled', True)

        # ---- 暂时使用 navigation_OT.py 中的点
        # 格式: [x, y, yaw, task_type]
        self.goals = [

            ###用红绿灯判断的模型

            # [1.069, -0.04, -0.038, 'none'],# 001 | 第一个斑马线前   | traffic_light | 向上直行

            # [1.069, -0.04, -0.038, 'none'],# 001 | A社区右。面向上   | 无任务 | 向上直行 ！！！加个向前上看的点
            # [2.5732, -0.0861, 1.4602, 'none'],# 001 | A社区右。面向左   | yolo | 向上直行
            # [2.5732, -0.0861, 1.4602, 'none'],# 001 | A社区右。面向上   | yolo | 向上直行 ！！！加个恢复直行的点

        
            # [3.279, -0.082, 0.262, 'none'],# 002 | 右上角，面向上 | 无任务 | 向上直行
            # [3.386, -0.151, 1.4979, 'none'],# 003 | 右上角，面向左 | 无任务 | 左转
            

            ###用社区人员判断 + 电动车的模型
            
            # [3.4052, 0.422, 1.533, 'none'],# 001 | A社区上，面向左   | 无任务 | 向上直行 
            # [3.3846, 0.422, -3.027, 'none'],# 001 | A社区上，面向下   | yolo | 向上直行 
            # [3.4052, 0.422, 1.533, 'none'],# 001 | A社区上，面向左   | 无任务 | 恢复直行


            ###用判断静止直行的模型

            # [3.442, 0.989, 1.4617, 'none'],# 004 | 静止直行指示牌面前，面向左 | yolo | 向左直行
            # [3.451, 0.967, 3.109, 'none'],# 005 | 静止直行指示牌面前，面向下 | 无任务 | 原地左转


            ###用社区人员判断 + 电动车的模型

            # [2.695, 0.968, 3.081, 'none'],# 006 | A，B社区正中间，面向下 | 无任务 | 向下直行
            # [2.760, 1.028, 1.6967, 'none'],# 007 | A，B社区正中间，面向B | yolo | 原地右转  
            # [2.789, 1.021, -1.705, 'none'],# 008 | A，B社区正中间，面向A | yolo | 原地右转
            # [2.695, 0.968, 3.081, 'none'],# 006 | A，B社区正中间，面向下 | 无任务 | 恢复直行
  

            ###用楼宇火灾检测的模型

            # [2.007, 1.034, 3.094, 'none'],# 010 | 楼宇A前，面向下 | yolo | 向下直行
            # [1.852, 1.059, 1.428, 'none'],# 011 | 楼宇A前，面向左 | 无任务 | 原地右转

            ###用社区人员判断 + 电动车的模型
            # [1.852, 1.059, 1.428, 'none'],# 011 | B社区下，面向左 | 无任务 | 原地右转 ！！！
            # [1.852, 1.059, 1.428, 'none'],# 011 | B社区下，面向上 | yolo | 原地右转 ！！！
            # [1.852, 1.059, 1.428, 'none'],# 011 | B社区下，面向左 | 无任务 | 恢复直行 ！！！

            ###用楼宇火灾检测的模型

            # [1.920, 1.834, 1.415, 'none'],# 012 | 楼宇C前，面向左 | 无任务 | 向左直行 ！！！这里可能用B社区下那个点更好？
            # [1.920, 1.834, 1.415, 'none'],# 012 | 楼宇C前，面向下 | yolo | 向左直行 ！！！
            # [1.920, 1.834, 1.415, 'none'],# 012 | 楼宇C前，面向左 | 无任务 | 恢复直行 ！！！

            ###用红绿灯判断的模型

            # [1.920, 1.834, 1.415, 'none'],# 012 | 楼宇C前，面向左 | traffic_light | 恢复直行 ！！！

            ###用垃圾检测模型
            
            # [1.920, 3.254, 1.432, 'none'],# 013 | 垃圾桶上，面向左 | 无任务 | 向左直行 ！！！
            # [1.920, 3.254, 1.432, 'none'],# 013 | 垃圾桶上，面向下 | yolo | 向左直行 ！！！
            # [1.920, 3.254, 1.432, 'none'],# 013 | 垃圾桶上，面向左 | 无任务 | 恢复直行 ！！！

            ###

            # [1.920, 3.254, 1.432, 'none'],# 013 | 第二个红绿灯左边，面向左 | 无任务 | 向左直行
            # [2.000, 3.292, -3.074, 'none'],# 014 | 第二个红绿灯左边，面向下 | 无任务 |   ！！！

            ###用车牌检测的模型

            # [0.828, 3.318, 3.117, 'none'],# 015 | 车牌1前，面向车牌 | ocr |             
            # [0.828, 2.734, 3.111, 'none'],# 016 | 车牌1前，面向右 | 无任务 | 恢复直行     ！！！要改


            # [0.828, 2.734, 3.111, 'none'],# 016 | 车牌2前，面向右 | 无任务 |              ！！！要改
            # [0.828, 2.734, 3.111, 'none'],# 015 | 车牌2前，面向车牌 | ocr |             
            # [0.828, 2.734, 3.111, 'none'],# 016 | 车牌2前，面向右 | 无任务 | 恢复直行     ！！！要改


            # [0.828, 2.734, 3.111, 'none'],# 016 | 车牌3前，面向右 | 无任务 | 恢复直行     ！！！要改
            # [0.740, 2.122, 3.107, 'none'],# 017 | 面向车牌3 | ocr |   
            # [0.828, 2.734, 3.111, 'none'],# 016 | 车牌3前，面向右 | 无任务 | 恢复直行     ！！！要改


            ###用楼宇火灾检测的模型

            # [ 'none'],# 016 | 楼宇B前，面向右 | 无任务 |      ！！！要改
            # [ 'none'],# 016 | 楼宇B前，面向上 | yolo |      ！！！要改
            # [ 'none'],# 016 | 楼宇B前，面向右 | 无任务 |      ！！！在这直接斜着看电动车行吗？待讨论

            ###用电动车倒伏检测模型

            #原倒车入库点
            #[0.520, -0.050, 0.135, 'none'],     # 无任务 
            #[0.066, -0.061, 0.041, 'none'],    # 无任务



            ############ 以上为新点，下为旧点位备份 ############
            #########################################################

            # [-0.4053, -1.1052, 1.6902, 'none'],

            # # 正对第二个红绿灯斑马线前（等待红绿灯）
            # [0.1801, -1.0755, 0.1630, 'none'],

            # # 正对第三个大厦（路过点）
            # [2.8235, 0.8104, 1.6822, 'none'],

            # # 正对垃圾桶（ROG/YOLO识别）
            # [2.8366, 2.3067, -1.5437, 'none'],

            # # 过第二个红绿灯向左（路过点）
            # [1.8527, 3.2476, -2.9267, 'none'],

            # # 第一块车牌
            # [0.7143, 3.2433, -3.0183, 'none'],

            # # 第二块车牌
            # [0.7885, 2.6114, -2.9194, 'none'],

            # # 第三块车牌
            # [0.7745, 2.0028, 3.1294, 'none'],

            # # 正对第二个大厦（路过点）
            # [0.7343, 2.0609, -0.0190, 'none'],

            # # 面向电瓶车 1（ROG 识别）
            # [0.6724, 0.9397, -3.1296, 'none'],

            # # 起点外面，面向墙壁
            # [0.6915, 0.2107, -1.5365, 'none'],
            # [0.520, -0.050, 0.135, 'none'],
            # [0.066, -0.061, 0.041, 'none'],

            ###
            [1.069, -0.061, -0.038, 'none'],      # 第一个红绿灯 - OCR识别
            [3.279, -0.082, 0.262, 'none'],     # 无任务
            [3.386, -0.151, 1.4979, 'none'],     # 无任务

            [3.442, 0.989, 1.4617, 'none'],     # 无任务
            [3.451, 0.967, 3.109, 'none'],     # 无任务

            [2.695, 0.968, 3.081, 'none'],     # 无任务

            [2.760, 1.028, 1.6967, 'none'],     # shequ
            [2.789, 1.021, -1.705, 'none'],     # shequ
            [2.783, 0.999, 3.086, 'none'],     # 无任务

            [2.007, 1.034, 3.094, 'none'],     # 无任务
            [1.852, 1.059, 1.428, 'none'],     # 无任务

            [1.958, 1.834, 1.415, 'none'],      # 红绿灯 - OCR识别

            [2.142, 3.254, 1.432, 'none'],     # 无任务
            [2.081, 3.292, -3.074, 'none'],     # 无任务


            [1.035, 3.318, 3.117, 'none'],     # 车牌１
            [0.828, 2.734, 3.111, 'none'],     # 车牌2
            [0.740, 2.122, 3.107, 'none'],     # 车牌3
            [0.673, 2.088, -1.645, 'none'],     # 

            [0.520, -0.050, 0.135, 'none'],     # 无任务
            [0.066, -0.061, 0.041, 'none'],    # 无任务

        ]

        # ---- 内部状态
        self.current_goal_index = 0
        self.paused = False
        self.skip_requested = False
        self.shutdown_requested = False
        self.executing_nav = False
        self.task_completed = False
        self.task_result = None
        self.client = None

        # ---- ROS IO
        self.task_pub = rospy.Publisher('/navigation_task', String, queue_size=5)
        self.status_pub = rospy.Publisher('/nav_ctrl/status', String, queue_size=5)
        rospy.Subscriber('/nav_ctrl/param_update', String, self.param_update_cb)

        # 视觉完成订阅（Manager 在等待任务阶段会用这些回调设置标志）
        rospy.Subscriber('/ocr_task_done', String, self.ocr_done_cb)
        rospy.Subscriber('/yolo_task_done', String, self.yolo_done_cb)
        rospy.Subscriber('/traffic_light_done', String, self.traffic_light_done_cb)

        # AMCL pose 订阅（保留用于监控/扩展）
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)

        # 键盘线程
        if self.keyboard_enabled:
            kb = threading.Thread(target=self.keyboard_listener, daemon=True)
            kb.start()

        rospy.loginfo("NavigationManager initialized, %d goals loaded." % len(self.goals))
        rospy.sleep(self.startup_delay)
        self.publish_status()

    # ---------------- callbacks ----------------
    def param_update_cb(self, msg):
        try:
            data = json.loads(msg.data)
            for k, v in data.items():
                if k == 'default_nav_timeout':
                    self.default_nav_timeout = float(v)
                elif k == 'default_task_timeout':
                    self.default_task_timeout = float(v)
                elif k == 'post_task_pause':
                    self.post_task_pause = float(v)
                else:
                    rospy.set_param(k, v)
            rospy.loginfo("[nav_manager] params updated: %s" % json.dumps(data))
            self.publish_status()
        except Exception as e:
            rospy.logerr("param_update parse error: %s" % str(e))

    def ocr_done_cb(self, msg: String):
        self.task_completed = True
        self.task_result = msg.data
        rospy.loginfo("[nav_manager] ocr_done: %s" % str(msg.data))

    def yolo_done_cb(self, msg: String):
        self.task_completed = True
        self.task_result = msg.data
        rospy.loginfo("[nav_manager] yolo_done: %s" % str(msg.data))

    def traffic_light_done_cb(self, msg: String):
        self.task_completed = True
        self.task_result = msg.data
        rospy.loginfo("[nav_manager] traffic_light_done: %s" % str(msg.data))

    def amcl_cb(self, msg):
        # 用于扩展监控位置；当前不依赖
        pass

    # ---------------- keyboard control ----------------
    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        self.print_keyboard_help()
        try:
            while not rospy.is_shutdown() and not self.shutdown_requested:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch == 'p':
                        self.request_pause()
                    elif ch == 'r':
                        self.request_resume()
                    elif ch == 's':
                        self.request_skip()
                    elif ch == 'q':
                        rospy.logwarn("Keyboard requested shutdown (q).")
                        self.shutdown_requested = True
                        self.paused = False
                        break
                rospy.sleep(0.01)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def print_keyboard_help(self):
        rospy.loginfo("Keyboard: p=PAUSE, r=RESUME, s=SKIP, q=QUIT")

    def request_pause(self):
        if not self.paused:
            rospy.logwarn("[nav_manager] PAUSE requested")
            self.paused = True
            try:
                if self.client:
                    self.client.cancel_goal()
            except:
                pass
            self.publish_status()

    def request_resume(self):
        if self.paused:
            rospy.loginfo("[nav_manager] RESUME requested")
            self.paused = False
            self.publish_status()

    def request_skip(self):
        rospy.logwarn("[nav_manager] SKIP requested")
        self.skip_requested = True
        try:
            if self.client:
                self.client.cancel_goal()
        except:
            pass
        self.publish_status()

    # ---------------- helpers ----------------
    def publish_status(self):
        st = {
            'paused': bool(self.paused),
            'current_goal_index': int(self.current_goal_index),
            'total_goals': len(self.goals),
            'executing_nav': bool(self.executing_nav),
            'default_nav_timeout': float(self.default_nav_timeout),
            'default_task_timeout': float(self.default_task_timeout),
            'post_task_pause': float(self.post_task_pause)
        }
        try:
            self.status_pub.publish(String(json.dumps(st)))
        except:
            pass

    def create_move_base_goal(self, goal):
        """将 [x, y, yaw] 转换为 MoveBaseGoal 消息"""
        mb = MoveBaseGoal()
        mb.target_pose.header.frame_id = "map"
        mb.target_pose.header.stamp = rospy.Time.now()
        mb.target_pose.pose.position.x = float(goal[0])
        mb.target_pose.pose.position.y = float(goal[1])
        q = quaternion_from_euler(0, 0, float(goal[2]))
        mb.target_pose.pose.orientation.x = q[0]
        mb.target_pose.pose.orientation.y = q[1]
        mb.target_pose.pose.orientation.z = q[2]
        mb.target_pose.pose.orientation.w = q[3]
        return mb

    # ---------------- navigation ----------------
    def navigate_to_goal(self, idx):
        g = self.goals[idx]
        nav_timeout = float(g[4]) if len(g) > 4 else self.default_nav_timeout

        if not self.client:
            self.client = SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server not available.")
            return False

        mb_goal = self.create_move_base_goal(g)
        goal_sent = False
        start_time = time()

        while not rospy.is_shutdown() and not self.shutdown_requested:
            # user skip
            if self.skip_requested:
                rospy.logwarn("Skipping goal %d by user.", idx)
                self.skip_requested = False
                try:
                    self.client.cancel_goal()
                except:
                    pass
                return False

            # pause
            if self.paused:
                if goal_sent:
                    try:
                        self.client.cancel_goal()
                    except:
                        pass
                    goal_sent = False
                rospy.loginfo_throttle(2, "Paused before/while navigating.")
                rospy.sleep(0.2)
                continue

            # send goal when not sent
            if not goal_sent:
                try:
                    self.client.send_goal(mb_goal)
                    rospy.loginfo("Sent goal %d -> (%.3f, %.3f) task=%s nav_timeout=%.1f",
                                  idx, float(g[0]), float(g[1]), str(g[3]), nav_timeout)
                    goal_sent = True
                    start_time = time()
                    self.executing_nav = True
                    self.publish_status()
                except Exception as e:
                    rospy.logerr("Failed to send goal: %s" % str(e))
                    return False

            state = self.client.get_state()
            if state == 3:
                rospy.loginfo("Goal %d SUCCEEDED." % idx)
                self.executing_nav = False
                self.publish_status()
                return True
            if state in [4,5,9]:
                rospy.logwarn("Goal %d failed with state %d." % (idx, state))
                self.executing_nav = False
                self.publish_status()
                return False

            # nav timeout
            elapsed = time() - start_time
            if elapsed > nav_timeout:
                rospy.logwarn("Goal %d navigation timeout %.1fs (limit %.1fs)." % (idx, elapsed, nav_timeout))
                try:
                    self.client.cancel_goal()
                except:
                    pass
                self.executing_nav = False
                self.publish_status()
                return False

            rospy.sleep(0.12)

        try:
            self.client.cancel_goal()
        except:
            pass
        return False

    # ---------------- task trigger & wait ----------------
    def trigger_visual_task(self, task_type):
        if not task_type or task_type == 'none':
            return
        if task_type == 'ocr':
            msg = "trigger_ocr"
        elif task_type in ['rog', 'yolo']:
            msg = "trigger_rog"
        elif task_type == 'traffic_light':
            msg = "trigger_traffic_light"
        else:
            msg = str(task_type)
        rospy.loginfo("Publish visual trigger: %s" % msg)
        self.task_pub.publish(String(msg))

    def wait_for_task_completion(self, task_type, task_timeout=None):
        self.task_completed = False
        self.task_result = None

        if task_type == 'none':
            return True
        if task_type == 'wait':
            rospy.sleep(2.0)
            return True

        if task_type == 'traffic_light':
            rospy.loginfo("Waiting traffic light -> need 'green_light' (manual skip allowed).")
            while not rospy.is_shutdown() and not self.shutdown_requested:
                if self.skip_requested:
                    rospy.logwarn("Skip requested during traffic_light wait.")
                    self.skip_requested = False
                    return False
                if self.paused:
                    rospy.loginfo_throttle(2, "Paused during traffic_light wait.")
                    rospy.sleep(0.2)
                    continue
                if self.task_completed:
                    if isinstance(self.task_result, str) and self.task_result.lower() == "green_light":
                        rospy.loginfo("Green light received.")
                        return True
                    else:
                        rospy.logwarn("Traffic light returned not-green: %s" % str(self.task_result))
                        return False
                rospy.sleep(0.12)
            return False

        # ocr / rog behavior
        timeout = float(task_timeout) if task_timeout is not None else float(self.default_task_timeout)
        rospy.loginfo("Waiting for '%s' done, timeout %.1fs..." % (task_type, timeout))
        start = time()
        while not rospy.is_shutdown() and not self.shutdown_requested:
            if self.skip_requested:
                rospy.logwarn("Skip requested during task wait.")
                self.skip_requested = False
                return False
            if self.paused:
                rospy.loginfo_throttle(2, "Paused during task wait.")
                rospy.sleep(0.2)
                continue
            if self.task_completed:
                rospy.loginfo("Task %s completed, result: %s" % (task_type, str(self.task_result)))
                return True
            if time() - start > timeout:
                rospy.logwarn("Task %s timeout after %.1fs." % (task_type, timeout))
                return False
            rospy.sleep(0.12)
        return False

    # ---------------- main loop ----------------
    def run(self):
        rospy.loginfo("NavigationManager running.")
        rospy.sleep(0.5)

        while not rospy.is_shutdown() and not self.shutdown_requested and self.current_goal_index < len(self.goals):
            idx = self.current_goal_index
            rospy.loginfo("=== Goal %d / %d ===" % (idx+1, len(self.goals)))

            # pause before sending
            if self.paused:
                rospy.loginfo_throttle(2, "Paused before sending goal.")
                rospy.sleep(0.2)
                continue

            reached = self.navigate_to_goal(idx)

            task_type = self.goals[idx][3] if len(self.goals[idx]) > 3 else 'none'
            task_timeout = None  # could be extended to per-goal timeout if desired

            if reached:
                rospy.loginfo("Reached goal %d. Triggering task: %s" % (idx, task_type))
                # trigger (visual module is separate)
                self.trigger_visual_task(task_type)
                # wait for completion according to task rules
                task_ok = self.wait_for_task_completion(task_type, task_timeout)
                rospy.sleep(self.post_task_pause)
                if task_ok:
                    rospy.loginfo("Task at goal %d succeeded. Move to next." % idx)
                    self.current_goal_index += 1
                else:
                    rospy.logwarn("Task at goal %d failed/timeout/skipped. Skipping to next." % idx)
                    self.current_goal_index += 1
            else:
                rospy.logwarn("Failed to reach goal %d. Skipping." % idx)
                self.current_goal_index += 1

            self.publish_status()

        rospy.loginfo("NavigationManager finished (all goals or shutdown).")
        self.publish_status()

if __name__ == '__main__':
    try:
        manager = NavigationManager()
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted by ROS.")
