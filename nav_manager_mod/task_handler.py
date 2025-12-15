"""任务触发与等待逻辑。"""

from time import time
import rospy
from std_msgs.msg import String

from .config import NavConfig, NavState


class TaskHandler:
    """处理视觉任务的触发与结果等待。"""
    def __init__(self):
        self.task_pub = rospy.Publisher("/vision/cmd", String, queue_size=5)

    def trigger(self, task_type: str, seq_id: int = 0) -> None:
        """
        根据任务类型发布触发消息到 /vision/cmd。
        
        Args:
            task_type (str): 任务类型 (例如: 'ocr', 'doll', 'fire', 'traffic_light')。
            seq_id (int): 序列号。
        """
        if not task_type or task_type == "none":
            return
        
        # 构造消息: trigger <task_type> <seq_id>
        msg = "trigger %s %d" % (task_type, seq_id)
        rospy.loginfo("Publish visual trigger: %s" % msg)
        self.task_pub.publish(String(msg))

    def wait_for_completion(
        self, task_type: str, cfg: NavConfig, state: NavState, task_timeout=None
    ) -> bool:
        """
        等待视觉任务完成。
        
        支持超时、暂停、跳过。
        对于 'traffic_light' 任务，会一直等待直到收到 'green_light' 或被手动跳过。
        
        Args:
            task_type (str): 任务类型。
            cfg (NavConfig): 导航配置。
            state (NavState): 当前状态。
            task_timeout (Optional[float]): 任务超时时间。如果为 None，则使用默认配置。
            
        Returns:
            bool: 任务成功完成返回 True，超时/跳过/失败返回 False。
        """
        state.task_completed = False
        state.task_result = None
        state.current_task = task_type  # 设置当前正在等待的任务类型

        if task_type in [None, "none"]:
            return True
        if task_type == "wait":
            rospy.sleep(2.0)
            return True

        if task_type == "traffic_light":
            rospy.loginfo("Waiting traffic light -> need 'green_light' (manual skip allowed).")
            green_count = 0
            while not rospy.is_shutdown() and not state.shutdown_requested:
                if state.skip_requested:
                    rospy.logwarn("Skip requested during traffic_light wait.")
                    state.skip_requested = False
                    # 即使跳过，也发送停止指令
                    self.task_pub.publish(String("stop traffic_light"))
                    return False
                if state.paused:
                    rospy.loginfo_throttle(2, "Paused during traffic_light wait.")
                    rospy.sleep(0.2)
                    continue
                
                # 检查当前状态 (由 manager.py 回调更新到 state.task_result)
                current_status = str(state.task_result).lower() if state.task_result else "null"
                
                if current_status == "green_light":
                    green_count += 1
                    if green_count % 10 == 0:
                        rospy.loginfo("Seeing Green Light... count=%d" % green_count)
                elif current_status == "red_light":
                    if green_count > 0: rospy.loginfo("Status changed to RED, reset count.")
                    green_count = 0
                    rospy.loginfo_throttle(0.5, "Seeing Red Light, waiting...")
                elif current_status == "yellow_light":
                    if green_count > 0: rospy.loginfo("Status changed to YELLOW, reset count.")
                    green_count = 0
                    rospy.loginfo_throttle(0.5, "Seeing Yellow Light, waiting...")
                else:
                    if green_count > 0:
                        rospy.loginfo("Traffic light status changed to %s, resetting count." % current_status)
                    green_count = 0
                    rospy.loginfo_throttle(0.5, "Waiting for green... Current: %s" % current_status)

                # 连续 5 帧绿灯则放行
                if green_count >= 2:
                    rospy.loginfo("Traffic light confirmed GREEN. Go!")
                    # 发送停止指令给视觉节点
                    self.task_pub.publish(String("stop traffic_light"))
                    return True
                
                rospy.sleep(0.05) # 降低休眠时间，提高响应速度
            return False

        timeout = float(task_timeout) if task_timeout is not None else float(cfg.default_task_timeout)
        rospy.loginfo("Waiting for '%s' done, timeout %.1fs..." % (task_type, timeout))
        start = time()
        while not rospy.is_shutdown() and not state.shutdown_requested:
            if state.skip_requested:
                rospy.logwarn("Skip requested during task wait.")
                state.skip_requested = False
                self.task_pub.publish(String("stop %s" % task_type))
                return False
            if state.paused:
                rospy.loginfo_throttle(2, "Paused during task wait.")
                rospy.sleep(0.2)
                continue
            if state.task_completed:
                rospy.loginfo("Task %s completed, result: %s" % (task_type, str(state.task_result)))
                return True
            if time() - start > timeout:
                rospy.logwarn("Task %s timeout after %.1fs." % (task_type, timeout))
                self.task_pub.publish(String("stop %s" % task_type))
                return False
            rospy.sleep(0.12)
        return False
