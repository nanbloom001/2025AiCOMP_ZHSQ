"""封装 move_base 的导航执行器。"""

from time import time
import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

from .config import Goal, NavConfig, NavState


class NavExecutor:
    """
    负责与 move_base action server 交互，执行导航任务。
    """
    def __init__(self):
        self.client = SimpleActionClient("move_base", MoveBaseAction)

    def create_move_base_goal(self, goal: Goal) -> MoveBaseGoal:
        """将 Goal 数据类转换为 MoveBaseGoal 消息。"""
        mb = MoveBaseGoal()
        mb.target_pose.header.frame_id = "map"
        mb.target_pose.header.stamp = rospy.Time.now()
        mb.target_pose.pose.position.x = float(goal.x)
        mb.target_pose.pose.position.y = float(goal.y)
        q = quaternion_from_euler(0, 0, float(goal.yaw))
        mb.target_pose.pose.orientation.x = q[0]
        mb.target_pose.pose.orientation.y = q[1]
        mb.target_pose.pose.orientation.z = q[2]
        mb.target_pose.pose.orientation.w = q[3]
        return mb

    def navigate(self, goal: Goal, cfg: NavConfig, state: NavState) -> bool:
        """
        执行导航到指定目标点。
        
        Args:
            goal (Goal): 目标点信息。
            cfg (NavConfig): 导航配置。
            state (NavState): 当前导航状态（用于检查暂停/跳过/关闭请求）。
            
        Returns:
            bool: 导航成功返回 True，失败/超时/被跳过返回 False。
        """
        nav_timeout = goal.nav_timeout if goal.nav_timeout else cfg.default_nav_timeout

        rospy.loginfo("Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server not available.")
            return False

        mb_goal = self.create_move_base_goal(goal)
        goal_sent = False
        start_time = time()

        while not rospy.is_shutdown() and not state.shutdown_requested:
            if state.skip_requested:
                rospy.logwarn("Skipping current goal by user request.")
                state.skip_requested = False
                try:
                    self.client.cancel_goal()
                except Exception:
                    pass
                state.executing_nav = False
                return False

            if state.paused:
                if goal_sent:
                    try:
                        self.client.cancel_goal()
                    except Exception:
                        pass
                    goal_sent = False
                rospy.loginfo_throttle(2, "Paused before/while navigating.")
                rospy.sleep(0.2)
                continue

            if not goal_sent:
                try:
                    self.client.send_goal(mb_goal)
                    rospy.loginfo(
                        "Sent goal -> (%.3f, %.3f) task=%s nav_timeout=%.1f",
                        goal.x,
                        goal.y,
                        goal.task_type,
                        nav_timeout,
                    )
                    goal_sent = True
                    start_time = time()
                    state.executing_nav = True
                except Exception as exc:
                    rospy.logerr("Failed to send goal: %s" % str(exc))
                    state.executing_nav = False
                    return False

            state_code = self.client.get_state()
            if state_code == 3:
                rospy.loginfo("Goal succeeded.")
                state.executing_nav = False
                return True
            if state_code in [4, 5, 9]:
                rospy.logwarn("Goal failed with state %d." % state_code)
                state.executing_nav = False
                return False

            elapsed = time() - start_time
            if elapsed > nav_timeout:
                rospy.logwarn(
                    "Navigation timeout %.1fs (limit %.1fs). Last state: %s", elapsed, nav_timeout, str(state_code)
                )
                try:
                    self.client.cancel_goal()
                except Exception:
                    pass
                state.executing_nav = False
                return False

            rospy.sleep(0.05)

        try:
            self.client.cancel_goal()
        except Exception:
            pass
        state.executing_nav = False
        return False
