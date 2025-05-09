import rclpy
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose2D
from base_nav2.action import NavCMD
from ..model.NavigationPoint import NavigationPoint


class NavigationActionClient(Behaviour):
    def __init__(self, points: list[NavigationPoint], linear_speed: float, rotation_speed: float, rotation_acc: float, rotation_decel: float, reverse: bool):
        super().__init__("Navigation Action")
        self.node = None
        self.action_client = None
        self.goal_handle = None
        self.result_future = None
        self.sent_goal = False

        self.points = points
        self.linear_speed = linear_speed
        self.rotation_speed = rotation_speed
        self.rotation_acc = rotation_acc
        self.rotation_decel = rotation_decel
        self.reverse = reverse

    def setup(self, **kwargs) -> None:
        self.node = kwargs['node']
        self.action_client = ActionClient(self.node, NavCMD, '/nav2_action')

    def initialise(self):
        self.sent_goal = False
        self.goal_handle = None
        self.result_future = None

    def update(self):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("导航服务端未准备好")
            return Status.FAILURE

        if not self.sent_goal:
            self.__send_goal()
            return Status.RUNNING

        if self.result_future.done():
            self.logger.debug("导航完成")
            return Status.SUCCESS

        return Status.RUNNING

    def __send_goal(self):
        goal_msg = NavCMD.Goal()

        for p in self.points:
            pose2d = Pose2D(x=float(p.x), y=float(p.y), theta=0.0)
            goal_msg.points.append(pose2d)

        goal_msg.linear_vel = float(self.linear_speed)
        goal_msg.rotation_vel = float(self.rotation_speed)
        goal_msg.rotate_acc = float(self.rotation_acc)
        goal_msg.rotate_decel = float(self.rotation_decel)
        goal_msg.heading = float(self.points[-1].yaw)
        goal_msg.back = self.reverse

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        self.goal_handle = send_goal_future.result()
        if not self.goal_handle.accepted:
            self.logger.error("导航请求被拒绝")
            return

        self.result_future = self.goal_handle.get_result_async()
        self.sent_goal = True

    def terminate(self, new_status):
        if self.goal_handle and self.goal_handle.accepted:
            self.goal_handle.cancel_goal_async()
