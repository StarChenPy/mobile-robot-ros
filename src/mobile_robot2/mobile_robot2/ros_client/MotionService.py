import typing

import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from base_motion_ros2.srv import BaseMotion

class MotionService(Behaviour):
    """
    基本移动服务
    """

    def __init__(self):
        super().__init__("基本移动ROS服务客户端")
        self.node = None
        self.ros_client = None
        self.future = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node: rclpy.Node = kwargs.get("node")
        self.ros_client = self.node.create_client(BaseMotion, "/base_motion")

    def initialise(self) -> None:
        request: BaseMotion.Request = Blackboard.get("/base_motion/request")
        self.future = self.ros_client.call_async(request)

    def update(self) -> Status:
        if self.ros_client is None:
            self.logger.error("ROS 客户端未初始化")
            return Status.INVALID

        rclpy.spin_once(self.node)

        if not self.future.done():
            return Status.RUNNING

        response: BaseMotion.Response = self.future.result()
        if response.success:
            Blackboard.set("/base_motion/response", response)
            return Status.SUCCESS
        else:
            self.logger.error("基础运动服务失败")
            return Status.FAILURE
