import typing

import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from position_motor_ros2.srv import CtrlImpl

class LiftMotorService(Behaviour):
    """
    旋转电机服务
    """

    def __init__(self):
        super().__init__("旋转电机ROS服务客户端")
        self.node = None
        self.ros_client = None
        self.future = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node: rclpy.Node = kwargs.get("node")
        self.ros_client = self.node.create_client(CtrlImpl, "/position_motor/rotate_motor/ctrl")

    def initialise(self) -> None:
        request: CtrlImpl.Request = Blackboard.get("/rotate_motor/request")
        self.future = self.ros_client.call_async(request)

    def update(self) -> Status:
        if self.ros_client is None:
            self.logger.error("ROS 客户端未初始化")
            return Status.INVALID

        rclpy.spin_once(self.node)

        if not self.future.done():
            return Status.RUNNING

        response: CtrlImpl.Response = self.future.result()
        if response.success:
            Blackboard.set("/rotate_motor/response", response)
            return Status.SUCCESS
        else:
            self.logger.error("旋转电机服务失败")
            return Status.FAILURE
