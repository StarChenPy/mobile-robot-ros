import typing

import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from base_motion_ros2.srv import BaseMotion

class SensorService(Behaviour):
    """
    基本移动服务
    """

    def __init__(self):
        super().__init__("传感器矫正ROS服务客户端")
        self.node = None
        self.ros_client = None
        self.future = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node: rclpy.Node = kwargs.get("node")
        self.ros_client = self.node.create_client(BaseMotion, "/user/sensor_service_cmd")

    def initialise(self) -> None:
        request: BaseMotion.Request = Blackboard.get("/sensor/request")
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
            Blackboard.set("/sensor/response", response)
            return Status.SUCCESS
        else:
            self.logger.error("传感器矫正服务失败")
            return Status.FAILURE
