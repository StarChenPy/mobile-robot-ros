import typing

import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from chassis_msgs.srv import ResetOdom

class OdomService(Behaviour):
    """
    里程计服务
    """

    def __init__(self):
        super().__init__("里程计ROS服务客户端")
        self.node = None
        self.ros_client = None
        self.future = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node: rclpy.Node = kwargs["node"]
        self.ros_client = self.node.create_client(ResetOdom, "/chassis/reset_odom")

    def initialise(self) -> None:
        request: ResetOdom.Request = Blackboard.get("/odom/request")
        self.future = self.ros_client.call_async(request)

    def update(self) -> Status:
        if self.ros_client is None:
            self.logger.error("ROS 客户端未初始化")
            return Status.INVALID

        rclpy.spin_once(self.node)

        if not self.future.done():
            return Status.RUNNING

        response: ResetOdom.Response = self.future.result()
        if response.success:
            Blackboard.set("/odom/response", response)
            return Status.SUCCESS
        else:
            self.logger.error("里程计重置服务失败")
            return Status.FAILURE
