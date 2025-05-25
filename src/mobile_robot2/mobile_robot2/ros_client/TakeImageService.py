import typing

import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from camera_orbbec2.srv import ReqImage

class TakeImageService(Behaviour):
    """
    拍摄服务 响应时间约100ms-200ms
    """

    def __init__(self):
        super().__init__("拍照ROS服务客户端")
        self.node = None
        self.ros_client = None
        self.future = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node: rclpy.Node = kwargs.get("node")
        self.ros_client = self.node.create_client(ReqImage, "/camera/req_image")

    def initialise(self) -> None:
        self.future = self.ros_client.call_async(ReqImage.Request(cmd=0, sync_snap=True))

    def update(self) -> Status:
        if self.ros_client is None:
            self.logger.error("ROS 客户端未初始化")
            return Status.INVALID

        self.ros_client.wait_for_service(timeout_sec=5)
        rclpy.spin_once(self.node)

        if not self.future.done():
            return Status.RUNNING

        if self.future.result().success:
            Blackboard.set("camera/raw", self.future.result())
            return Status.SUCCESS
        else:
            self.logger.error("拍照失败")
            return Status.FAILURE
