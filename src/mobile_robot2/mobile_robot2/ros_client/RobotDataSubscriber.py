import typing

import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from web_message_transform_ros2.msg import RobotData

class RobotDataSubscriber(Behaviour):
    """
    用于订阅机器人数据的类
    """

    def __init__(self):
        super().__init__("机器人数据ROS订阅客户端")
        self.node = None
        self.ros_client = None
        self.data = None
        self.data_old = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node: rclpy.Node = kwargs.get("node")
        self.ros_client = self.node.create_subscription(RobotData, "/web_transform_node/robot_data", self.__callback, 10)

    def update(self) -> Status:
        if self.ros_client is None:
            self.logger.error("ROS 客户端未初始化")
            return Status.INVALID

        rclpy.spin_once(self.node)

        if self.data is not None and self.data != self.data_old:
            Blackboard.set("/robot_data/raw", self.data)
            self.data_old = self.data
            return Status.SUCCESS
        else:
            return Status.RUNNING

    def __callback(self, msg):
        self.data = msg
