import typing

import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from web_message_transform_ros2.msg import RobotData

class RobotCtrlPublisher(Behaviour):
    """
    用于发布机器人数据的类
    """

    def __init__(self):
        super().__init__("机器人数据ROS发布客户端")
        self.node = None
        self.ros_client = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node: rclpy.Node = kwargs.get("node")
        self.ros_client = self.node.create_publisher(RobotData, "/web_transform_node/robot_ctrl", 10)

    def update(self) -> Status:
        if self.ros_client is None:
            self.logger.error("ROS 客户端未初始化")
            return Status.INVALID

        if not Blackboard.exists("robot_ctrl"):
            self.logger.error("黑板上没有robot_ctrl数据")
            return Status.FAILURE

        robot_ctrl = Blackboard.get("robot_ctrl")
        self.ros_client.publish(robot_ctrl)

        return Status.SUCCESS
