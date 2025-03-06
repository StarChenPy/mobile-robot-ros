import rclpy.node

from ..service.RobotService import RobotService
from ..util.Singleton import singleton


@singleton
class RobotController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__robot_data = RobotService(node)

    def with_robot_connect(self):
        self.__robot_data.with_robot_connect()