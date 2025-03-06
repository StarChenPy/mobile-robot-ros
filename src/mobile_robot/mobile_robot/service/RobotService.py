import rclpy.node

from ..dao.RobotDataDao import RobotDataDao
from ..util.Singleton import singleton


@singleton
class RobotService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__robot_data = RobotDataDao(node)

    def with_robot_connect(self):
        self.__logger.info("[机器人] 正在链接")
        if self.__robot_data.get_robot_data() is not None:
            self.__logger.info("[机器人] 链接成功")
            return