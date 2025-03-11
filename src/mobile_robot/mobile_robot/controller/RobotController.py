import rclpy.node

from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..util.Singleton import singleton


@singleton
class RobotController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__robot_data = RobotService(node)
        self.__sensor = SensorService()

    def get_radar_data(self, angle):
        return self.__sensor.get_radar_data(angle)

    def with_robot_connect(self):
        self.__robot_data.with_robot_connect()