import rclpy.node

from ..popo.Direction import Direction
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..util.Singleton import singleton


@singleton
class RobotController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__robot = RobotService(node)
        self.__sensor = SensorService()

    def get_radar_data(self, angle):
        return self.__sensor.get_radar_data(angle)

    def get_distance_from_wall(self, direction: Direction) -> float:
        return self.__sensor.get_distance_from_wall(direction)

    def get_angle_from_wall(self, direction: Direction) -> float:
        return self.__sensor.get_angle_from_wall(direction)

    def with_robot_connect(self):
        self.__logger.info("[机器人] 正在链接")
        self.__robot.with_robot_connect()
        self.__logger.info("[机器人] 链接成功")

    def with_start_button(self):
        self.__robot.start_button()
        self.__logger.info("[机器人] 开始按钮被按下")
        self.set_start_led(True)

    def set_start_led(self, state: bool):
        self.__robot.start_led(state)