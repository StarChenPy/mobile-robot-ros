import rclpy.node

from ..param import ArmMovement
from ..service.ArmService import ArmService
from ..service.RobotService import RobotService
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class RobotController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__robot = RobotService(node)
        self.__arm = ArmService(node)

    def with_robot_connect(self):
        self.__logger.info("正在链接")
        self.__robot.with_robot_connect()
        self.__logger.info("链接成功")

    def with_start_button(self):
        self.__robot.with_start_button()
        self.__logger.info("开始按钮被按下")

    def set_start_led(self, state: bool):
        self.__robot.set_start_led(state)

    def reset_arm(self):
        self.__arm.back_origin()
        self.__arm.control(ArmMovement.MOVING)
