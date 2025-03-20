import rclpy
from rclpy.node import Node

from mobile_robot.mobile_robot.param import ArmMovement
from .controller.RobotController import RobotController
from .controller.MoveController import MoveController
from .controller.ArmController import ArmController


class TestModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.__arm = ArmController(self)
        self.__move = MoveController(self)
        self.__robot = RobotController(self)

        self.__robot.with_robot_connect()

        self.__arm.reset()
        self.__arm.control(ArmMovement.MOVING)


def main():
    rclpy.init()

    node = TestModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
