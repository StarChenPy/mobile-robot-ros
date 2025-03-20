import time

import rclpy
from rclpy.node import Node

from .popo.ServoMotor import ServoMotor
from .popo.ArmMovement import ArmMovement
from .popo.MotorMovement import MotorMovement
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

        input("按任意键开始游戏")

        # self.__arm.reset()

        # 1号框
        self.__arm.control(ArmMovement(MotorMovement(-30, 18), ServoMotor(0, 0, 2.5, 7)))
        self.__arm.control(ArmMovement(MotorMovement(-30, 18), ServoMotor(0, 0, 2.5, 10)))

        # 2号框
        self.__arm.control(ArmMovement(MotorMovement(0, 18), ServoMotor(0, 0, 1, 7)))
        self.__arm.control(ArmMovement(MotorMovement(0, 18), ServoMotor(0, 0, 1, 10)))

        # 3号框
        self.__arm.control(ArmMovement(MotorMovement(28, 18), ServoMotor(0, 0, 2.5, 7)))
        self.__arm.control(ArmMovement(MotorMovement(28, 18), ServoMotor(0, 0, 2.5, 10)))


        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = TestModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
