import time

import rclpy
from rclpy.node import Node

from .controller.CModuleController import CModuleController
from .controller.RobotController import RobotController


class TestModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.__robot = RobotController(self)
        self.__grab = CModuleController(self)

        self.__robot.with_robot_connect()

        input("123")

        time.sleep(1)

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = TestModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
