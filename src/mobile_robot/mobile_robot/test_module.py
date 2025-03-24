import time

import rclpy
from rclpy.node import Node

from .controller.GrabFruitController import GrabFruitController
from .param import ArmMovement, NavigationPath
from .controller.RobotController import RobotController
from .controller.MoveController import MoveController
from .controller.ArmController import ArmController


class TestModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.__arm = ArmController(self)
        self.__move = MoveController(self)
        self.__robot = RobotController(self)
        self.__grab = GrabFruitController(self)

        self.__robot.with_robot_connect()

        input("123")

        self.__move.navigation([NavigationPath.WAREHOUSE_CORRECTIVE_POINT, NavigationPath.WAREHOUSE_1_POINT])
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
