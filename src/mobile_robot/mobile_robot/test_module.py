import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.RobotController import RobotController
from .controller.TestModuleController import TestModuleController


class TestModule(Node):
    def __init__(self):
        super().__init__("test_module")

        self.get_logger().set_level(LoggingSeverity.INFO)

        self.__test = TestModuleController(self)
        self.__robot = RobotController(self)

        self.__robot.with_robot_connect()

        self.__test.run()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = TestModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
