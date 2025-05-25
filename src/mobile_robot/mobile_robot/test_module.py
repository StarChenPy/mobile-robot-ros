import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.TestController import TestController
from .controller.GrabFruitOnTreeController import GrabFruitOnTreeController
from .util.Logger import Logger


class TestModule(Node):
    def __init__(self):
        super().__init__("test_module")

        Logger().set_ros_logger(self.get_logger(), LoggingSeverity.INFO)

        test = TestController(self)

        test.run()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = TestModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
