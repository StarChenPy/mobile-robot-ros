import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.ShandongTrialsController import ShandongTrialsController
from .util.Logger import Logger


class ShandongTrialsModule(Node):
    def __init__(self):
        super().__init__("shandong_trials_module")

        Logger().set_ros_logger(self.get_logger(), LoggingSeverity.INFO)

        test = ShandongTrialsController(self)

        test.run()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = ShandongTrialsModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
