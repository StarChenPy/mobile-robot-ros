import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.DebugController import DebugController
from .util.Logger import Logger


class DebugModule(Node):
    def __init__(self):
        super().__init__("debug_module")

        Logger().set_ros_logger(self.get_logger(), LoggingSeverity.DEBUG)

        controller = DebugController(self)

        controller.run()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():

    try:
        rclpy.init()

        node = DebugModule()

        rclpy.spin(node)
    except KeyboardInterrupt:
        Logger().info("Debug模块接收到键盘中断信号, 退出.")


if __name__ == '__main__':
    main()
