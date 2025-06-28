import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.PuchipuchiController import PuchipuchiController
from .util.Logger import Logger


class PuchipuchiModule(Node):
    def __init__(self):
        super().__init__("shandong_trials_module")

        Logger().set_ros_logger(self.get_logger(), LoggingSeverity.INFO)

        controller = PuchipuchiController(self)

        controller.run()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():

    try:
        rclpy.init()

        node = PuchipuchiModule()

        rclpy.spin(node)
    except KeyboardInterrupt:
        Logger().info("Puchi模块因键盘中断而关闭.")


if __name__ == '__main__':
    main()
