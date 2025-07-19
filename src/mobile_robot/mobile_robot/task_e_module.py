import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.TaskEController import TaskEController
from .util.Logger import Logger


class TaskEModule(Node):
    def __init__(self):
        super().__init__("task_b_module")

        Logger().set_ros_logger(self.get_logger(), LoggingSeverity.INFO)

        controller = TaskEController(self)

        controller.run()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():

    try:
        rclpy.init()

        node = TaskEModule()

        rclpy.spin(node)
    except KeyboardInterrupt:
        Logger().info("任务E模块接收到键盘中断信号, 退出.")


if __name__ == '__main__':
    main()
