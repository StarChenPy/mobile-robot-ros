import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.TaskBController import TaskBController
from .util.Logger import Logger


class TaskBModule(Node):
    def __init__(self):
        super().__init__("task_b_module")

        Logger().set_ros_logger(self.get_logger(), LoggingSeverity.INFO)

        controller = TaskBController(self)

        controller.run()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():

    try:
        rclpy.init()

        node = TaskBModule()

        rclpy.spin(node)
    except KeyboardInterrupt:
        Logger().info("任务B模块接收到键盘中断信号, 退出.")


if __name__ == '__main__':
    main()
