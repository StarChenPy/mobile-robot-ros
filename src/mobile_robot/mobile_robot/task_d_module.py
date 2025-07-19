import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.TaskDController import TaskDController
from .util.Logger import Logger


class TaskDModule(Node):
    def __init__(self):
        super().__init__("task_d_module")

        Logger().set_ros_logger(self.get_logger(), LoggingSeverity.INFO)

        controller = TaskDController(self)

        controller.run()

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():

    try:
        rclpy.init()

        node = TaskDModule()

        rclpy.spin(node)
    except KeyboardInterrupt:
        Logger().info("任务D模块接收到键盘中断信号, 退出.")


if __name__ == '__main__':
    main()
