import rclpy
from rclpy.node import Node

from .util.Logger import Logger
from .controller.RobotController import RobotController
from .controller.BModuleController import BModuleController


class BModule(Node):
    def __init__(self):
        super().__init__('b_module')

        Logger().set_ros_logger(self.get_logger())

        b_module = BModuleController(self)
        robot = RobotController(self)

        robot.with_robot_connect()
        select = int(input("从1 - 15开始, 0 退出, 你的选择是: "))
        robot.set_start_led(False)
        robot.reset_arm()

        b_module.task(select)

        self.destroy_node()
        rclpy.shutdown()
        exit(0)



def main():
    rclpy.init()

    node = BModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
