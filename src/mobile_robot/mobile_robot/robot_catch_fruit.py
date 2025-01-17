import rclpy
import time

from rclpy.node import Node

from .robot.param.arm_movement import ArmMovementParam
from .robot.param.navigation_path import NavPath
from .robot.robot import MobileRobot


class RobotCatchFruit(Node):
    def __init__(self):
        super().__init__('robot_catch_fruit')

        self.robot = MobileRobot(self)
        self.run()

    def run(self):
        # self.robot.with_start_button()
        # self.robot.arm_reset()
        # time.sleep(2)
        # self.robot.ping_revise(15, 0)
        # time.sleep(2)
        # self.grab_basket()
        # self.robot.navigation(NavPath.STARTING_POINT2PICKING_POINT)
        while True:
            rclpy.spin_once(self)
            self.robot.vision()
            time.sleep(1)

    def grab_basket(self):
        """
        抓篮子并放到车上
        """
        self.robot.arm_control(ArmMovementParam.READY_GRAB_BASKET)
        self.robot.arm_control(ArmMovementParam.GRAB_BASKET)
        self.robot.arm_control(ArmMovementParam.READY_PUT_BASKET)
        self.robot.arm_control(ArmMovementParam.PUT_BASKET_CENTER)
        # 未完待续


def main():
    rclpy.init()

    robot = RobotCatchFruit()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
