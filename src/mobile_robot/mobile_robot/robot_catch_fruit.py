from enum import Enum

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

        while True:
            input("回车继续")

            self.robot.arm_reset()
            self.robot.ping_revise(20, 0)

            self.grab_basket()
            self.robot.navigation(NavPath.STARTING_POINT2PICKING_POINT)
            self.robot.ir_revise(14)
            self.robot.rotate(-90)
            self.robot.ping_revise(20, 0)
            self.robot.grab_fruits(NavPath.PICKING_CORRIDOR_1, ["Yellow Grape", "Green Grape"])

    def grab_basket(self):
        """
        抓篮子并放到车上
        """
        self.robot.arm_control(ArmMovementParam.READY_GRAB_BASKET_1)
        self.robot.arm_control(ArmMovementParam.READY_GRAB_BASKET_2)
        self.robot.arm_control(ArmMovementParam.GRAB_BASKET)

        self.robot.arm_control(ArmMovementParam.READY_PUT_BASKET)
        time.sleep(2)
        self.robot.arm_control(ArmMovementParam.PUT_BASKET_CENTER)
        self.robot.arm_control(ArmMovementParam.FINISH_PUT_BASKET)

        self.robot.arm_control(ArmMovementParam.BASKET_MOVING)


def main():
    rclpy.init()

    robot = RobotCatchFruit()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
