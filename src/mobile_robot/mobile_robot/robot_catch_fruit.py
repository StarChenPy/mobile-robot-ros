from enum import Enum

import rclpy
import time

from rclpy.node import Node

from .robot.param.arm_movement import ArmMovementParam
from .robot.param.navigation_path import NavPath
from .robot.robot import MobileRobot
from .robot.data_type import *


def calculate_rectangle_center(rectangle: Rectangle):
    center_x = (rectangle.x1 + rectangle.x2) / 2
    center_y = (rectangle.y1 + rectangle.y2) / 2
    return center_x, center_y


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
            self.grab_fruits(NavPath.PICKING_CORRIDOR_1, ["Yellow Grape", "Green Grape"])

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

    def grab_fruits(self, nav_path: NavPath, fruits: list):
        """
        扫描并抓取水果
        """
        self.robot.arm_control(ArmMovementParam.READY_RECOGNITION_GRAPE)
        self.robot.arm_control(ArmMovementParam.RECOGNITION_GRAPE)
        self.robot.navigation(nav_path, 0.05, False)
        while rclpy.ok() and self.robot.get_navigation_state():
            result_list = self.robot.vision()
            for result in result_list:
                if result.classId not in fruits:
                    continue

                center_x, center_y = calculate_rectangle_center(result.box)
                print(f"x {center_x}, y {center_y}")

                if not 180 < center_x < 400:
                    continue

                self.robot.cancel_navigation()

                # 调用不同高度下的抓取程序
                if center_y > 300:
                    print(11111)
                    self.robot.arm_control(ArmMovementParam.READY_GRAB_GRAPE)
                    self.robot.arm_control(ArmMovementParam.GRAB_GRAPE)
                elif center_y > 200:
                    print(22222)
                    self.robot.arm_control(ArmMovementParam.READY_GRAB_GRAPE)
                    self.robot.arm_control(ArmMovementParam.GRAB_GRAPE)
                else:
                    print(33333)
                    self.robot.arm_control(ArmMovementParam.READY_GRAB_GRAPE)
                    self.robot.arm_control(ArmMovementParam.GRAB_GRAPE)

                # 收回夹爪
                self.robot.arm_control(ArmMovementParam.READY_PUT_FRUIT_INTO_BASKET, True)
                self.robot.arm_control(ArmMovementParam.PUT_FRUIT_INTO_BASKET)

                self.robot.arm_control(ArmMovementParam.READY_RECOGNITION_GRAPE)
                self.robot.arm_control(ArmMovementParam.RECOGNITION_GRAPE)

                self.robot.navigation(nav_path, 0.05, False, False)
        self.robot.arm_control(ArmMovementParam.MOVING)


def main():
    rclpy.init()

    robot = RobotCatchFruit()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
