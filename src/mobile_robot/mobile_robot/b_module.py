import time

import rclpy
from rclpy.node import Node

from .robot.robot import MobileRobot
from .robot.param.navigation_path import NavPath
from .robot.param.arm_movement import ArmMovementParam


class BModule(Node):
    def __init__(self):
        super().__init__('b_module')

        self.robot = MobileRobot(self)

        select = int(input("等待按键按下, 1 - 15, 0 退出\n"))

        match select:
            case 0:
                exit(0)
            case 1:
                # 直线1米
                self.robot.navigation(NavPath.B_MODULE_1)
            case 2:
                # 旋转180度
                self.robot.rotate(180)
            case 3:
                # 抓水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.READY_GRAB_APPLE_LOW)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_LOW, is_block=True)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_END, is_block=True)
            case 4:
                # 果仓1到起始区
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)
                self.robot.navigation(NavPath.B_MODULE_4)
            case 5:
                # 起始区到果仓1
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)
                self.robot.navigation(NavPath.B_MODULE_5)
            case 6:
                # 采摘1到起始区
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)
                self.robot.navigation(NavPath.B_MODULE_6)
            case 7:
                # 起始区到采摘1
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)
                self.robot.navigation(NavPath.B_MODULE_7)
            case 8:
                # 起始区到采摘1抓高水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)
                self.robot.navigation(NavPath.B_MODULE_7)
                self.robot.arm_control(ArmMovementParam.READY_GRAB_APPLE_TALL)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_TALL)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_END)
            case 9:
                # 起始区到采摘1抓中水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)
                self.robot.navigation(NavPath.B_MODULE_7)
                self.robot.arm_control(ArmMovementParam.READY_GRAB_APPLE_MIDDLE)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_MIDDLE)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_END)
            case 10:
                # 起始区到采摘1抓低水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)
                self.robot.navigation(NavPath.B_MODULE_7)
                self.robot.arm_control(ArmMovementParam.READY_GRAB_APPLE_LOW)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_LOW)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_END)
            case 11:
                # 起始区到果仓一号放水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)
                self.robot.navigation(NavPath.B_MODULE_11)
                self.robot.rotate(90)
                self.robot.arm_control(ArmMovementParam.READY_PULL_GUO_CANG)
                self.robot.arm_control(ArmMovementParam.PULL_GUO_CANG)
                self.robot.arm_control(ArmMovementParam.MOVING)
            case 12:
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)

                self.robot.navigation(NavPath.B_MODULE_7)

                self.robot.arm_control(ArmMovementParam.READY_GRAB_APPLE_TALL)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_TALL)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_END)

                self.robot.ping_revise(20)

                self.robot.navigation(NavPath.B_MODULE_12)


def main():
    rclpy.init()

    node = BModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
