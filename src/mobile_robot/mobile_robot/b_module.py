import rclpy
from rclpy.node import Node

from .robot.robot import MobileRobot, FruitHeight
from .robot.param.navigation_path import NavPath, ORCHARD_1_POINT
from .robot.param.arm_movement import ArmMovementParam
from .robot.util.math import calculate_rectangle_center
from .robot.robot import get_fruit_height, Pose


class BModule(Node):
    def __init__(self):
        super().__init__('b_module')

        self.robot = MobileRobot(self)

        select = int(input("等待按键按下, 1 - 15, 0 退出\n"))

        match select:
            case -1:
                self.robot.ping_revise(30)
                self.robot.ping_revise(50)
                self.robot.ping_revise(10)
            case 0:
                exit(0)
            case 1:
                # 直线1米
                self.robot.init_pose(Pose(0, 0, 0))
                self.robot.navigation(NavPath.B_MODULE_1)
            case 2:
                # 旋转180度
                self.robot.rotate(180)
            case 3:
                # 抓水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.READY_GRAB_APPLE_TALL)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_TALL, is_block=True)
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

                self.robot.init_pose(ORCHARD_1_POINT)
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
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_END, True)
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
                # 起始区到果园摘水果然后去果仓1号放水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)

                self.robot.navigation(NavPath.B_MODULE_7)

                self.robot.arm_control(ArmMovementParam.READY_GRAB_APPLE_TALL)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_TALL)
                self.robot.arm_control(ArmMovementParam.GRAB_APPLE_END)

                self.robot.ping_revise(20)

                self.robot.navigation(NavPath.B_MODULE_12)
                self.robot.rotate(90)

                self.robot.arm_control(ArmMovementParam.READY_PULL_GUO_CANG)
                self.robot.arm_control(ArmMovementParam.PULL_GUO_CANG)
                self.robot.arm_control(ArmMovementParam.MOVING)
            case 13:
                # 起始区到果仓识别一个水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)

                self.robot.navigation(NavPath.B_MODULE_5)
                self.robot.ping_revise(20)
                self.robot.rotate(90)

                self.robot.arm_control(ArmMovementParam.RECOGNITION_WAREHOUSE)
                while True:
                    s = input("等待...")
                    if s == "1":
                        self.robot.arm_control(ArmMovementParam.MOVING)
                        exit(0)
                    print(self.robot.vision())
            case 14:
                # 起始区到果园识别一个水果
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)

                self.robot.navigation(NavPath.B_MODULE_7)

                self.robot.arm_control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT)

                while True:
                    s = input("等待...")
                    if s == "1":
                        self.robot.arm_control(ArmMovementParam.MOVING)
                        exit(0)
                    print(self.robot.vision())
            case 15:
                # 起始区到果园识别一个水果的高低
                self.robot.arm_control(ArmMovementParam.RESET)
                self.robot.arm_control(ArmMovementParam.MOVING)

                self.robot.navigation(NavPath.B_MODULE_7)

                self.robot.arm_control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT)

                while True:
                    s = input("等待...")
                    if s == "1":
                        self.robot.arm_control(ArmMovementParam.MOVING)
                        exit(0)
                    result = self.robot.vision()
                    print(result)
                    for e in result:
                        _, center_y = calculate_rectangle_center(e.box)
                        print(center_y)
                        match get_fruit_height(center_y):
                            case FruitHeight.TALL:
                                print("高水果")
                            case FruitHeight.MIDDLE:
                                print("中水果")
                            case FruitHeight.LOW:
                                print("低水果")
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = BModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
