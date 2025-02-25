import rclpy
from rclpy.node import Node

from mobile_robot.mobile_robot.controller.GrabFruitController import GrabFruitController
from mobile_robot.mobile_robot.popo.FruitHeight import FruitHeight
from .controller.ArmController import ArmController
from .param.arm_movement import ArmMovementParam
from .param.navigation_path import NavPath, ORCHARD_1_POINT
from .popo.NavigationPoint import NavigationPoint
from .controller.MoveController import NavigationController


class BModule(Node):
    def __init__(self):
        super().__init__('b_module')

        self.__move = NavigationController(self)
        self.__arm = ArmController(self)
        self.__grab_fruit = GrabFruitController(self)

        select = int(input("等待按键按下, 1 - 15, 0 退出\n"))

        match select:
            case 0:
                exit(0)
            case 1:
                # 直线1米
                self.__move.init_pose(NavigationPoint(0, 0, 0))
                self.__move.navigation(NavPath.B_MODULE_1)
            case 2:
                # 旋转180度
                self.__move.rotate(180)
            case 3:
                # 抓水果
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_TALL, is_block=True)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END, is_block=True)
            case 4:
                # 果仓1到起始区
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.init_pose(NavigationPoint(0.62, -1.04, 90))
                self.__move.navigation(NavPath.B_MODULE_4)
            case 5:
                # 起始区到果仓1
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_5)
            case 6:
                # 采摘1到起始区
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.init_pose(ORCHARD_1_POINT)
                self.__move.navigation(NavPath.B_MODULE_6)
            case 7:
                # 起始区到采摘1
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_7)
            case 8:
                # 起始区到采摘1抓高水果
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_7)

                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_TALL)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END)
            case 9:
                # 起始区到采摘1抓中水果
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_7)

                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_MIDDLE)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END)
            case 10:
                # 起始区到采摘1抓低水果
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_7)

                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_LOW)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END, True)
            case 11:
                # 起始区到果仓一号放水果
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_11)

                self.__move.rotate(90)

                self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
                self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
                self.__arm.control(ArmMovementParam.MOVING)
            case 12:
                # 起始区到果园摘水果然后去果仓1号放水果
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_7)

                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_TALL)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END)

                self.__move.navigation(NavPath.B_MODULE_12)
                self.__move.rotate(90)

                self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
                self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
                self.__arm.control(ArmMovementParam.MOVING)
            case 13:
                # 起始区到果仓识别一个水果
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_5)
                self.__move.rotate(90)

                self.__arm.control(ArmMovementParam.RECOGNITION_WAREHOUSE)
                while True:
                    s = input("等待...")
                    if s == "1":
                        self.__arm.control(ArmMovementParam.MOVING)
                        exit(0)
                    print(self.__grab_fruit.vision())
            case 14:
                # 起始区到果园识别一个水果
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_7)

                self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT)

                while True:
                    s = input("等待...")
                    if s == "1":
                        self.__arm.control(ArmMovementParam.MOVING)
                        exit(0)
                    print(self.__grab_fruit.vision())
            case 15:
                # 起始区到果园识别一个水果的高低
                self.__arm.control(ArmMovementParam.RESET)
                self.__arm.control(ArmMovementParam.MOVING)

                self.__move.navigation(NavPath.B_MODULE_7)

                self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT)

                while True:
                    s = input("等待...")
                    if s == "1":
                        self.__arm.control(ArmMovementParam.MOVING)
                        exit(0)
                    result = self.__grab_fruit.vision()
                    for e in result:
                        match self.__grab_fruit.get_fruit_height(e.box):
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
