import time

import rclpy
from rclpy.node import Node

from .controller.RobotController import RobotController
from .controller.ArmController import ArmController
from .controller.GrabFruitController import GrabFruitController, get_fruit_height
from .controller.MoveController import MoveController
from .popo.NavigationPoint import NavigationPoint
from .popo.FruitHeight import FruitHeight
from .param.ArmMovement import ArmMovementParam
from .param import NavigationPath


class BModule(Node):
    def __init__(self):
        super().__init__('b_module')

        self.__move = MoveController(self)
        self.__arm = ArmController(self)
        self.__grab_fruit = GrabFruitController(self)
        self.__robot = RobotController(self)

        self.__robot.with_robot_connect()

        select = int(input("从1 - 15开始, 0 退出, 你的选择是: "))
        self.__robot.set_start_led(False)

        self.__arm.reset()
        self.__arm.control(ArmMovementParam.MOVING)

        for i in range(select, 16):

            self.__robot.with_start_button()

            self.__move.reset_odom()
            self.task(i)

            self.__robot.set_start_led(False)
            time.sleep(1)
            rclpy.spin_once(self)

        rclpy.shutdown()
        exit(0)

    def task(self, select: int):
        match select:
            case 0:
                exit(0)
            case 1:
                # 直线1米
                self.__move.init_pose(NavigationPoint(0, 0, 0))
                self.__move.navigation([NavigationPoint(1, 0, 0)])
            case 2:
                # 旋转360度
                self.__move.rotate(361)
            case 3:
                # 抓水果
                self.__grab_fruit.execute_grab_sequence(FruitHeight.TALL, False)
            case 4:
                # 果仓1到起始区
                self.__move.init_pose(NavigationPoint(0.56, -1.12, 90))
                self.__move.navigation(NavigationPath.B_MODULE_4)
            case 5:
                # 起始区到果仓1
                self.__move.navigation(NavigationPath.B_MODULE_5)
            case 6:
                # 采摘1到起始区
                self.__move.init_pose(NavigationPath.ORCHARD_1_POINT)
                self.__move.navigation(NavigationPath.B_MODULE_6)
            case 7:
                # 起始区到采摘1
                self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)
            case 8:
                # 起始区到采摘1抓高水果
                self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE_TALL)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_TALL)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END)
            case 9:
                # 起始区到采摘1抓中水果
                self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE_MIDDLE)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_MIDDLE)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_MIDDLE_END)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END)
            case 10:
                # 起始区到采摘1抓低水果
                self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE_LOW)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_LOW)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_LOW_END)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END)
            case 11:
                # 起始区到果仓一号放水果
                self.__move.navigation(NavigationPath.B_MODULE_11)

                self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
                self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
                self.__arm.control(ArmMovementParam.MOVING)
            case 12:
                # 起始区到果园摘水果然后去果仓1号放水果
                self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE)
                self.__arm.control(ArmMovementParam.READY_GRAB_APPLE_TALL)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_TALL)
                self.__arm.control(ArmMovementParam.GRAB_APPLE_END)

                self.__move.navigation(NavigationPath.B_MODULE_12)

                self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
                self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
                self.__arm.control(ArmMovementParam.MOVING)
            case 13:
                # 起始区到果仓识别一个水果
                self.__move.navigation(NavigationPath.B_MODULE_5)

                self.__arm.control(ArmMovementParam.RECOGNITION_WAREHOUSE)
                for _ in range(10):
                    print(self.__grab_fruit.vision())
                self.__arm.control(ArmMovementParam.MOVING)
            case 14:
                # 起始区到果园识别一个水果
                self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

                self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT)

                for _ in range(10):
                    print(self.__grab_fruit.vision())
                self.__arm.control(ArmMovementParam.MOVING)
            case 15:
                # 起始区到果园识别一个水果的高低
                self.__move.navigation(NavigationPath.START_TO_ORCHARD_1)

                self.__arm.control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT)

                for _ in range(10):
                    result = self.__grab_fruit.vision()
                    for e in result:
                        match get_fruit_height(e.box):
                            case FruitHeight.TALL:
                                print("高水果")
                            case FruitHeight.MIDDLE:
                                print("中水果")
                            case FruitHeight.LOW:
                                print("低水果")
                self.__arm.control(ArmMovementParam.MOVING)



def main():
    rclpy.init()

    node = BModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
