import rclpy
from rclpy.node import Node

from .param import NavigationPath
from .param.ArmMovement import ArmMovementParam
from .popo.NavigationPoint import NavigationPoint
from .controller.MoveController import MoveController
from .controller.SensorController import SensorController
from .controller.GrabFruitController import GrabFruitController
from .controller.ArmController import ArmController


class CModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.__arm = ArmController()
        self.__grub_fruit = GrabFruitController()
        self.__sensor = SensorController()
        self.__move = MoveController()

        input("按任意键开始游戏...")

        self.run()

    def run(self):
        self.__arm.reset()
        self.__arm.control(ArmMovementParam.MOVING)

        # self.robot.navigation(NavPath.START_TO_ORCHARD_ENTER_1)
        self.__sensor.ping_revise(14)
        self.__move.init_pose(NavigationPoint(1.99, -3.65, 90))

        task = {1: ["Red Apple", "Red Apple"], 2: ["Red Apple"], 3: []}
        task_count = {1: 2, 2: 1, 3: 0}

        flag = self.__grub_fruit.grab_fruits(NavigationPath.ORCHARD_CORRIDOR_ENTER_1, task, "right")

        for i in range(4):
            flag = self.__grub_fruit.grab_fruits(NavigationPath.ORCHARD_CORRIDOR_ENTER_1, task, "right")

            if flag:
                print("它找到了水果，准备回去放")
                self.__move.navigation(NavigationPath.TO_WAREHOUSE_1_POINT)
                self.push(task_count)
            else:
                print("它没找到水果，准备去走廊2找")
                self.__move.navigation(NavigationPath.EXIT_1_TO_EXIT_2)
                if self.__grub_fruit.grab_fruits(NavigationPath.EXIT_2_TO_ENTER_2, task, "left"):
                    return
                self.__move.navigation(NavigationPath.ENTER_2_POINT_TO_WAREHOUSE_1_POINT)
                self.push(task_count)

    def push(self, task_count):
        if task_count[1] != 0:
            task_count[1] -= 1
            self.__move.rotate(90)

            self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.MOVING)

            self.__move.navigation(NavigationPath.WAREHOUSE_TO_ORCHARD_ENTER_1)
        elif task_count[2] != 0:
            task_count[2] -= 1
            self.__move.navigation(NavigationPath.WAREHOUSE_1_TO_WAREHOUSE_2)

            self.__move.rotate(90)

            self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.MOVING)

            self.__move.navigation(NavigationPath.WAREHOUSE_TO_ORCHARD_ENTER_1)
        elif task_count[3] != 0:
            task_count[3] -= 1
            self.__move.navigation(NavigationPath.WAREHOUSE_1_TO_WAREHOUSE_3)

            self.__move.rotate(90)

            self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.MOVING)

            self.__move.navigation(NavigationPath.WAREHOUSE_TO_ORCHARD_ENTER_1)


def main():
    rclpy.init()

    node = CModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
