import rclpy
from rclpy.node import Node

from .popo.FruitHeight import FruitHeight
from .controller.RobotController import RobotController
from .param import NavigationPath
from .param.ArmMovement import ArmMovementParam
from .popo.FruitType import FruitType
from .controller.MoveController import MoveController
from .controller.GrabFruitController import GrabFruitController
from .controller.ArmController import ArmController


class CModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.__arm = ArmController(self)
        self.__grub_fruit = GrabFruitController(self)
        self.__move = MoveController(self)
        self.__robot = RobotController(self)

        self.__robot.with_robot_connect()

        s = input("已知y，未知w，选择：")

        self.__arm.reset()
        self.__arm.control(ArmMovementParam.MOVING)

        self.__robot.set_start_led(False)
        self.__robot.with_start_button()
        if s == "y":
            self._run_y()
        elif s == "w":
            self._run_w()

        self.__robot.set_start_led(False)

        exit(0)

    def grab_and_store(self, orchard_path, warehouse_path):
        """通用的抓取水果并存储的函数"""
        self.__move.navigation(orchard_path)
        self.__grub_fruit.execute_grab_sequence(FruitHeight.TALL, False)
        self.__move.navigation(warehouse_path)
        self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
        self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
        self.__arm.control(ArmMovementParam.MOVING)

    def _run_y(self):
        tasks = [
            (NavigationPath.START_TO_ORCHARD_1, NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT),
            (NavigationPath.WAREHOUSE_TO_ORCHARD_2, NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT),
            (NavigationPath.WAREHOUSE_TO_ORCHARD_3, NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT),
            (NavigationPath.WAREHOUSE_TO_ORCHARD_4, NavigationPath.ORCHARD_CORRIDOR_2_TO_WAREHOUSE_1_POINT),
            (NavigationPath.WAREHOUSE_TO_ORCHARD_5, NavigationPath.ORCHARD_CORRIDOR_2_TO_WAREHOUSE_1_POINT),
        ]

        for orchard, warehouse in tasks:
            self.grab_and_store(orchard, warehouse)
        self.__move.navigation(NavigationPath.B_MODULE_4)

    def _run_w(self):
        """
        执行整个任务流程，如前往一号走廊抓取水果并放置到果仓
        """
        # 前往一号走廊并初始化姿势
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_ENTER_1)

        task = [[FruitType.RED_APPLE, FruitType.RED_APPLE, FruitType.RED_APPLE, FruitType.RED_APPLE, FruitType.RED_APPLE]]

        for index, warehouse in enumerate(task):
            for fruit in warehouse:
                self.get_logger().info(f"[Module C] 前往1号走廊抓取 {fruit.name}.")
                if self.__grub_fruit.patrol_the_line(NavigationPath.ORCHARD_CORRIDOR_EXIT_1_POINT, fruit):
                    self.handle_fruit_grab(index)
                else:
                    self.get_logger().info(f"[Module C] 未寻找到 {fruit.name}, 前往二号走廊寻找.")
                    self.__arm.control(ArmMovementParam.MOVING)
                    self.__move.navigation(NavigationPath.EXIT_1_TO_EXIT_2)
                    if self.__grub_fruit.patrol_the_line(NavigationPath.ORCHARD_CORRIDOR_ENTER_2_POINT, fruit, True):
                        self.handle_fruit_grab(index)
                    else:
                        self.get_logger().error(f"[Module C] 仍未寻找到 {fruit.name}, 停止.")
                        self.__arm.control(ArmMovementParam.MOVING)
                        self.__move.navigation(NavigationPath.B_MODULE_6)
                        return

        self.__move.navigation(NavigationPath.B_MODULE_6)

    def handle_fruit_grab(self, index):
        """
        根据抓取的水果，控制机械臂将其放置到相应的果仓
        :param index: 当前任务中的果仓编号
        """
        self.get_logger().info(f"[Module C] 已抓取，前往 {index} 号果仓放置.")

        self.__move.navigation(NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT)

        if index == 1:
            self.__move.navigation(NavigationPath.WAREHOUSE_1_TO_WAREHOUSE_2)
        elif index == 2:
            self.__move.navigation(NavigationPath.WAREHOUSE_1_TO_WAREHOUSE_3)

        self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
        self.__arm.control(ArmMovementParam.PULL_GUO_CANG)

        self.__arm.control(ArmMovementParam.MOVING)
        self.get_logger().info("[Module C] 放置完成, 前往果园一号走廊.")
        self.__move.navigation(NavigationPath.WAREHOUSE_TO_ORCHARD_ENTER_1)


def main():
    rclpy.init()

    node = CModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
