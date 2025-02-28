import rclpy
from rclpy.node import Node

from .param import NavigationPath
from .param.ArmMovement import ArmMovementParam
from .popo.FruitType import FruitType
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
        """
        执行整个任务流程，如前往一号走廊抓取水果并放置到果仓。
        """
        self.__arm.reset()
        self.__arm.control(ArmMovementParam.MOVING)

        # 前往一号走廊并初始化姿势
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_ENTER_1)
        self.__sensor.ping_revise(13.5)
        self.__move.init_pose(NavigationPoint(1.99, -3.65, -90))

        task = [[FruitType.RED_APPLE, FruitType.GREEN_APPLE], [FruitType.YELLOW_APPLE]]

        for index, warehouse in enumerate(task):
            for fruit in warehouse:
                self.get_logger().info(f"前往1号走廊抓取 {fruit.name}.")
                if self.__grub_fruit.patrol_the_line(NavigationPath.ORCHARD_CORRIDOR_EXIT_1_CORRECTIVE_POINT, fruit):
                    self.handle_fruit_grab(index)
                else:
                    self.get_logger().info(f"未寻找到 {fruit.name}, 前往二号走廊寻找.")
                    self.__move.navigation(NavigationPath.EXIT_1_TO_EXIT_2)
                    if self.__grub_fruit.patrol_the_line(NavigationPath.EXIT_2_TO_ENTER_2, fruit):
                        self.handle_fruit_grab(index)
                    else:
                        self.get_logger().error(f"仍未寻找到 {fruit.name}, 停止.")

    def handle_fruit_grab(self, index):
        """
        根据抓取的水果，控制机械臂将其放置到相应的果仓。
        :param index: 当前任务中的果仓编号
        """
        self.get_logger().info(f"抓取成功，前往 {index} 号果仓放置.")
        self.__move.navigation(NavigationPath.TO_WAREHOUSE_1_POINT)
        if index == 0:
            self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
        elif index == 1:
            self.__move.navigation(NavigationPath.WAREHOUSE_1_TO_WAREHOUSE_2)
            self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
        elif index == 2:
            self.__move.navigation(NavigationPath.WAREHOUSE_1_TO_WAREHOUSE_3)
            self.__arm.control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.__arm.control(ArmMovementParam.PULL_GUO_CANG)
        self.get_logger().info(f"放置完成, 前往一号走廊.")
        self.__move.navigation(NavigationPath.WAREHOUSE_TO_ORCHARD_ENTER_1)


def main():
    rclpy.init()

    node = CModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
