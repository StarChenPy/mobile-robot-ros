import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
from rclpy.node import Node

from .controller.RobotController import RobotController
from .param import NavigationPath
from .popo.FruitType import FruitType
from .controller.CModuleController import CModuleController


class CModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.get_logger().set_level(LoggingSeverity.INFO)

        self.__grub_fruit = CModuleController(self)
        self.__robot = RobotController(self)

        self.__robot.with_robot_connect()

        s = input("已知y，未知w，选择：")

        self.__robot.reset_arm()

        self.__robot.set_start_led(False)
        self.__robot.with_start_button()

        if s == "y":
            task = [
                (NavigationPath.START_TO_ORCHARD_1, NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT),
                (NavigationPath.WAREHOUSE_TO_ORCHARD_2, NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT),
                (NavigationPath.WAREHOUSE_TO_ORCHARD_3, NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT),
                (NavigationPath.WAREHOUSE_TO_ORCHARD_4, NavigationPath.ORCHARD_CORRIDOR_2_TO_WAREHOUSE_1_POINT),
                (NavigationPath.WAREHOUSE_TO_ORCHARD_5, NavigationPath.ORCHARD_CORRIDOR_2_TO_WAREHOUSE_1_POINT),
            ]
            self.__grub_fruit.known_fruit_grab_task(task)
        elif s == "w":
            task = {1: FruitType.RED_APPLE, 2: FruitType.YELLOW_APPLE, 3: FruitType.GREEN_APPLE}
            self.__grub_fruit.unknown_fruit_grab_task(task)

        self.__robot.set_start_led(False)

        exit(0)


def main():
    rclpy.init()

    node = CModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
