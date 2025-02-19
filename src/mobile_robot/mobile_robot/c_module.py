import rclpy
from rclpy.node import Node

from .robot.param.navigation_path import NavPath
from .robot.param.arm_movement import ArmMovementParam
from .robot.robot import MobileRobot


class CModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.robot = MobileRobot(self)

        input("按任意键开始游戏...")

        self.robot.arm_control(ArmMovementParam.RESET)
        self.robot.arm_control(ArmMovementParam.MOVING)

        self.robot.navigation(NavPath.B_MODULE_7)

        task = {1: ["Red Apple", "Red Apple"], 2: ["Red Apple"]}
        self.robot.grab_fruits(NavPath.ORCHARD_CORRIDOR_ENTER_1, task, "left")

        flag = False
        for i in task:
            if task[i]:
                flag = True

        if flag:
            self.robot.navigation(NavPath.EXIT_1_TO_EXIT_2)
            self.robot.grab_fruits(NavPath.EXIT_2_TO_ENTER_2, task, "right")
            self.robot.navigation(NavPath.ENTER_2_POINT_TO_WAREHOUSE_1_POINT)
        else:
            self.robot.navigation(NavPath.EXIT_1_POINT_TO_WAREHOUSE_1_POINT)


def main():
    rclpy.init()

    node = CModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
