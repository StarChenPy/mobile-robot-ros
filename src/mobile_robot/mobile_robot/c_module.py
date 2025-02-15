import rclpy
from rclpy.node import Node

from .robot.param.navigation_path import NavPath
from .robot.param.arm_movement import ArmMovementParam
from .robot.robot import MobileRobot


class CModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.robot = MobileRobot(self)

        self.robot.arm_control(ArmMovementParam.RESET)
        self.robot.arm_control(ArmMovementParam.MOVING)

        self.robot.navigation(NavPath.B_MODULE_7)

        self.robot.grab_fruits(NavPath.ORCHARD_CORRIDOR_ENTER_1, ["Red Apple"])


def main():
    rclpy.init()

    node = CModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
