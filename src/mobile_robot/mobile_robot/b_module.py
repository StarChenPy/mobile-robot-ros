import rclpy
from rclpy.node import Node

from .robot.robot import MobileRobot
from .robot.param.navigation_path import NavPath
from .robot.param.arm_movement import ArmMovementParam


class BModule(Node):
    def __init__(self):
        super().__init__('b_module')

        self.robot = MobileRobot(self)

        while True:
            select = int(input("等待按键按下, 1 - 15, 0 退出\n"))

            match select:
                case 0:
                    self.robot.ping_revise(30, 0)
                case 1:
                    self.robot.navigation(NavPath.B_MODULE_1)
                case 2:
                    self.robot.rotate(90)
                case 3:
                    self.robot.arm_control(ArmMovementParam.RESET)
                    self.robot.arm_control(ArmMovementParam.READY_GRAB_APPLE)
                    self.robot.arm_control(ArmMovementParam.GRAB_APPLE)
                    self.robot.arm_control(ArmMovementParam.GRAB_APPLE_END)
                case 4:
                    self.robot.arm_control(ArmMovementParam.RESET)
                    self.robot.arm_control(ArmMovementParam.MOVING)
                    self.robot.ping_revise(30, 0)
                    self.robot.navigation(NavPath.B_MODULE_4)

                case 5:
                    self.robot.arm_control(ArmMovementParam.RESET)
                    self.robot.arm_control(ArmMovementParam.MOVING)


def main():
    rclpy.init()

    node = BModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
