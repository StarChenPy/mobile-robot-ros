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

        self.run()

    def run(self):
        self.robot.arm_control(ArmMovementParam.RESET)
        self.robot.arm_control(ArmMovementParam.MOVING)

        self.robot.navigation(NavPath.B_MODULE_7)

        task = {1: ["Red Apple", "Red Apple"], 2: ["Red Apple"], 3: []}
        taskCount = {1: 2, 2: 1, 3: 0}
        flag = self.robot.grab_fruits(NavPath.ORCHARD_CORRIDOR_ENTER_1, task, "left")

        if flag:
            self.robot.navigation(NavPath.EXIT_1_POINT_TO_WAREHOUSE_1_POINT)
            self.push(taskCount)
        else:
            self.robot.navigation(NavPath.EXIT_1_TO_EXIT_2)
            if self.robot.grab_fruits(NavPath.EXIT_2_TO_ENTER_2, task, "right"):
                return
            self.robot.navigation(NavPath.ENTER_2_POINT_TO_WAREHOUSE_1_POINT)
            self.push(taskCount)

    def push(self, taskCount):
        if taskCount[1] != 0:
            taskCount[1] -= 1
            self.robot.rotate(90)

            self.robot.arm_control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.MOVING)

            self.robot.navigation(NavPath.WAREHOUSE_TO_ORCHARD_ENTER_1)
        elif taskCount[2] != 0:
            taskCount[2] -= 1
            self.robot.navigation(NavPath.WAREHOUSE_1_TO_WAREHOUSE_2)

            self.robot.rotate(90)

            self.robot.arm_control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.MOVING)
        elif taskCount[3] != 0:
            taskCount[3] -= 1
            self.robot.navigation(NavPath.WAREHOUSE_1_TO_WAREHOUSE_2)

            self.robot.rotate(90)

            self.robot.arm_control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.MOVING)


def main():
    rclpy.init()

    node = CModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
