import rclpy
from rclpy.node import Node

from .robot.param.navigation_path import NavPath, ORCHARD_CORRIDOR_ENTER_1_POINT
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

        # self.robot.navigation(NavPath.START_TO_ORCHARD_ENTER_1)
        self.robot.ping_revise(14)
        self.robot.init_pose(ORCHARD_CORRIDOR_ENTER_1_POINT)

        task = {1: ["Red Apple", "Red Apple"], 2: ["Red Apple"], 3: []}
        task_count = {1: 2, 2: 1, 3: 0}

        flag = self.robot.grab_fruits(NavPath.ORCHARD_CORRIDOR_ENTER_1, task, "right")

        # for i in range(4):
        #     flag = self.robot.grab_fruits(NavPath.ORCHARD_CORRIDOR_ENTER_1, task, "right")
        #
        #     if flag:
        #         print("它找到了水果，准备回去放")
        #         self.robot.navigation(NavPath.TO_WAREHOUSE_1_POINT)
        #         self.push(task_count)
        #     else:
        #         print("它没找到水果，准备去走廊2找")
        #         self.robot.navigation(NavPath.EXIT_1_TO_EXIT_2)
        #         if self.robot.grab_fruits(NavPath.EXIT_2_TO_ENTER_2, task, "left"):
        #             return
        #         self.robot.navigation(NavPath.ENTER_2_POINT_TO_WAREHOUSE_1_POINT)
        #         self.push(task_count)

    def push(self, task_count):
        if task_count[1] != 0:
            task_count[1] -= 1
            self.robot.rotate(90)

            self.robot.arm_control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.MOVING)

            self.robot.navigation(NavPath.WAREHOUSE_TO_ORCHARD_ENTER_1)
        elif task_count[2] != 0:
            task_count[2] -= 1
            self.robot.navigation(NavPath.WAREHOUSE_1_TO_WAREHOUSE_2)

            self.robot.rotate(90)

            self.robot.arm_control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.MOVING)

            self.robot.navigation(NavPath.WAREHOUSE_TO_ORCHARD_ENTER_1)
        elif task_count[3] != 0:
            task_count[3] -= 1
            self.robot.navigation(NavPath.WAREHOUSE_1_TO_WAREHOUSE_3)

            self.robot.rotate(90)

            self.robot.arm_control(ArmMovementParam.READY_PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.PULL_GUO_CANG)
            self.robot.arm_control(ArmMovementParam.MOVING)

            self.robot.navigation(NavPath.WAREHOUSE_TO_ORCHARD_ENTER_1)


def main():
    rclpy.init()

    node = CModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
