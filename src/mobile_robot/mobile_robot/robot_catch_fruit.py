import time
import rclpy

from rclpy.node import Node

from .robot.robot import MobileRobot
from .robot.param.arm_movement import ArmMovementParam


class RobotCatchFruit(Node):
    def __init__(self):
        super().__init__('robot_catch_fruit')

        self.robot = MobileRobot(self)
        self.run()

    def run(self):
        self.robot.with_start_button()
        self.robot.arm_reset()

        self.robot.ping_revise(20, 0)



def main():
    rclpy.init()

    robot = RobotCatchFruit()

    rclpy.spin(robot)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()