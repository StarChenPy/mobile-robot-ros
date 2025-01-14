import rclpy

from rclpy.node import Node

from .robot.robot import MobileRobot


class RobotCatchFruit(Node):
    def __init__(self):
        super().__init__('robot_catch_fruit')

        self.robot = MobileRobot(self)
        self.run()

    def run(self):
        self.robot.with_start_button()
        self.robot.arm_reset()



def main():
    rclpy.init()

    robot = RobotCatchFruit()

    rclpy.spin(robot)

    # 这个调用应该放在节点内部
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()