import rclpy
from rclpy.node import Node

from .controller.GrabFruitController import GrabFruitController
from .popo.Direction import Direction
from .param import ArmMovement
from .controller.RobotController import RobotController
from .controller.MoveController import MoveController
from .controller.ArmController import ArmController


class TestModule(Node):
    def __init__(self):
        super().__init__("c_module")

        self.__arm = ArmController(self)
        self.__move = MoveController(self)
        self.__robot = RobotController(self)
        self.__grab = GrabFruitController(self)

        self.__robot.with_robot_connect()

        input("按任意键开始游戏")

        # self.__arm.reset()

        ArmMovement.recognition_orchard(self.__arm, Direction.RIGHT)

        for _ in range(3):
            input("等待")
            for i in self.__grab.vision():
                print(i.box.get_area())

        ArmMovement.recognition_orchard_end(self.__arm, Direction.RIGHT)

        self.destroy_node()
        rclpy.shutdown()
        exit(0)


def main():
    rclpy.init()

    node = TestModule()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
