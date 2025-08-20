import rclpy

from ..param import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.GrabAppleTree import GrabAppleTree
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class TestController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.node = node
        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)

    def run(self):
        self.robot.with_robot_connect()
        # self.arm.back_origin()
        #
        # ArmMovement.motion(self.arm)

        while True:
            print(self.robot.with_start_button())

        # dao = RobotDataDao(self.node)
        # while len(dao.imu_data) < 20:
        #     rclpy.spin_once(self.node)
        #     time.sleep(0.1)
        #
        # with open("/Users/starchen/Desktop/静态数据记录.txt", 'a') as f:
        #     for imu in dao.imu_data:
        #         f.write(str(imu) + "\n")

        # import pdb
        # pdb.set_trace()
        # ArmMovement.identify_tree_fruit(self.arm, Direction.RIGHT)
        # ArmMovement.grab_apple_on_tree(self.arm, Direction.RIGHT, -1, False)

        # 抓苹果树测试
        # tree = GrabAppleTree(self.node)
        # tree.direction = Direction.LEFT
        # tree.basket_1 = [FruitType.GREEN_APPLE, FruitType.GREEN_APPLE]
        # tree.basket_2 = [FruitType.RED_APPLE, FruitType.RED_APPLE]
        # tree.basket_3 = [FruitType.GREEN_APPLE, FruitType.RED_APPLE]
        #
        # while True:
        #     input("Press Enter to continue...")
        #     tree.close_tree()
