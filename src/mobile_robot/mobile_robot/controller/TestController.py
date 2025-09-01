import rclpy

from ..param import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.GrabAppleTree import GrabAppleTree
from ..util.GrabGrapeWall import GrabGrapeWall
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
        self.arm.back_origin()

        # while True:
        #     input("等待...")
        #     self.sensor.correction("c_6")


        # # 抓葡萄墙测试
        # wall = GrabGrapeWall(self.node)
        # wall.direction = Direction.LEFT
        # while True:
        #     input("Press Enter to continue...")
        #     ArmMovement.identify_grape(self.arm, Direction.LEFT)
        #     fruit = self.vision.find_fruit([FruitType.PURPLE_GRAPE], True, kernel_size=71)
        #     wall.grab_grape(fruit)


        # # 矫正雷达
        # while True:
        #     input("等待")
        #     print("----------------左墙----------------")
        #     li = []
        #     for i in range(5):
        #         wall = self.sensor.get_angle_from_wall(Direction.LEFT)
        #         li.append(wall)
        #         print(wall)
        #     print(Math.average_without_extremes(li))
        #
        #     print("----------------前墙----------------")
        #     li = []
        #     for i in range(5):
        #         wall = self.sensor.get_angle_from_wall(Direction.FRONT)
        #         li.append(wall)
        #         print(wall)
        #     print(Math.average_without_extremes(li))
        #
        #     print("----------------右墙----------------")
        #     li = []
        #     for i in range(5):
        #         wall = self.sensor.get_angle_from_wall(Direction.RIGHT)
        #         li.append(wall)
        #         print(wall)
        #     print(Math.average_without_extremes(li))

        # dao = RobotDataDao(self.node)
        # while len(dao.imu_data) < 20:
        #     rclpy.spin_once(self.node)
        #     time.sleep(0.1)
        #
        # with open("/Users/starchen/Desktop/静态数据记录.txt", 'a') as f:
        #     for imu in dao.imu_data:
        #         f.write(str(imu) + "\n")

        # ArmMovement.identify_tree_fruit(self.arm, Direction.RIGHT)
        # while True:
        #     photograph = self.vision.photograph()
        #     self.vision.show_photo(photograph, True)
        #     input("等待...")

        # 抓苹果树测试
        tree = GrabAppleTree(self.node)
        tree.direction = Direction.LEFT
        tree.basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        tree.basket_2 = [FruitType.GREEN_APPLE, FruitType.YELLOW_APPLE]
        tree.basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_APPLE]

        while True:
            input("Press Enter to continue...")
            tree.grab_apple_from_tree()
