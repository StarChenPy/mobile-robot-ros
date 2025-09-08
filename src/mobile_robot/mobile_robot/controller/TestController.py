import time

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
from ..util import Math
from ..util.GrabAppleTree import GrabAppleTree
from ..util.GrabGrapeWall import GrabGrapeWall
from ..util.GrabGroundFruit import GrabGroundFruit
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

        # while True:
            # input("等待...")
            # self.sensor.init_odom_all(NavigationPoint(2.405, 1.55, 180))
            # self.move.my_navigation("s_y_2_r")
            # self.sensor.correction("c_2")

        # 矫正雷达
        while True:
            input("等待")
            print("----------------左墙----------------")
            li = []
            for i in range(10):
                wall = self.sensor.get_angle_from_wall(Direction.LEFT)
                li.append(wall)
                print(wall)
                time.sleep(0.3)
            print(Math.average_without_extremes(li))

            print("----------------前墙----------------")
            li = []
            for i in range(10):
                wall = self.sensor.get_angle_from_wall(Direction.FRONT)
                li.append(wall)
                print(wall)
                time.sleep(0.3)
            print(Math.average_without_extremes(li))

            print("----------------右墙----------------")
            li = []
            for i in range(10):
                wall = self.sensor.get_angle_from_wall(Direction.RIGHT)
                li.append(wall)
                print(wall)
                time.sleep(0.3)
            print(Math.average_without_extremes(li))

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


        # # 抓苹果树测试
        # tree = GrabAppleTree(self.node)
        # tree.direction = Direction.RIGHT
        # tree.basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        # tree.basket_2 = [FruitType.GREEN_APPLE, FruitType.YELLOW_APPLE]
        # tree.basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_APPLE]
        #
        # while True:
        #     input("Press Enter to continue...")
        #     tree.grab_apple_from_tree()


        # # 抓葡萄墙测试
        # wall = GrabGrapeWall(self.node)
        # wall.direction = Direction.LEFT
        # wall.basket_1 = [FruitType.GREEN_GRAPE] * 3
        # wall.basket_2 = [FruitType.YELLOW_GRAPE] * 3
        # wall.basket_3 = [FruitType.PURPLE_GRAPE] * 3
        # while True:
        #     input("Press Enter to continue...")
        #     wall.grab_grape_from_wall()


        # # 抓地板水果测试
        # ground_fruit = GrabGroundFruit(self.node)
        # print(ground_fruit.scan_corridor())
