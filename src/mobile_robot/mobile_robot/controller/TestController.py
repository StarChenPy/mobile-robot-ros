import time

import rclpy

from ..param import ArmMovement as Movement
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.FruitType import FruitType
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math, Util
from ..util.Logger import Logger
from ..util.NavigationPointParam import NavigationPointParam
from ..util.Singleton import singleton


@singleton
class TestController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__vision = VisionService(node)
        self.__arm = ArmService(node)
        self.__sensor = SensorService(node)
        self.__robot = RobotService(node)
        self.__move = MoveService(node)

        self.__param = NavigationPointParam("shandong_trials_navigation_point.yml")

    def rotation_correction(self):
        angle_by_front = self.__sensor.get_angle_from_wall(Direction.FRONT)
        angle_by_right = self.__sensor.get_angle_from_wall(Direction.RIGHT)
        angle_by_left = self.__sensor.get_angle_from_wall(Direction.LEFT)
        angles = [angle_by_front, angle_by_right, angle_by_left]
        angles = [x for x in angles if x != 0]

        self.__logger.info(f"直角矫正角度: {angles}")

        min_angle = min(angles, key=lambda x: (abs(x), -x))
        if abs(min_angle) < 15:
            self.__move.rotate(min_angle)

    def run(self):
        self.__sensor.initial_pose(NavigationPoint(3.7, 3.7, 180))
        self.__arm.back_origin()
        self.__arm.control(Movement.MOVING)
        while True:
            i = input("输入小题编号：")
            match i:
                case "1": self.b1()
                case "2": self.b2()
                case "3": self.b3()
                case "4": self.b4()
                case "5": self.b5()
                case "6": self.b6()
                case "7": self.b7()
                case "8": self.b8()
                case "9": self.b9()
                case "10": self.b10()
                case "11": self.fruit_location_test()
                case "12": Movement.grab_basket_to_warehouse(self.__arm, 2)
                case _:
                    print("未知小题")
                    continue

    # 篮子放黄站台，抓篮子放红站台，回起始区
    def b1(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("t")
        self.rotation_correction()
        Movement.grab_basket_from_station(self.__arm)
        Movement.put_basket_to_robot(self.__arm, 1)
        self.__move.my_navigation("l")
        self.rotation_correction()
        Movement.grab_basket_to_station(self.__arm, 1)
        self.__move.my_navigation("a")
        print("测试完成！")

    # 去台阶另一端，然后回起始区
    def b2(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("f")
        time.sleep(3)
        self.__move.my_navigation("a")
        print("测试完成！")

    # 去抓果树1上的固定位置的苹果，将其放到仓库A
    def b3(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("i")
        Movement.grab_apple_on_tree(self.__arm, FruitLocationOnTree.TOP_CENTER)
        self.__move.my_navigation("u")
        Movement.put_fruit_into_warehouse(self.__arm, 4)
        print("测试完成！")

    # 去抓香蕉树上固定位置的香蕉，将其放到仓库B
    def b4(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("d")
        Movement.grab_banana_on_tree(self.__arm, FruitLocationOnTree.BOTTOM_LEFT)
        self.__arm.control(Movement.MOVING_BANANA)
        self.__move.my_navigation("w")
        Movement.put_fruit_into_warehouse(self.__arm, 1.5)
        print("测试完成！")

    # 去抓葡萄行中固定位置的葡萄，将其放到仓库A
    def b5(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("q")
        self.rotation_correction()
        Movement.grab_grape(self.__arm)
        self.__arm.control(Movement.MOVING_GRAPE)
        self.__move.my_navigation("u")
        Movement.put_fruit_into_warehouse(self.__arm, 2)
        print("测试完成！")

    # 篮子放黄站台，去拿篮子，然后去香蕉树上摘一个随机放置的香蕉放到篮子里，然后将篮子放到红站台
    def b6(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("t")
        self.rotation_correction()
        Movement.grab_basket_from_station(self.__arm)
        Movement.put_basket_to_robot(self.__arm, 1)
        self.__move.my_navigation("c")
        Movement.recognition_orchard_banana_tree(self.__arm)
        pass  # 这里是识别
        location = FruitLocationOnTree.TOP_CENTER
        self.__move.my_navigation("d")
        Movement.grab_banana_on_tree(self.__arm, location)
        Movement.put_fruit_into_basket(self.__arm, 1, 1.5)
        self.__move.my_navigation("l")
        self.rotation_correction()
        Movement.grab_basket_to_station(self.__arm, 1)
        self.__move.my_navigation("a")

        print("测试完成！")

    # 两个篮子分别放在红黄站台，去拿篮子，然后去果树1与果树2抓一个随机放置的红/黄苹果分开放到篮子中，然后将红苹果篮子放到A仓库，黄苹果篮子放到B仓库
    def b7(self):
        input("按回车键开始导航...")
        # # 抓篮子1
        # self.__move.my_navigation("l")
        # self.rotation_correction()
        # Movement.grab_basket_from_station(self.__arm)
        # Movement.put_basket_to_robot(self.__arm, 1)
        # self.__arm.control(Movement.MOVING)
        #
        # # 抓篮子2
        # self.__move.my_navigation("t")
        # self.rotation_correction()
        # Movement.grab_basket_from_station(self.__arm)
        # Movement.put_basket_to_robot(self.__arm, 2)
        # self.__arm.control(Movement.MOVING)

        # 前往苹果树1
        self.__move.my_navigation("h")
        self.__sensor.ping_revise(8)
        Movement.recognition_orchard_apple_tree_1(self.__arm)
        identify = self.__vision.get_onnx_identify()
        fruit_location: dict = Util.get_fruit_location(identify, [FruitType.RED_APPLE])
        if fruit_location.keys():
            Movement.end_recognition_orchard_apple_tree_1(self.__arm)
            self.__move.my_navigation("i")
            self.__sensor.ping_revise(8)
            for i in fruit_location.keys():
                Movement.grab_apple_on_tree(self.__arm, i)
                Movement.put_fruit_into_basket(self.__arm, 1, 4)
        self.__arm.control(Movement.MOVING)

        # 前往苹果树2
        self.__move.my_navigation("j")
        self.rotation_correction()
        Movement.recognition_orchard_apple_tree_2(self.__arm)
        identify = self.__vision.get_onnx_identify()
        fruit_location = Util.get_fruit_location(identify, [FruitType.YELLOW_APPLE])
        if fruit_location.keys():
            Movement.end_recognition_orchard_apple_tree_2(self.__arm)
            self.__move.my_navigation("k")
            self.rotation_correction()
            for i in fruit_location.keys():
                Movement.grab_apple_on_tree(self.__arm, i)
                Movement.put_fruit_into_basket(self.__arm, 1, 4)
        self.__arm.control(Movement.MOVING)

        self.__move.my_navigation("u")
        Movement.grab_basket_to_warehouse(self.__arm, 1)
        self.__move.my_navigation("w")
        Movement.grab_basket_to_warehouse(self.__arm, 2)

    # 去识别区2识别一个随机放置的果蔬，然后去果蔬区抓取对应果蔬并放到仓库A
    def b8(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("x")
        Movement.recognition_orchard_apple_tree_1(self.__arm)
        fruit = self.__vision.get_onnx_identify()[0].classId

        # 寻找水果
        self.__move.my_navigation("g")
        Movement.recognition_orchard_grape(self.__arm)
        self.__move.my_navigation("f")
        identify = self.__vision.get_onnx_identify()
        if identify[0].classId == fruit:
            self.__move.stop_my_navigation()
            Movement.grab_grape(self.__arm)
        self.__arm.control(Movement.MOVING)

        # 放到仓库
        self.__move.my_navigation("w")
        Movement.put_fruit_into_warehouse(self.__arm, 4)

        self.__move.my_navigation("a")
        print("测试完成！")

    # 去黄站台拿篮子，然后去识别区1识别订单板，再去抓对应目标放到篮子中，将篮子放到红站台，并回到起始区
    def b9(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("x")
        Movement.recognition_orchard_apple_tree_1(self.__arm)

    # 将A仓库和B仓库中的苹果互换位置，并返回起始区
    def b10(self):
        pass

    def fruit_location_test(self):
        Movement.recognition_orchard_apple_tree_1(self.__arm)
        while True:
            identify = self.__vision.get_onnx_identify()
            fruit_location = Util.get_fruit_location(identify, [FruitType.RED_APPLE])
            print(fruit_location)

    def create_point(self):
        s = input("是否已有点？y/n: ")
        point_name = None
        if s == "y":
            point_name = input("请输入矫正点名称：")
            point = self.__param.get_navigation_point(point_name)
        else:
            x, y, yaw = input("输入当前坐标(x y yaw): ").split(" ")
            point = NavigationPoint(float(x), float(y), float(yaw))

        dirs = input("输入要矫正的方向 f:前 b:后 l:左 r:右 :").split(" ")

        corrective_data = []
        for direction in dirs:
            if direction == "f":
                from_wall = self.__sensor.get_distance_from_wall(Direction.FRONT)
                corrective_data.append(Corrective(Direction.FRONT, round(from_wall, 3)))
            elif direction == "b":
                sonar = self.__sensor.get_sonar()
                from_wall = Math.distance_from_origin(-5, sonar[0], 5, sonar[1]) + 0.222
                corrective_data.append(Corrective(Direction.BACK, round(from_wall, 3)))
            elif direction == "l":
                from_wall = self.__sensor.get_distance_from_wall(Direction.LEFT)
                corrective_data.append(Corrective(Direction.LEFT, round(from_wall, 3)))
            elif direction == "r":
                from_wall = self.__sensor.get_distance_from_wall(Direction.RIGHT)
                print(type(from_wall))
                corrective_data.append(Corrective(Direction.RIGHT, round(from_wall, 3)))

        corrective_point = CorrectivePoint(point.x, point.y, point.yaw, corrective_data)

        print(f"已生成矫正点数据: {corrective_point}")
        if "y" == input("是否保存？y/n: "):
            if point_name is None:
                point_name = input("输入矫正点名称: ")
            self.__param.set_navigation_point(point_name, corrective_point)
