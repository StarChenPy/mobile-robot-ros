import time

import rclpy

from ..param import ArmMovement as Movement
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
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
        self.__arm.back_origin()
        self.__sensor.initial_pose(NavigationPoint(3.7, 3.7, 180))
        self.__arm.control(Movement.MOVING)
        self.b6()

    def b1(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("r")
        self.rotation_correction()
        Movement.grab_basket_from_station(self.__arm)
        Movement.put_basket_to_robot(self.__arm, 1)
        self.__move.my_navigation("j")
        self.rotation_correction()
        Movement.grab_basket_to_station(self.__arm, 1)
        self.__move.my_navigation("a")
        print("测试完成！")

    def b2(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("f")
        time.sleep(3)
        self.__move.my_navigation("a")
        print("测试完成！")

    def b3(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("n")
        Movement.grab_apple_on_tree(self.__arm, FruitLocationOnTree.TOP_CENTER)
        self.__arm.control(Movement.MOVING)
        self.__move.my_navigation("s")
        Movement.put_fruit_into_warehouse(self.__arm, 4)
        self.__move.my_navigation("a")
        print("测试完成！")

    def b4(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("c")
        Movement.grab_banana_on_tree(self.__arm, FruitLocationOnTree.BOTTOM_LEFT)
        self.__arm.control(Movement.MOVING_BANANA)
        self.__move.my_navigation("v")
        Movement.put_fruit_into_warehouse(self.__arm, 1.5)
        self.__move.my_navigation("a")
        print("测试完成！")

    def b5(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("x")
        self.rotation_correction()
        Movement.grab_grape(self.__arm)
        self.__arm.control(Movement.MOVING_GRAPE)
        self.__move.my_navigation("s")
        Movement.put_fruit_into_warehouse(self.__arm, 2)
        self.__move.my_navigation("a")
        print("测试完成！")

    def b6(self):
        input("按回车键开始导航...")
        self.__move.my_navigation("r")
        self.rotation_correction()
        Movement.grab_basket_from_station(self.__arm)
        Movement.put_basket_to_robot(self.__arm, 1)
        # self.__move.my_navigation("y")
        # Movement.recognition_orchard_banana_tree(self.__arm)  # 假装识别一下
        location = FruitLocationOnTree.TOP_CENTER
        self.__move.my_navigation("c")
        Movement.grab_banana_on_tree(self.__arm, location)
        Movement.put_fruit_into_basket(self.__arm, 1, 1.5)
        self.__move.my_navigation("j")
        self.rotation_correction()
        Movement.grab_basket_to_station(self.__arm, 1)
        self.__move.my_navigation("a")

        print("测试完成！")

    def b7(self):
        pass

    def b8(self):
        pass

    def b9(self):
        pass

    def b10(self):
        pass

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
