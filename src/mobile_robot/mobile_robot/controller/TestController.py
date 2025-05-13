import rclpy

from ..popo.ArmMovement import ArmMovement
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.MotorMovement import MotorMovement
from ..popo.NavigationPoint import NavigationPoint
from ..popo.ServoMotor import ServoMotor
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Math
from ..util.Logger import Logger
from ..util.NavigationPointParam import NavigationPointParam
from ..util.Singleton import singleton
from ..param import ArmMovement as Movement


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

        print(angles)

        min_angle = min(angles, key=lambda x: (abs(x), -x))
        if abs(min_angle) > 2:
            self.__move.rotate(min_angle)

    def run(self):
        self.rotation_correction()

    def create_point(self):
        s = input("是否已有点？y/n: ")
        point_name = None
        if s == "y":
            point_name = input("请输入矫正点名称：")
            point = self.__param.get_navigation_point(point_name)
        else:
            x, y, yaw = input("输入当前坐标(x y yaw): ").split(" ")
            point = NavigationPoint(x, y, yaw)

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
