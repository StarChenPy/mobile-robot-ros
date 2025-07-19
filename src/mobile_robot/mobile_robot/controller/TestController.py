import rclpy

from ..param.ArmMovement import ArmMovement
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
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

        self.movement = ArmMovement(node)
        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)

        self.__param = NavigationPointParam("shandong_trials_navigation_point.yml")

    def run(self):
        self.robot.with_robot_connect()
        # self.arm.back_origin()

        self.movement.ready_to_grab_fruit_from_station()

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
                from_wall = self.sensor.get_distance_from_wall(Direction.FRONT)
                corrective_data.append(Corrective(Direction.FRONT, round(from_wall, 3)))
            elif direction == "b":
                sonar = self.sensor.get_sonar()
                from_wall = Math.distance_from_origin(-5, sonar[0], 5, sonar[1]) + 0.222
                corrective_data.append(Corrective(Direction.BACK, round(from_wall, 3)))
            elif direction == "l":
                from_wall = self.sensor.get_distance_from_wall(Direction.LEFT)
                corrective_data.append(Corrective(Direction.LEFT, round(from_wall, 3)))
            elif direction == "r":
                from_wall = self.sensor.get_distance_from_wall(Direction.RIGHT)
                print(type(from_wall))
                corrective_data.append(Corrective(Direction.RIGHT, round(from_wall, 3)))

        corrective_point = CorrectivePoint(point.x, point.y, point.yaw, corrective_data)

        print(f"已生成矫正点数据: {corrective_point}")
        if "y" == input("是否保存？y/n: "):
            if point_name is None:
                point_name = input("输入矫正点名称: ")
            self.__param.set_navigation_point(point_name, corrective_point)
