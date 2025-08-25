import time

from ..param import NavMovement, ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.RobotService import RobotService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.GrabGrapeWall import GrabGrapeWall
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class TaskDController:
    def __init__(self, node):
        self.node = node
        self.logger = Logger()

        self.vision = VisionService(node)
        self.arm = ArmService(node)
        self.sensor = SensorService(node)
        self.robot = RobotService(node)
        self.move = MoveService(node)

    def run(self):
        self.robot.with_robot_connect()
        self.arm.back_origin()
        ArmMovement.motion(self.arm)
        self.sensor.correction("c_start")
        self.robot.set_start_led(False)

        self.robot.with_start_button()

        start_time = time.time()

        self.grab_baskets()
        self.grab_grapes()
        self.put_baskets()

        # 回起始区，结束任务
        self.move.my_navigation("c_start")
        self.arm.nod_servo(90)
        self.robot.set_start_led(False)

        end_time = time.time()
        use_time = end_time - start_time
        self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")

    def grab_baskets(self):
        self.move.my_navigation("s_y_2_r")
        self.move.rotation_correction(Direction.RIGHT, True)
        self.sensor.lidar_revise(1.52)
        self.arm.grab_slope_basket_from_station(Direction.RIGHT)
        ArmMovement.put_basket_to_robot(self.arm, 1)
        ArmMovement.motion(self.arm)

        self.move.my_navigation("s_y_1_r")
        self.move.rotation_correction(Direction.RIGHT, True)
        self.arm.grab_basket_from_station(Direction.RIGHT)
        ArmMovement.put_basket_to_robot(self.arm, 2)
        ArmMovement.motion(self.arm)

        self.move.my_navigation("s_y_3_l")
        self.move.rotation_correction(Direction.LEFT, True)
        self.sensor.lidar_revise(1.52)
        self.arm.grab_basket_from_station(Direction.RIGHT)
        ArmMovement.put_basket_to_robot(self.arm, 3)
        ArmMovement.motion(self.arm)

    def grab_grapes(self):
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.basket_1 = [FruitType.PURPLE_GRAPE] * 6
        grab_grape_wall.basket_2 = [FruitType.GREEN_GRAPE] * 6
        grab_grape_wall.basket_3 = [FruitType.YELLOW_GRAPE] * 6

        self.move.my_navigation("c_2")
        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.find_grape_and_grab("c_3")
        if not grab_grape_wall.has_grape():
            return

        self.move.my_navigation("c_4")
        grab_grape_wall.direction = Direction.RIGHT
        grab_grape_wall.find_grape_and_grab("v_2")
        if not grab_grape_wall.has_grape():
            return

        self.move.my_navigation("c_4")
        self.move.my_navigation("v_3")
        grab_grape_wall.direction = Direction.RIGHT
        grab_grape_wall.find_grape_and_grab("c_5")
        if not grab_grape_wall.has_grape():
            return

        self.move.my_navigation("v_4")
        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.find_grape_and_grab("c_6")

    def put_baskets(self):
        # 去放第1个框子
        self.move.my_navigation("s_r_3_r")
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 1)
        ArmMovement.put_basket_to_station(self.arm, Direction.RIGHT)
        ArmMovement.motion(self.arm)

        # 去放第2个框子
        self.move.my_navigation("s_r_2_r")
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 2)
        ArmMovement.put_basket_to_station(self.arm, Direction.RIGHT)
        ArmMovement.motion(self.arm)

        # 去放第3个框子
        self.move.my_navigation("c_8")
        self.move.my_navigation("s_r_1_l")
        self.move.rotation_correction()
        ArmMovement.grab_basket_from_robot(self.arm, 3)
        ArmMovement.put_basket_to_station(self.arm, Direction.LEFT)
        ArmMovement.motion(self.arm)
