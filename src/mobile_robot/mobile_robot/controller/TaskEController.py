import time

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
from ..util.Station import Station


@singleton
class TaskEController:
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
        self.robot.set_start_led(True)
        self.arm.back_origin()
        ArmMovement.motion(self.arm)
        self.sensor.correction("c_start")
        self.robot.set_start_led(False)

        self.robot.with_start_button()

        start_time = time.time()

        self.grab_baskets()
        self.grab_grapes()
        self.grab_apples()
        self.put_baskets()

        self.move.my_navigation("c_start")
        self.arm.servo_nod(90)
        self.robot.set_start_led(False)

        end_time = time.time()
        use_time = end_time - start_time
        self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")

    def grab_baskets(self):
        Station.YELLOW_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(1))
        self.arm.plan_list(ArmMovement.motion())

        Station.YELLOW_3.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2))
        self.arm.plan_list(ArmMovement.motion())

        Station.YELLOW_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(3))
        self.arm.plan_list(ArmMovement.motion())

    def grab_apples(self):
        grab_apple_tree = GrabAppleTree(self.node)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        grab_apple_tree.basket_2 = [FruitType.GREEN_APPLE, FruitType.YELLOW_APPLE]
        grab_apple_tree.basket_3 = [FruitType.YELLOW_APPLE, FruitType.PURPLE_APPLE]

        # 抓果树 1 后面
        self.move.my_navigation("c_7")
        self.move.my_navigation("t_1_f")
        grab_apple_tree.direction = Direction.LEFT
        grab_apple_tree.grab_apple_from_tree()
        if not grab_apple_tree.has_apple():
            return

        # 抓果树 3 右面
        self.move.my_navigation("c_7")
        self.move.my_navigation("t_3_r")
        grab_apple_tree.direction = Direction.RIGHT
        grab_apple_tree.grab_apple_from_tree()
        if not grab_apple_tree.has_apple():
            return

        # 抓果树 1 前面
        self.move.my_navigation("c_7")
        self.move.my_navigation("t_1_b")
        grab_apple_tree.direction = Direction.RIGHT
        if not grab_apple_tree.has_apple():
            return

        # 抓果树 2 右面
        self.move.my_navigation("c_7")
        self.move.my_navigation("t_2_r")
        grab_apple_tree.direction = Direction.LEFT
        grab_apple_tree.grab_apple_from_tree()
        if not grab_apple_tree.has_apple():
            return

        # 抓果树 2 左面
        self.move.my_navigation("c_7")
        self.move.my_navigation("c_8")
        self.move.my_navigation("t_2_l")
        grab_apple_tree.direction = Direction.LEFT
        grab_apple_tree.grab_apple_from_tree()
        if not grab_apple_tree.has_apple():
            return

        # 抓果树 3 左面
        self.move.my_navigation("c_8")
        self.move.my_navigation("t_3_l")
        grab_apple_tree.direction = Direction.RIGHT
        grab_apple_tree.grab_apple_from_tree()

        self.move.my_navigation("c_8")


    def grab_grapes(self):
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.basket_1 = [FruitType.PURPLE_GRAPE] * 2
        grab_grape_wall.basket_2 = [FruitType.GREEN_GRAPE] * 2
        grab_grape_wall.basket_3 = [FruitType.YELLOW_GRAPE] * 2

        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.find_grape_and_grab("c_2", "c_3")
        if not grab_grape_wall.has_grape():
            return

        grab_grape_wall.direction = Direction.RIGHT
        grab_grape_wall.find_grape_and_grab("c_4", "v_2")
        if not grab_grape_wall.has_grape():
            return

        self.move.my_navigation("c_4")
        grab_grape_wall.direction = Direction.RIGHT
        grab_grape_wall.find_grape_and_grab("v_3", "c_5")
        if not grab_grape_wall.has_grape():
            return

        grab_grape_wall.direction = Direction.LEFT
        grab_grape_wall.find_grape_and_grab("v_4", "c_6")

    def put_baskets(self):
        # 去放第1个框子
        Station.RED_2.nav_and_put(self.node, 3)
        self.arm.plan_list(ArmMovement.motion())

        # 去放第2个框子
        Station.RED_1.nav_and_put(self.node, 2)
        self.arm.plan_list(ArmMovement.motion())

        # 去放第3个框子
        self.move.my_navigation("c_8")
        Station.RED_3.nav_and_put(self.node, 1)
        self.arm.plan_list(ArmMovement.motion())