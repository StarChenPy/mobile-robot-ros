import time

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

        self.start_time = None

    def run(self):
        self.robot.with_robot_connect()
        self.robot.set_start_led(True)
        self.arm.back_origin()
        self.arm.plan_list(ArmMovement.motion())
        self.robot.set_start_led(False)

        self.robot.with_start_button()
        self.start_time = time.time()
        self.sensor.init_odom_all(NavigationPoint(1.827, 1.51, -16))
        # self.sensor.correction("start")
        time.sleep(0.5)

        self.grab_baskets()
        self.grab_grapes()
        self.grab_apples()
        self.put_baskets()

        self.move.my_navigation("start")
        self.arm.servo_nod(90)
        self.robot.set_start_led(False)

        end_time = time.time()
        use_time = end_time - self.start_time
        self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")

    def grab_baskets(self):
        self.move.my_navigation("c_1")

        # 抓1号框子和2号框子
        Station.YELLOW_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(1))
        Station.YELLOW_1.grab_basket(self.node, again=True)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2) + ArmMovement.motion(), block=False)

        self.move.my_navigation("c_2")
        # 抓3号框子
        Station.YELLOW_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(3))
        self.arm.plan_list(ArmMovement.motion(), block=False)

    def grab_apples(self):
        grab_apple_tree = GrabAppleTree(self.node)
        grab_apple_tree.basket_1 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]
        grab_apple_tree.basket_2 = [FruitType.YELLOW_APPLE, FruitType.YELLOW_APPLE]
        grab_apple_tree.basket_3 = [FruitType.RED_APPLE, FruitType.GREEN_APPLE]

        def grab_apple_tree_func(_path: list, _direction: Direction, target_waypoint: str):
            now_time = time.time()
            use_time = now_time - self.start_time
            if use_time > (8 * 60):
                self.logger.warn("在抓取苹果上耗费的时间过长，直接去放框子!")
                return True

            if _path:
                for r in _path:
                    self.move.my_navigation(r)

            grab_apple_tree.direction = _direction
            self.move.my_navigation(target_waypoint)
            grab_apple_tree.grab_apple_from_tree()
            if not grab_apple_tree.has_apple():
                return True
            return False

        plans = [
            (["c_3"], Direction.LEFT, "t_3_b"),
            (["c_3"], Direction.LEFT, "t_2_l"),
            (["c_3"], Direction.RIGHT, "t_1_f"),
            (["c_2"], Direction.RIGHT, "t_1_b"),
        ]

        for path, direction, waypoint in plans:
            self.logger.info(f"尝试到 {waypoint} 抓苹果")
            if grab_apple_tree_func(path, direction, waypoint):
                self.logger.info("苹果抓取完成!")
                break

    def grab_grapes(self):
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.basket_1 = [FruitType.PURPLE_GRAPE, FruitType.GREEN_GRAPE]
        grab_grape_wall.basket_2 = [FruitType.PURPLE_GRAPE, FruitType.YELLOW_GRAPE]
        grab_grape_wall.basket_3 = [FruitType.GREEN_GRAPE, FruitType.YELLOW_GRAPE]

        def grab_grape_wall_func(_path: list, _direction: Direction, target_waypoint: str, _continuous: bool):
            now_time = time.time()
            use_time = now_time - self.start_time
            if use_time > (8 * 60):
                self.logger.warn("在抓取葡萄上耗费的时间过长，直接去放框子!")
                if self.arm.wait_plan_finish():
                    self.arm.plan_list(ArmMovement.motion())
                return True

            if _path:
                for r in _path:
                    self.move.my_navigation(r)

            grab_grape_wall.direction = _direction
            self.move.my_navigation(target_waypoint)
            grab_grape_wall.grab_grape_from_wall(_continuous)
            if not grab_grape_wall.has_grape():
                return True
            return False

        plans = [
            (["c_2"], Direction.RIGHT, "v_1", True),
            ([], Direction.RIGHT, "v_2", True),
            ([], Direction.RIGHT, "v_3", True),
            ([], Direction.RIGHT, "v_4", True),
            ([], Direction.RIGHT, "v_5", False),
            (["c_2", "c_3"], Direction.RIGHT, "v_6", True),
            ([], Direction.RIGHT, "v_7", True),
            ([], Direction.RIGHT, "v_8", False)
        ]

        for path, direction, waypoint, continuous in plans:
            self.logger.info(f"尝试到 {waypoint} 抓葡萄")
            if grab_grape_wall_func(path, direction, waypoint, continuous):
                self.logger.info("葡萄抓取完成!")
                if self.arm.wait_plan_finish():
                    self.arm.plan_list(ArmMovement.motion())
                break

    def put_baskets(self):
        # 去放第1个和第2个框子
        Station.RED_3.nav_and_put(self.node, 3, again=True)
        ArmMovement.grab_basket_from_robot(self.arm, 2)
        Station.RED_3.put_basket(self.node)
        self.arm.plan_list(ArmMovement.motion())

        self.move.my_navigation("c_2")
        # 去放第3个框子
        Station.RED_2.nav_and_put(self.node, 1)
        self.arm.plan_list(ArmMovement.motion())

