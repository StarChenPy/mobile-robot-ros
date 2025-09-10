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
from ..util.GrabGrapeWall import GrabGrapeWall
from ..util.Logger import Logger
from ..util.Singleton import singleton
from ..util.Station import Station


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

        self.start_time = None

    def run(self):
        self.robot.with_robot_connect()
        self.robot.set_start_led(True)
        self.arm.back_origin()
        self.arm.plan_list(ArmMovement.motion())
        # self.sensor.init_odom_all(NavigationPoint(2.4, 1.54, 180))
        self.sensor.correction("c_start")
        self.robot.set_start_led(False)

        self.robot.with_start_button()

        self.start_time = time.time()

        self.grab_baskets()
        self.grab_grapes()
        self.put_baskets()

        # 回起始区，结束任务
        self.move.my_navigation("end")
        self.robot.set_start_led(False)

        end_time = time.time()
        use_time = end_time - self.start_time
        self.logger.info(f"执行时间：{int(use_time / 60)} 分 {use_time % 60}秒")

    def grab_baskets(self):
        Station.YELLOW_3.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(1))
        Station.YELLOW_3.grab_basket(self.node, again=True)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2))

        Station.YELLOW_2.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(3) + ArmMovement.motion(), block=False)

        # Station.YELLOW_1.nav_and_grab(self.node)
        # self.arm.plan_list(ArmMovement.put_basket_to_robot(3))
        # self.arm.plan_list(ArmMovement.motion(), block=False)

    def grab_grapes(self):
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.basket_1 = [FruitType.GREEN_GRAPE] * 6
        grab_grape_wall.basket_2 = [FruitType.YELLOW_GRAPE] * 6
        grab_grape_wall.basket_3 = [FruitType.PURPLE_GRAPE] * 6

        def grab_graph_wall(_path: list, _direction: Direction, target_waypoint: str, _continuous: bool):
            # now_time = time.time()
            # use_time = now_time - self.start_time
            # if use_time > (8 * 60):
            #     self.logger.warn("在抓取葡萄上耗费的时间过长，直接去放框子!")
            #     self.arm.wait_plan_finish()
            #     self.arm.plan_list(ArmMovement.motion())
            #     return True

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
            (["c_4"], Direction.LEFT, "v_1", True),
            ([], Direction.LEFT, "v_2", False),
            (["c_4"], Direction.LEFT, "v_7", False),
            (["c_4"], Direction.RIGHT, "v_4", False),
            (["c_4"], Direction.LEFT, "v_3", False),
            (["c_2"], Direction.RIGHT, "v_5", True),
            ([], Direction.RIGHT, "v_6", False),
            (["c_3", "c_4"], Direction.RIGHT, "v_8", True),
            ([], Direction.RIGHT, "v_9", False),
            (["c_5"], Direction.LEFT, "v_10", False),
        ]

        for path, direction, waypoint, continuous in plans:
            self.logger.info(f"尝试到 {waypoint} 抓葡萄")
            if grab_graph_wall(path, direction, waypoint, continuous):
                self.logger.info("葡萄抓取完成!")
                break

    def put_baskets(self):
        self.move.my_navigation("c_4")

        # 去放第1个框子
        Station.RED_1.nav_and_put(self.node, 3)
        self.arm.plan_list(ArmMovement.motion(), block=False)

        # 去放第2个框子
        Station.RED_2.nav_and_put(self.node, 2)
        self.arm.plan_list(ArmMovement.motion(), block=False)

        # 去放第3个框子
        self.move.my_navigation("c_7")
        Station.RED_3.nav_and_put(self.node, 1)
        self.arm.plan_list(ArmMovement.motion(), block=False)
