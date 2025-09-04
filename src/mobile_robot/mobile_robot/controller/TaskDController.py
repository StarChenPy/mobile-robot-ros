import time

from ..param import ArmMovement
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

    def run(self):
        self.robot.with_robot_connect()
        self.robot.set_start_led(True)
        self.arm.back_origin()
        self.arm.plan_list(ArmMovement.motion())
        self.sensor.correction("c_start")
        self.robot.set_start_led(False)

        self.robot.with_start_button()

        start_time = time.time()

        self.grab_baskets()
        self.grab_grapes()
        self.put_baskets()

        # 回起始区，结束任务
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

        Station.YELLOW_1.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(2) + ArmMovement.motion(), block=False)

        Station.YELLOW_3.nav_and_grab(self.node)
        self.arm.plan_list(ArmMovement.put_basket_to_robot(3))
        self.arm.plan_list(ArmMovement.motion(), block=False)

    def grab_grapes(self):
        grab_grape_wall = GrabGrapeWall(self.node)
        grab_grape_wall.basket_1 = [FruitType.PURPLE_GRAPE] * 6
        grab_grape_wall.basket_2 = [FruitType.GREEN_GRAPE] * 6
        grab_grape_wall.basket_3 = [FruitType.YELLOW_GRAPE] * 6

        def grab_graph_wall(_path: list, _direction: Direction, target_waypoint: str):
            if _path:
                for r in _path:
                    self.move.my_navigation(r)

            grab_grape_wall.direction = _direction
            self.move.my_navigation(target_waypoint)
            grab_grape_wall.grab_grape_from_wall()
            if not grab_grape_wall.has_grape():
                return True
            return False

        plans = [
            ([], Direction.RIGHT, "v_2"),
            ([], Direction.RIGHT, "v_1"),
            (["c_5"], Direction.LEFT, "v_3"),
            (["c_2"], Direction.LEFT, "v_4"),
            ([], Direction.LEFT, "v_5"),
            ([], Direction.LEFT, "v_6"),
            (["c_5"], Direction.LEFT, "v_7"),
            ([], Direction.LEFT, "v_8"),
        ]

        for path, direction, waypoint in plans:
            self.logger.info(f"尝试扫描到 {waypoint} 抓葡萄")
            if grab_graph_wall(path, direction, waypoint):
                self.logger.info("葡萄抓取完成!")
                break

    def put_baskets(self):
        # 去放第1个框子
        Station.RED_3.nav_and_put(self.node, 3)
        self.arm.plan_list(ArmMovement.motion())

        # 去放第2个框子
        Station.RED_2.nav_and_put(self.node, 2)
        self.arm.plan_list(ArmMovement.motion())

        # 去放第3个框子
        self.move.my_navigation("c_8")
        Station.RED_1.nav_and_put(self.node, 1)
        self.arm.plan_list(ArmMovement.motion())
