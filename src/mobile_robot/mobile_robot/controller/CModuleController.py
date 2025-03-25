import time

import rclpy

from ..param import ArmMovement, NavigationPath
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.FruitHeight import FruitHeight
from ..popo.FruitType import FruitType
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util import Util
from ..util import Math
from ..util.Singleton import singleton


@singleton
class CModuleController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__arm = ArmService(node)
        self.__move = MoveService(node)
        self.__sensor = SensorService(node)
        self.__vision = VisionService(node)

    def patrol_the_line(self, task: dict[int: FruitType], target_point: NavigationPoint, direction: Direction,
                        corrective_data: tuple[float, float]=None):
        """
        巡线，给予一个坐标点，以分段的方式步进向前，同时扫描水果进行抓取
        @param task: 要执行的任务，由篮子编号与对应的水果类型组成
        @param target_point: 目标点
        @param direction 检测方向，同时决定了机械臂的旋转方向与雷达的扫描方向
        @param corrective_data 矫正数据，可以辅助走直线
        """
        if direction != Direction.RIGHT and direction != Direction.LEFT:
            self.__logger.error(f"[GrabFruitController] 传入了不支持的方向 {direction}")
            raise ValueError(f"传入了不支持的方向 {direction}")

        ArmMovement.recognition_orchard(self.__arm, direction)

        odom = self.__sensor.get_odom_data()
        points = Math.point_to_point(NavigationPoint(odom.x, odom.y, odom.w), target_point, 0.2)

        for point in points:
            self.__move.navigation([point], 0.2, True)

            if corrective_data is not None:
                dis = corrective_data[0] + (point.x - corrective_data[1])
                self.__move.corrective(CorrectivePoint.form_point(point, [Corrective(direction.invert(), dis)]))

            results = self.__vision.get_onnx_identify_result()
            result = None
            for r in results:
                if r.box.get_area() < 5000:
                    self.__logger.warn(f"[GrabFruitController] 识别对象面积 {r.box.get_area()} 过小，跳过")
                    continue
                if 140 > r.box.get_rectangle_center().x > 500:
                    self.__logger.warn(f"[GrabFruitController] 识别对象中心点 {r.box.get_rectangle_center().x} 位于边缘，跳过")
                    continue
                result = r
            if not result:
                continue

            for key in task:
                if task[key] == FruitType.get_by_value(result.classId):
                    center = result.box.get_rectangle_center()
                    travel_distance = (320 - center.x) / 1400
                    self.__move.line(travel_distance if direction == Direction.RIGHT else -travel_distance)
                    ArmMovement.grab_fruit(self.__arm, Util.get_fruit_height(result.box), direction)
                    ArmMovement.put_fruit_into_basket(self.__arm, key)
                    ArmMovement.recognition_orchard(self.__arm, direction)
                    break

        self.__arm.control(ArmMovement.MOVING, 45, False)

    def unknown_fruit_grab_task(self, task: dict[int: FruitType]):
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_ENTER_1, 0.4, True)
        self.__move.navigation([NavigationPath.ORCHARD_CORRIDOR_ENTER_1_CORRECTIVE_POINT], 0.2, True)

        self.patrol_the_line(task, NavigationPath.ORCHARD_CORRIDOR_EXIT_1_POINT, Direction.RIGHT, (0.5, 2))

        # ----------------去2号走廊的分割线----------------

        self.__move.navigation(NavigationPath.EXIT_1_TO_EXIT_2, 0.4, True)
        self.__move.navigation([NavigationPath.ORCHARD_CORRIDOR_EXIT_2_POINT], 0.2, True)

        # ----------------去2号走廊的分割线----------------

        self.patrol_the_line(task, NavigationPath.ORCHARD_CORRIDOR_ENTER_2_POINT, Direction.LEFT, (0.48, 2.82))

        self.__move.navigation(NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT, 0.4, True)
        if 1 in task:
            ArmMovement.grab_basket_to_warehouse(self.__arm, 1)
        if 2 in task:
            self.__move.navigation([NavigationPath.WAREHOUSE_CORRECTIVE_POINT, NavigationPath.WAREHOUSE_2_POINT], 0.2, True)
            ArmMovement.grab_basket_to_warehouse(self.__arm, 2)
        if 3 in task:
            self.__move.navigation([NavigationPath.WAREHOUSE_CORRECTIVE_POINT, NavigationPath.WAREHOUSE_3_POINT], 0.2, True)
            ArmMovement.grab_basket_to_warehouse(self.__arm, 3)

        self.__move.navigation([NavigationPath.WAREHOUSE_CORRECTIVE_POINT], 0.2, True)
        self.__move.navigation(NavigationPath.B_MODULE_4, 0.4, True)

    def known_fruit_grab_task(self, task):
        def grab_and_store(orchard_path, warehouse_path):
            """通用的抓取水果并存储的函数"""
            self.__move.navigation(orchard_path, 0.4, True)
            ArmMovement.grab_fruit(self.__arm, FruitHeight.TALL, Direction.RIGHT)
            self.__move.navigation(warehouse_path, 0.4, True)
            self.__arm.control(ArmMovement.READY_PULL_WAREHOUSE)
            self.__arm.control(ArmMovement.PULL_WAREHOUSE)
            time.sleep(1)
            self.__arm.control(ArmMovement.MOVING)

        for orchard, warehouse in task:
            grab_and_store(orchard, warehouse)
        self.__move.navigation(NavigationPath.B_MODULE_4, 0.4, True)
