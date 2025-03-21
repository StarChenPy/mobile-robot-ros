import rclpy

from ..param import ArmMovement, NavigationPath
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.FruitHeight import FruitHeight
from ..popo.FruitType import FruitType
from ..popo.NavigationPoint import NavigationPoint
from ..popo.Rectangle import Rectangle
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService
from ..util.Math import Math
from ..util.Singleton import singleton


def get_fruit_height(box: Rectangle) -> FruitHeight:
    height = box.get_rectangle_center().y

    if height < 150:
        return FruitHeight.TALL
    elif height < 240:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.LOW


@singleton
class GrabFruitController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__arm = ArmService(node)
        self.__move = MoveService(node)
        self.__sensor = SensorService(node)
        self.__vision = VisionService(node)

    def vision(self):
        return self.__vision.get_onnx_identify_result()

    def grub_task(self, task: dict[int: FruitType]):
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_ENTER_1, 0.4, True)

        ArmMovement.recognition_orchard(self.__arm, Direction.RIGHT)

        odom = self.__sensor.get_odom_data()
        points = Math.point_to_point(NavigationPoint(odom.x, odom.y, odom.w), NavigationPath.ORCHARD_CORRIDOR_EXIT_1_POINT, 0.3)

        for point in points:
            self.__move.navigation([point], 0.2, True)

            dis = 0.416 + (point.x - 1.92)
            self.__move.corrective(CorrectivePoint.form_point(point, [Corrective(Direction.LEFT, dis)]))

            results = self.__vision.get_onnx_identify_result()
            result = None
            for r in results:
                if r.box.get_area() < 5000:
                    continue
                result = r
            if not result:
                continue

            for key in task:
                if task[key] == FruitType.get_by_value(result.classId):
                    center = result.box.get_rectangle_center()
                    travel_distance = (320 - center.x) / 1600
                    self.__move.line(travel_distance)
                    ArmMovement.grab_fruit(self.__arm, get_fruit_height(result.box), Direction.RIGHT)
                    ArmMovement.put_fruit_into_basket(self.__arm, key)
                    ArmMovement.recognition_orchard(self.__arm, Direction.RIGHT)
                    break

        ArmMovement.recognition_orchard_end(self.__arm, Direction.RIGHT)
        self.__move.navigation(NavigationPath.EXIT_1_TO_EXIT_2, 0.4, True)

        # ----------------分割线----------------

        ArmMovement.recognition_orchard(self.__arm, Direction.LEFT)

        odom = self.__sensor.get_odom_data()
        points = Math.point_to_point(NavigationPoint(odom.x, odom.y, odom.w), NavigationPath.ORCHARD_CORRIDOR_ENTER_2_POINT, 0.3)

        for point in points:
            self.__move.navigation([point], 0.2, True)

            dis = 0.395 + (point.x - 2.74)
            self.__move.corrective(CorrectivePoint.form_point(point, [Corrective(Direction.RIGHT, dis)]))

            results = self.__vision.get_onnx_identify_result()
            result = None
            for r in results:
                if r.box.get_area() < 5000:
                    continue
                result = r
            if not result:
                continue

            for key in task:
                if task[key] == FruitType.get_by_value(result.classId):
                    center = result.box.get_rectangle_center()
                    travel_distance = (320 - center.x) / 1600
                    self.__move.line(-travel_distance)
                    ArmMovement.grab_fruit(self.__arm, get_fruit_height(result.box), Direction.LEFT)
                    ArmMovement.put_fruit_into_basket(self.__arm, key)
                    ArmMovement.recognition_orchard(self.__arm, Direction.LEFT)
                    break

        ArmMovement.recognition_orchard_end(self.__arm, Direction.LEFT)

        self.__move.navigation(NavigationPath.ORCHARD_CORRIDOR_1_TO_WAREHOUSE_1_POINT, 0.4, True)
        if 1 in task:
            ArmMovement.grab_basket_to_warehouse(self.__arm, 1)
        if 2 in task:
            self.__move.navigation([NavigationPath.WAREHOUSE_CORRECTIVE_POINT, NavigationPath.WAREHOUSE_2_POINT], 0.4, True)
            ArmMovement.grab_basket_to_warehouse(self.__arm, 2)
        if 3 in task:
            self.__move.navigation([NavigationPath.WAREHOUSE_CORRECTIVE_POINT, NavigationPath.WAREHOUSE_3_POINT], 0.4, True)
            ArmMovement.grab_basket_to_warehouse(self.__arm, 3)

        self.__arm.control(ArmMovement.MOVING, 0.4, True)
        self.__move.navigation(NavigationPath.B_MODULE_4, 0.4, True)

    def patrol_the_line(self, target_point: NavigationPoint, target_fruit: FruitType, direction: Direction) -> bool:
        self.__logger.info(f"[GrabFruitController] 准备开始 目标点 {target_point}, 目标水果 {target_fruit}")
        ArmMovement.recognition_orchard(self.__arm, direction)

        odom = self.__sensor.get_odom_data()
        points = Math.point_to_point(NavigationPoint(odom.x, odom.y, odom.w), target_point, 0.3)

        for point in points:
            self.__move.navigation([point], 0.2, True)

            if direction == Direction.LEFT:
                dis = 0.407 + (point.x - 2.78)
                self.__move.corrective(CorrectivePoint.form_point(point, [Corrective(Direction.LEFT, dis)]))
            elif direction == Direction.RIGHT:
                dis = 0.401 + (point.x - 1.92)
                self.__move.corrective(CorrectivePoint.form_point(point, [Corrective(Direction.RIGHT, dis)]))

            results = self.__vision.get_onnx_identify_result()

            self.__logger.info(f"[GrabFruitController] 识别到的内容: {results}")

            result = None
            for r in results:
                if r.box.get_area() < 5000:
                    continue
                if r.classId != target_fruit.value:
                    continue
                result = r

            if not result:
                continue

            center = result.box.get_rectangle_center()
            travel_distance = (320 - center.x) / 2000
            self.__move.line(-travel_distance if direction == Direction.RIGHT else travel_distance)
            ArmMovement.grab_fruit(self.__arm, get_fruit_height(result.box), direction)
            return True

        return False
