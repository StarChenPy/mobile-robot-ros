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
from ..util import Math
from ..util.ConfigAndParam import ConfigAndParam
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class CModuleController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__point_param = ConfigAndParam()

        self.__arm = ArmService(node)
        self.__move = MoveService(node)
        self.__sensor = SensorService(node)
        self.__vision = VisionService(node)

    def patrol_the_line(self, task: dict[int: FruitType], target_point: NavigationPoint, direction: Direction,
                        corrective_data: tuple[float, float] = None):
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
        time.sleep(2)

        odom = self.__sensor.get_odom_data()
        points = Math.point_to_point(NavigationPoint(odom.x, odom.y, odom.w), target_point, 0.2)

        for point in points:
            self.__move.navigation([point], 0.2, True)

            if corrective_data:
                dis = corrective_data[0] + (point.x - corrective_data[1])
                self.__move.corrective(CorrectivePoint.form_point(point, [Corrective(direction.invert(), dis)]))

            # 获取可靠的水果识别结果
            result = self.get_identify_result()
            if not result:
                continue

            self.__logger.info("寻找到水果，准备抓取.")

            for key in task:
                if task[key] == FruitType.get_by_value(result.classId):
                    # 平移到水果前
                    center = result.box.get_rectangle_center()
                    travel_distance = Math.pixel_to_horizontal_distance(320 - center.x, result.distance)
                    self.__move.line(travel_distance if direction == Direction.RIGHT else -travel_distance)

                    # 再次检测
                    result = None
                    for i in range(1, 6):
                        result = self.get_identify_result()
                        if result:
                            if result.distance != 0:
                                break
                        self.__logger.warn(f"二次检测未能通过，正在重试第 {i} 次")

                    if not result or result.distance == 0:
                        self.__logger.error("二次检测失败，跳过该水果")
                        break

                    # 获取水果高度并抓取到框子中
                    self.__arm.grab_fruit(result.distance * 100, direction)
                    ArmMovement.put_fruit_into_basket(self.__arm, key)
                    ArmMovement.recognition_orchard(self.__arm, direction)
                    break

    def get_identify_result(self):
        results = self.__vision.get_onnx_identify_result()
        result = None
        for r in results:
            if r.box.get_area() < 3000:
                self.__logger.warn(f"[GrabFruitController] 识别对象面积 {r.box.get_area()} 过小，跳过")
                continue
            if 220 > r.box.get_rectangle_center().x > 420:
                self.__logger.warn(
                    f"[GrabFruitController] 识别对象中心点 {r.box.get_rectangle_center().x} 位于边缘，跳过")
                continue
            result = r
        return result

    def unknown_fruit_grab_task(self, task: dict[int: FruitType]):
        self.__logger.info("正在前往 走廊1")
        self.__move.navigation(NavigationPath.START_TO_ORCHARD_ENTER_1)
        self.__move.navigation([self.__point_param.get_navigation_point("orchard_corridor_start_1_corrective_point")], 0.2, True)

        self.__logger.info("开始寻找 走廊1 的水果")
        time.sleep(1)
        self.patrol_the_line(task, self.__point_param.get_navigation_point("orchard_corridor_end_1_point"), Direction.RIGHT, (0.5, 2))
        self.__arm.control(ArmMovement.MOVING, is_block=False)

        # ----------------去2号走廊的分割线----------------

        self.__logger.info("正在前往 走廊2")
        self.__move.navigation(NavigationPath.EXIT_1_TO_EXIT_2)

        # ----------------去2号走廊的分割线----------------

        self.__logger.info("开始寻找 走廊2 的水果")
        self.patrol_the_line(task, NavigationPath.param.get_navigation_point("orchard_corridor_start_2_point"), Direction.LEFT, (0.483, 2.84))
        self.__arm.control(ArmMovement.MOVING, is_block=False)

        self.__logger.info("正在前往果仓区域")
        self.__move.navigation([self.__point_param.get_navigation_point("orchard_corridor_start_2_corrective_point")])
        self.__move.navigation(NavigationPath.ORCHARD_CORRIDOR_2_TO_WAREHOUSE)
        if 1 in task:
            self.__logger.info("放置 果篮1")
            self.__move.navigation([self.__point_param.get_navigation_point("warehouse_1_point")], 0.2, True)
            ArmMovement.grab_basket_to_warehouse(self.__arm, 1)
        if 2 in task:
            self.__move.navigation([self.__point_param.get_navigation_point("warehouse_corrective_point"), self.__point_param.get_navigation_point("warehouse_2_point")], 0.2)
            self.__logger.info("放置 果篮2")
            ArmMovement.grab_basket_to_warehouse(self.__arm, 2)
        if 3 in task:
            self.__move.navigation([self.__point_param.get_navigation_point("warehouse_corrective_point"), self.__point_param.get_navigation_point("warehouse_3_point")], 0.2)
            self.__logger.info("放置 果篮3")
            ArmMovement.grab_basket_to_warehouse(self.__arm, 3)

        self.__logger.info("正在前往起始区")
        self.__move.navigation(NavigationPath.B_MODULE_4)

    def known_fruit_grab_task(self, task):
        i = 0
        for orchard in task:
            i += 1
            self.__logger.info(f"正在前往采摘点 {i}")
            self.__move.navigation(orchard)

            self.__logger.info(f"正在抓取水果")

            self.__arm.grab_fruit(FruitHeight.TALL.value, Direction.RIGHT)
            if i < 5:
                ArmMovement.put_fruit_into_basket(self.__arm, 2)

            time.sleep(1)
            self.__arm.control(ArmMovement.MOVING)

        self.__move.navigation(NavigationPath.ORCHARD_CORRIDOR_2_TO_WAREHOUSE)
        self.__move.navigation([self.__point_param.get_navigation_point("warehouse_1_point")], 0.2)

        self.__logger.info(f"正在放置水果")
        self.__arm.control(ArmMovement.READY_PULL_WAREHOUSE)
        self.__arm.control(ArmMovement.PULL_WAREHOUSE)
        ArmMovement.grab_basket_to_warehouse(self.__arm, 2)

        self.__logger.info(f"任务完成，正在返程")
        self.__move.navigation(NavigationPath.B_MODULE_4)
        self.__logger.info(f"已知任务结束")
