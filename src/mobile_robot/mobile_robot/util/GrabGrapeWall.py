import time

from rclpy.node import Node

from . import Math
from .Logger import Logger
from .Singleton import singleton
from ..param import ArmMovement
from ..param import RobotConstant
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService
from ..service.VisionService import VisionService


@singleton
class GrabGrapeWall:
    def __init__(self, node: Node):
        self.logger = Logger()
        self.node = node

        self.extra_move_distance = 0
        self.continuous = False
        self.direction = None
        self.basket_1 = []
        self.basket_2 = []
        self.basket_3 = []

        self.move = MoveService(node)
        self.arm = ArmService(node)
        self.vision = VisionService(node)
        self.sensor = SensorService(node)

    def has_grape(self):
        """
        判断框子中是否还有葡萄
        """

        grape_types = set(FruitType.grapes())
        for i in range(1, 4):
            basket = getattr(self, f"basket_{i}")
            if basket and any(fruit in grape_types for fruit in basket):
                return True
        return False

    def grapes_in_basket(self, grape: FruitType | str) -> bool:
        """
        判断水果在不在需求列表中
        :param grape: 葡萄，可以是名称或是FruitType
        """

        if isinstance(grape, str):
            grape = FruitType(grape)

        basket = self.basket_1 + self.basket_2 + self.basket_3
        return grape in basket

    def get_grape_in_which_basket(self, grape: FruitType | str) -> int:
        """
        获取给定的葡萄所在的框子编号
        """

        if isinstance(grape, str):
            grape = FruitType(grape)

        for i in range(1, 4):
            bastet = getattr(self, f"basket_{i}")
            if grape in bastet:
                return i
        return 0

    def find_grapes_you_need(self) -> list[IdentifyResult]:
        """
        获取所需的葡萄识别结果
        """

        def get_result() -> list[IdentifyResult]:
            identify = self.vision.get_onnx_identify_depth(True, 95)
            return [i for i in identify if self.grapes_in_basket(i.class_id)]

        # 第一次结果
        result = get_result()

        while True:
            new_result = get_result()
            if len(result) == len(new_result):
                return result  # 数量一致，返回
            result = new_result  # 更新结果，继续循环

    def calculate_grape_lateral_distance(self, grape: IdentifyResult) -> float:
        """
        返回葡萄距离 OMS 中心的距离
        :param grape 葡萄识别结果
        :return 葡萄离OMS中心的横向现实距离
        """

        pixel_x = grape.box.get_rectangle_center().x
        if self.direction == Direction.LEFT:
            pixel_x = pixel_x - RobotConstant.CAMERA_WIDTH / 2
        elif self.direction == Direction.RIGHT:
            pixel_x = RobotConstant.CAMERA_WIDTH / 2 - pixel_x
        real_x = Math.pixel_to_horizontal_distance_x_centered(pixel_x, grape.distance)
        real_x += RobotConstant.IDENTIFY_GRAPE_DISTANCE - self.extra_move_distance
        if self.direction == Direction.LEFT:
            real_x += 0.05

        self.logger.debug(f"{grape.class_id} 距离OMS中心的真实 x 距离为: {real_x}")

        return real_x

    def calculate_grape_polar(self, fruit: IdentifyResult) -> tuple[float, float]:
        """
        计算出葡萄距离 OMS 中心的距离
        :param fruit 葡萄识别结果
        :return 以旋转中心为0，葡萄的极坐标
        """

        real_x = self.calculate_grape_lateral_distance(fruit)


        if self.direction == Direction.LEFT:
            distance = fruit.distance
            distance -= RobotConstant.GRAPE_THICKNESS
        else:
            distance = -fruit.distance
            distance += RobotConstant.GRAPE_THICKNESS
        r, angle = Math.cartesian_to_polar((0, 0), (real_x, distance))

        self.logger.debug(f"{fruit.class_id} 距离OMS中心的距离为 {r}, 角度为 {angle}")

        return r, angle

    def can_grab_fruit(self, grape: IdentifyResult) -> bool:
        """
        判断在这个位置能否抓取葡萄
        :param grape 葡萄识别结果
        :return 能否抓到葡萄
        """

        r, angle = self.calculate_grape_polar(grape)

        i = 1 if self.direction == Direction.LEFT else -1
        if not (RobotConstant.MIN_GRAB_ANGLE < angle * i < RobotConstant.MAX_GRAB_ANGLE):
            self.logger.debug(f"{grape.class_id} 距离OMS中心的角度为 {angle}, 超出范围.")
            return False

        if not (RobotConstant.MIN_GRAB_DISTANCE < r < RobotConstant.MAX_GRAB_DISTANCE):
            self.logger.debug(f"{grape.class_id} 距离OMS中心的距离为 {r}, 超出范围.")
            return False

        return True

    def close_grape(self, grape: IdentifyResult):
        """
        移动机器人到适合抓取的位置
        """
        if self.can_grab_fruit(grape):
            self.logger.info(f"{grape.class_id} 可以直接抓到, 无需移动.")
            return

        distance = self.calculate_grape_lateral_distance(grape)
        move_distance = distance - RobotConstant.MIN_GRAB_DISTANCE
        self.logger.info(f"为抓取 {grape.class_id} 移动 {move_distance}.")
        self.move.line(move_distance)

        self.extra_move_distance += move_distance

    def grab_grape(self, grape: IdentifyResult):
        """
        以合适的角度和伸缩长度抓取葡萄
        """

        r, angle = self.calculate_grape_polar(grape)
        from_bottom = Math.pixel_to_distance_from_center(grape.box.get_rectangle_center().y, grape.distance)
        from_bottom = 16 - (from_bottom * 100)

        telescopic = (r - 0.25) * 100

        self.logger.debug(f"抓取 {grape.class_id}. 方向 {self.direction.name}, 高度 {from_bottom}, 角度 {angle}, 伸缩 {telescopic}.")
        self.arm.plan_list(ArmMovement.grab_grape_on_wall(self.direction, from_bottom, angle, telescopic))

    def grab_grape_from_wall(self, continuous=False):
        """
        识别一面墙上的葡萄，然后抓取它们
        :param continuous: 是否连续作业，若为True，则抓取结束后继续调整为识别姿态
        :return: None
        """

        if self.direction is None:
            self.logger.error("方向未设置。无法抓葡萄!")
            return False

        self.extra_move_distance = 0

        self.move.rotation_correction(self.direction, scan_angle=10)
        if self.continuous and continuous:
            self.arm.wait_plan_finish()
        else:
            self.arm.plan_list(ArmMovement.identify_grape(self.direction))
            self.continuous = continuous
        time.sleep(1)

        fruits = self.find_grapes_you_need()
        if not fruits:
            self.logger.warn("没有检测到水果!")
            return False
        else:
            self.logger.info(f"检测到 {len(fruits)} 个水果")

        # 排序，顺序取决于抓取的方向
        fruits.sort(key=lambda fruit: fruit.box.get_rectangle_center().x, reverse=self.direction == Direction.RIGHT)
        for i in fruits:
            if not self.grapes_in_basket(i.class_id):
                self.logger.info(f"已经抓够的的水果，跳过 {i.class_id}")
                continue

            if i.box.get_area() < 4000:
                self.logger.warn(f"面积 {i.box.get_area()} 太小，可能有遮挡或过于遥远，跳过 {i.class_id}")
                continue

            # 调整到合适的位置并抓取
            self.logger.info(f"开始接近合适抓取 {i.class_id} 葡萄位置.")
            self.close_grape(i)
            self.logger.info(f"抓取 {i.class_id} 葡萄.")
            self.grab_grape(i)

            # 在待抓列表中移除框子
            basket_num = self.get_grape_in_which_basket(i.class_id)
            basket = getattr(self, f"basket_{basket_num}")
            basket.remove(FruitType(i.class_id))
            # 放入葡萄
            ArmMovement.put_fruit_to_basket(self.arm, basket_num, True)

        if continuous:
            self.arm.plan_list(ArmMovement.identify_grape(self.direction), block=False)
        else:
            self.arm.plan_list(ArmMovement.motion(), block=False)

        return True
