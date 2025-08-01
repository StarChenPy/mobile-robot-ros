import rclpy.node

from typing import List
from . import Math
from .Logger import Logger
from .Singleton import singleton
from ..param import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.NavigationPoint import NavigationPoint
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.VisionService import VisionService


def create_tree_path(tree_point: NavigationPoint, tree_dir: list[Direction]) -> list[tuple[NavigationPoint, Direction]]:
    """
    根据树的位置和方向来计算识别点坐标点
    """

    identify_distance = 0.57
    identify_offset = 0.34
    min_x, max_x = 0.0, 4.0
    min_y, max_y = 0.0, 4.0

    direction_deltas = {
        Direction.LEFT:   (0,  identify_distance),
        Direction.RIGHT:  (0, -identify_distance),
        Direction.FRONT:  (identify_distance, 0),
        Direction.BACK:  (-identify_distance, 0),
    }

    offset_map = {
        (Direction.LEFT,  Direction.FRONT):  (identify_offset, 0, 180, Direction.LEFT),
        (Direction.LEFT,  Direction.BACK):  (-identify_offset, 0, 0, Direction.RIGHT),
        (Direction.RIGHT, Direction.FRONT):  (identify_offset, 0, 180, Direction.RIGHT),
        (Direction.RIGHT, Direction.BACK):  (-identify_offset, 0, 0, Direction.LEFT),
        (Direction.FRONT, Direction.LEFT):   (0, identify_offset, -90, Direction.RIGHT),
        (Direction.FRONT, Direction.RIGHT):  (0, -identify_offset, 90, Direction.LEFT),
        (Direction.BACK,  Direction.LEFT):   (0, identify_offset, -90, Direction.LEFT),
        (Direction.BACK,  Direction.RIGHT):  (0, -identify_offset, 90, Direction.RIGHT),
    }

    nav_points = []

    for i in tree_dir:
        dx, dy = direction_deltas[i]
        x = tree_point.x + dx
        y = tree_point.y + dy

        if not (min_x <= x <= max_x):
            raise RuntimeError(f"{i} 无法推出路径点，x超出边界: {x}")
        if not (min_y <= y <= max_y):
            raise RuntimeError(f"{i} 无法推出路径点，y超出边界: {y}")

        # 找到匹配的 offset 方向组合
        for j in tree_dir:
            if i == j:
                continue
            key = (i, j)
            if key in offset_map:
                odx, ody, yaw, identify_dir = offset_map[key]
                x2 = x + odx
                y2 = y + ody

                if not (min_x <= x2 <= max_x):
                    raise RuntimeError(f"{i}->{j} 无法推出路径点，x超出边界: {x2}")
                if not (min_y <= y2 <= max_y):
                    raise RuntimeError(f"{i}->{j} 无法推出路径点，y超出边界: {y2}")

                nav_points.append((NavigationPoint(x2, y2, yaw), identify_dir))
                break
        else:
            raise ValueError(f"{i} 无法推出路径点, 需要相邻的方向来推算!")

    return nav_points


@singleton
class GrabAppleTree:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()
        self.node = node

        self.direction = None
        self.max_move_distance = 0.68
        self.max_grab_distance = 0.46
        self.min_grab_distance = 0.27

        self.basket_1 = []
        self.basket_2 = []
        self.basket_3 = []

        self.move = MoveService(node)
        self.arm = ArmService(node)
        self.vision = VisionService(node)

    def has_apple(self):
        apple_types = set(FruitType.apples())
        for i in range(1, 4):
            basket = getattr(self, f"basket_{i}")
            if basket and any(fruit in apple_types for fruit in basket):
                return True
        return False

    def find_fruits(self, fruit=None) -> List[IdentifyResult]:
        identify = self.vision.get_onnx_identify_depth()

        result = []
        for i in identify:
            if fruit:
                if FruitType(i.class_id) not in fruit:
                    continue
            if i.distance > self.max_move_distance:
                continue
            result.append(i)
        return result

    def close_tree(self, distance: float):
        if not self.direction:
            raise RuntimeError("没有方向!")

        self.logger.info(f"向果树靠近 {distance} 米")

        """靠近果树"""
        ArmMovement.motion(self.arm)
        self.move.rotate(90)
        if self.direction == Direction.LEFT:
            self.move.line(distance)
        if self.direction == Direction.RIGHT:
            self.move.line(-distance)
        self.move.rotate(-90)

    def grab_apple_from_tree(self):
        if not self.direction:
            raise RuntimeError("没有方向!")

        ArmMovement.identify_tree_fruit(self.arm, self.direction)
        fruits = self.find_fruits(self.basket_1 + self.basket_2 + self.basket_3)
        prev_move_len = 0

        if not fruits:
            self.logger.warn("没有检测到水果!")
            ArmMovement.motion(self.arm)
            return False

        # 靠近果树
        min_distance = min(self.find_fruits(), key=lambda fruit: fruit.distance).distance
        max_distance = max(self.find_fruits(), key=lambda fruit: fruit.distance).distance
        piff_dis = 0
        if max_distance > self.max_grab_distance:
            piff_dis = min_distance - self.min_grab_distance
            # piff_dis = max_distance - self.max_grab_distance
            self.close_tree(piff_dis)

        fruits.sort(key=lambda fruit: fruit.box.get_rectangle_center().x)
        for i in fruits:
            if i.distance == -1:
                self.logger.warn(f"没有深度信息，跳过{i.class_id}")
                continue

            if i.box.get_area() < 2000:
                self.logger.warn(f"面积太小，可能有遮挡，跳过{i.class_id}")
                self.logger.warn(f"面积 {i.box.get_area()}")

            if max_distance - i.distance > 18:
                self.logger.warn(f"与可见的最大深度差值过大，不可抓，跳过{i.class_id}")
                continue

            center = i.box.get_rectangle_center()
            fruit_type = FruitType(i.class_id)

            distance = 0.3
            if i.distance != -1:
                distance = i.distance + 0.04
                self.logger.info(f"抓取苹果使用深度信息 {distance}")
            else:
                self.logger.warn(f"深度相机没有深度信息，使用默认值 {distance}")

            if self.direction == Direction.LEFT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(center.x - 320, distance)
            elif self.direction == Direction.RIGHT:
                move_distance = Math.pixel_to_horizontal_distance_x_centered(320 - center.x, distance)
            else:
                raise ValueError()
            move_distance = move_distance + 0.215

            self.arm.lift(0)
            ArmMovement.open_half_gripper(self.arm)
            self.move.line(move_distance - prev_move_len)
            prev_move_len = move_distance

            ArmMovement.grab_apple_on_tree(self.arm, self.direction, (distance - 0.34 - piff_dis) * 100, center.y > 220)
            for i in range(1, 4):
                basket = getattr(self, f"basket_{i}")
                if fruit_type in basket:
                    ArmMovement.put_fruit_to_basket(self.arm, i)
                    basket.remove(fruit_type)
                    break

        ArmMovement.motion(self.arm)
        return True

    def grab_tree(self, tree_point: NavigationPoint, tree_dir: list[Direction]):
        path = create_tree_path(tree_point, tree_dir)

        for p, d in path:
            self.move.navigation([p])
            self.direction = d
            self.grab_apple_from_tree()
