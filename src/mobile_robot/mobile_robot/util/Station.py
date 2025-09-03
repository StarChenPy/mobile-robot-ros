import enum

import rclpy.node

from .Logger import Logger
from ..param import ArmMovement
from ..popo.Direction import Direction
from ..service.ArmService import ArmService
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService


class Station(enum.Enum):
    YELLOW_1 = enum.auto(), "s_y_1_r", Direction.RIGHT, 1.49, False, "s_y_1_l", Direction.LEFT, 2.11
    YELLOW_2 = enum.auto(), "s_y_2_r", Direction.RIGHT, 1.52, True
    YELLOW_3 = enum.auto(), "s_y_3_l", Direction.LEFT, 1.61, False
    RED_1 = enum.auto(), "s_r_1_l", Direction.LEFT, 1.51, True
    RED_2 = enum.auto(), "s_r_2_r", Direction.RIGHT, 0, False
    RED_3 = enum.auto(), "s_r_3_l", Direction.LEFT, 1.7, False

    def __new__(cls, key, main_waypoint: str, main_direction: Direction, main_revise: float, on_slope: bool,
                sub_waypoint=None, sub_direction: Direction=None, sub_revise: float = 0):
        obj = object.__new__(cls)
        obj._value_ = key
        # 主路径点与方向
        obj.main_waypoint = main_waypoint
        obj.main_direction = main_direction
        # 是否在坡上
        obj.on_slope = on_slope
        # 位置修正(可选)
        obj.main_revise = main_revise
        # 副路径点与方向(可选)
        obj.sub_waypoint = sub_waypoint
        obj.sub_direction = sub_direction
        obj.sub_revise = sub_revise
        return obj

    def nav_and_grab(self, node: rclpy.node.Node, is_sub: bool = False):
        """
        导航至站台并抓取篮子
        :param node: ROS2节点
        :param is_sub: 是否使用副路径点
        :return: None
        """

        self.nav_to_station(node, is_sub)
        self.grab_basket(node, is_sub)

    def nav_and_put(self, node: rclpy.node.Node, basket_num: int, is_sub: bool = False):
        """
        导航至站台并放置篮子
        :param node: ROS2节点
        :param basket_num: 要放置的篮子编号
        :param is_sub: 是否使用副路径点
        :return: None
        """
        arm = ArmService(node)

        self.nav_to_station(node, is_sub)
        ArmMovement.grab_basket_from_robot(arm, basket_num)
        self.put_basket(node, is_sub)

    def nav_to_station(self, node: rclpy.node.Node, is_sub: bool = False):
        """
        导航至站台并进行位置修正

        :param node: ROS2节点
        :param is_sub: 是否使用副路径点
        :return: None
        """

        logger = Logger()
        move = MoveService(node)
        sensor = SensorService(node)

        # 1. 导航至路径点
        if is_sub:
            if self.sub_waypoint and self.sub_direction:
                logger.info(f"前往 {self.name} 站台副路径点 {self.sub_waypoint}.")
                move.my_navigation(self.sub_waypoint)
            else:
                logger.warn(f"未定义 {self.name} 站台副路径点或方向，使用站台主路径点.")
                is_sub = False
                move.my_navigation(self.main_waypoint)
        else:
            logger.info(f"前往 {self.name} 站台主路径点 {self.main_waypoint}.")
            move.my_navigation(self.main_waypoint)

        logger.info(f"{self.name} 站台位置旋转修正.")
        if is_sub:
            move.rotation_correction(self.sub_direction)
        else:
            move.rotation_correction(self.main_direction)

        if is_sub and self.sub_revise:
            logger.info(f"{self.name} 站台副路径点微调修正 {self.sub_revise}.")
            if self.sub_revise > 0:
                sensor.lidar_revise(self.sub_revise)
            else:
                sensor.ping_revise(-self.sub_revise)
        elif not is_sub and self.main_revise:
            logger.info(f"{self.name} 站台主路径点微调修正 {self.main_revise}.")
            if self.main_revise > 0:
                sensor.lidar_revise(self.main_revise)
            else:
                sensor.ping_revise(-self.main_revise)

        logger.info(f"到达站台 {self.name} {'副路径点' if is_sub else '主路径点'}.")

    def grab_basket(self, node: rclpy.node.Node, is_sub: bool = False):
        """
        抓取站台篮子
        :param node: ROS2节点
        :param is_sub: 是否使用副路径点
        :return: None
        """

        logger = Logger()
        arm = ArmService(node)
        sensor = SensorService(node)

        if is_sub:
            if self.sub_waypoint and self.sub_direction:
                logger.info(f"使用 {self.name} 站台副方向 {self.sub_direction} 抓取篮子.")
                direction = self.sub_direction
            else:
                logger.warn(f"未定义 {self.name} 站台副路径点或方向，使用站台主方向抓取篮子.")
                direction = self.main_direction
        else:
            direction = self.main_direction

        distance_from_wall = sensor.get_distance_from_wall(direction)

        ArmMovement.grab_basket_from_station(arm, direction, 15.5 if self.on_slope else 9.5,
                                             (distance_from_wall - 0.25) * 100)

    def put_basket(self, node: rclpy.node.Node, is_sub: bool = False):
        logger = Logger()
        arm = ArmService(node)

        if is_sub:
            if self.sub_waypoint and self.sub_direction:
                logger.info(f"使用 {self.name} 站台副方向 {self.sub_direction} 放置篮子.")
                direction = self.sub_direction
            else:
                logger.warn(f"未定义 {self.name} 站台副路径点或方向，使用站台主方向放置篮子.")
                direction = self.main_direction
        else:
            direction = self.main_direction

        ArmMovement.put_basket_to_station(arm, direction, 15.5 if self.on_slope else 9.5)
