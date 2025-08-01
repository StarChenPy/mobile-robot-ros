import time

import rclpy.node

import enum
from . import Math
from ..param import NavMovement
from ..popo.Direction import Direction
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService


class Station(bytes, enum.Enum):
    YELLOW_STATION_1 = (enum.auto(), NavMovement.YELLOW_STATION_1, 1)
    YELLOW_STATION_2 = (enum.auto(), NavMovement.YELLOW_STATION_2, 1)
    YELLOW_STATION_3 = (enum.auto(), NavMovement.YELLOW_STATION_3, 1)
    RED_STATION_1 = (enum.auto(), NavMovement.RED_STATION_1, 1)
    RED_STATION_2 = (enum.auto(), NavMovement.RED_STATION_2, 1)
    RED_STATION_3 = (enum.auto(), NavMovement.RED_STATION_3, 1)

    def __new__(cls, value, navigation_point, revise_distance):
        obj = bytes.__new__(cls, value)
        obj._value_ = value
        obj.navigation_point = navigation_point
        obj.revise_distance = revise_distance
        return obj

    def revise(self, node: rclpy.node.Node, direction: Direction):
        sensor = SensorService(node)
        move = MoveService(node)

        move.rotation_correction()

        if direction == Direction.FRONT:
            from_wall = sensor.get_distance_from_wall(direction)
            i = from_wall - (self.revise_distance + 0.11)
            while abs(i) > 0.05:
                move.line(i)
                time.sleep(0.2)
                from_wall = sensor.get_distance_from_wall(Direction.FRONT)
                i = from_wall - (self.revise_distance + 0.11)
        elif direction == Direction.BACK:
            sensor.ping_revise(self.revise_distance - 0.11)

    def nav_point(self, direction: Direction):
        if direction == Direction.LEFT:
            return Math.get_target_coordinate(self.navigation_point, -0.11)
        elif direction == Direction.RIGHT:
            point = Math.get_target_coordinate(self.navigation_point, 0.11)
            point.yaw = (point.yaw + 180) % 360
            if point.yaw > 180:
                point.yaw -= 360
            return point
        else:
            raise ValueError("不支持的方向")
