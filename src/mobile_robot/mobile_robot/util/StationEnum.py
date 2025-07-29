import rclpy.node

import enum
from ..popo.Direction import Direction
from ..service.MoveService import MoveService
from ..service.SensorService import SensorService


class Station(enum.Enum):
    YELLOW_STATION_1 = -0.89
    YELLOW_STATION_2 = 1.11
    YELLOW_STATION_3 = -0.89
    RED_STATION_1 = -0.89
    RED_STATION_2 = 1.11
    RED_STATION_3 = -0.89

    def revise(self, node: rclpy.node.Node):
        sensor = SensorService(node)
        move = MoveService(node)

        move.rotation_correction()

        if self.value > 0:
            from_wall = sensor.get_distance_from_wall(Direction.FRONT)
            i = from_wall - self.value
            while abs(i) > 0.3:
                move.line(i)
                from_wall = sensor.get_distance_from_wall(Direction.FRONT)
                i = from_wall - self.value
        else:
            sensor.ping_revise(-self.value)