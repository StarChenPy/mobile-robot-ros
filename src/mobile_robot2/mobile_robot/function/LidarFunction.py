import math

from py_trees.composites import Parallel
from py_trees.decorators import Decorator
from py_trees.common import Status, ParallelPolicy
from py_trees.blackboard import Blackboard
from sensor_msgs.msg import LaserScan

from ..model.Direction import Direction
from ..ros_client.TopicSubscriber import lidar_data_sub
from ..util import Math


RADAR_ERROR = 2.4


def get_start_angle(direction):
    if direction == Direction.RIGHT:
        start_angle = 0
    elif direction == Direction.FRONT:
        start_angle = 80
    elif direction == Direction.LEFT:
        start_angle = 155
    else:
        raise ValueError(f"方向必须是 RIGHT 或 FRONT 或 LEFT，但传入的是{direction}")

    return start_angle


class GetLidarPointFunction(Decorator):
    def __init__(self, angle: float):
        super().__init__("Lidar Function", lidar_data_sub)
        self.angle = angle

    def update(self) -> Status:
        """
        获取雷达与某物体在某角度下的距离
        """
        if self.decorated.status == Status.SUCCESS:
            radar_data: LaserScan = Blackboard.get("lidar_data/raw")

            # 转换目标角度为弧度
            target_angle_rad = math.radians(self.angle)
            angle_min = radar_data.angle_min
            angle_increment = radar_data.angle_increment

            best_range = None
            best_index = None
            best_diff = float('inf')  # 初始设为无穷大

            for i, range_value in enumerate(radar_data.ranges):
                # 跳过无效测量（0表示无效）
                if range_value == 0:
                    continue

                # 计算当前数据点的角度
                current_angle = angle_min + i * angle_increment
                # 计算与目标角度的差值
                diff = abs(current_angle - target_angle_rad)
                # 如果当前角度更接近目标角度，则更新
                if diff < best_diff:
                    best_diff = diff
                    best_range = range_value
                    best_index = i

            # 如果没有找到有效数据，则失败
            if best_index is None:
                return Status.FAILURE

            best_angle = angle_min + best_index * angle_increment
            Blackboard.set(f"lidar_data/by_angle/{self.angle}", (best_range, math.degrees(best_angle)))

        return self.decorated.status


class GetLidarPointsFunction(Parallel):
    def __init__(self, direction: Direction, number_of_samples=5):
        super().__init__("Get Lidar Points Function", ParallelPolicy.SuccessOnAll())
        self.number_of_samples = number_of_samples
        self.direction = direction

        for i in range(0, self.number_of_samples):
            angle = get_start_angle(direction) + 5 * i
            self.add_child(GetLidarPointFunction(angle))

    def update(self) -> Status:
        points = []
        for i in range(0, self.number_of_samples):
            angle = get_start_angle(self.direction) + 5 * i
            points.append(Blackboard.get(f"lidar_data/by_angle/{angle}"))

        Blackboard.set(f"lidar_data/by_direction/{self.direction.value}", points)

        return Status.SUCCESS


class GetAngleFunction(Decorator):
    def __init__(self, direction: Direction):
        super().__init__("Get Once Angle From Wall Function", GetLidarPointsFunction(direction))
        self.direction = direction

    def update(self) -> Status:
        if self.decorated.status == Status.SUCCESS:
            points = Blackboard.get(f"lidar_data/by_direction/{self.direction.value}")
            angle = Math.fit_polar_line_and_get_angle(points)

            if self.direction == Direction.FRONT:
                pass
            else:
                if angle < 0:
                    angle += 90
                else:
                    angle -= 90

            if abs(angle) > 30:
                self.logger.warn(f"异常角度 {angle}, 不可信")
                return Status.FAILURE

            Blackboard.set(f"lidar_data/by_direction/{self.direction.value}/angle", angle)

        return self.decorated.status

class GetDistanceFunction(Decorator):
    """
    通过激光雷达获取距离数据，同时也更新了角度数据
    """

    def __init__(self, direction: Direction):
        super().__init__("Get Once Distance From Wall Function", GetAngleFunction(direction))
        self.direction = direction

    def update(self) -> Status:
        if self.decorated.status == Status.SUCCESS:
            points = Blackboard.get(f"lidar_data/by_direction/{self.direction.value}")
            distance = Math.fit_polar_line_and_get_distance(points)

            if self.direction == Direction.FRONT:
                # 加上从雷达到机器人中心的距离
                distance += 0.225
            else:
                # 补偿因倾斜导致的雷达与墙和机器人中心与墙的距离不一致的问题3
                angle_from_wall = Blackboard.get(f"lidar_data/by_direction/{self.direction.value}/angle")
                if angle_from_wall == 0:
                    self.logger.warn("雷达距离数据无效")
                    return Status.FAILURE

                side = Math.calculate_right_angle_side(0.225, abs(angle_from_wall))

                if self.direction == Direction.LEFT:
                    side = -side

                if angle_from_wall > 0:
                    distance += side
                else:
                    distance -= side

            Blackboard.set(f"lidar_data/by_direction/{self.direction.value}/distance", distance)
            return Status.SUCCESS

        return self.decorated.status