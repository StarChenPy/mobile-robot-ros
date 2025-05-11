import math

from py_trees.blackboard import Blackboard
from py_trees.common import Status
from py_trees.composites import Sequence

from chassis_msgs.srv import ResetOdom
from web_message_transform_ros2.msg import RobotData
from ..function.InitOdomFunction import InitAllFunction
from ..function.LidarFunction import GetDistanceFunction
from ..model.CorrectivePoint import CorrectivePoint
from ..model.Direction import Direction
from ..ros_client.TopicSubscriber import robot_data_sub
from ..util import Math


class GetSensorDataFunction(Sequence):
    def __init__(self, point: CorrectivePoint):
        super().__init__("Get Corrective Data Service", True)
        self.add_child(robot_data_sub)
        for corrective in point.corrective_data:
            self.add_child(GetDistanceFunction(corrective.direction))
        self.point = point

    def update(self) -> Status:
        robot_data: RobotData = Blackboard.get("robot_data/raw")

        yaw = robot_data.odom.w
        x = robot_data.odom.x
        y = robot_data.odom.y

        x_buffer = 0
        y_buffer = 0
        angle_from_wall = 0

        for corrective in self.point.corrective_data:
            direction = corrective.direction

            if direction == Direction.FRONT:
                distance_from_wall = Blackboard.get(f"lidar_data/by_direction/{direction.value}/distance")
                angle_from_wall = Blackboard.get(f"lidar_data/by_direction/{direction.value}/angle")
                x_buffer = distance_from_wall - corrective.distance
            elif direction == Direction.BACK:
                distance_from_wall = Math.distance_from_origin(-5, robot_data.sonar[0], 5, robot_data.sonar[1]) + 0.222
                x_buffer = distance_from_wall - corrective.distance
            elif direction == Direction.LEFT or direction == Direction.RIGHT:
                distance_from_wall = Blackboard.get(f"lidar_data/by_direction/{direction.value}/distance")
                angle_from_wall = Blackboard.get(f"lidar_data/by_direction/{direction.value}/angle")
                y_buffer = distance_from_wall - corrective.distance

        if angle_from_wall != 0:
            new_yaw = self.point.yaw - angle_from_wall
            abs1 = abs(yaw - new_yaw)
            # 陀螺仪不会歪那么多，角度超过15就是不可信的数据
            if abs1 > 300:
                self.logger.warn("矫正角度与陀螺仪误差超过300度, 可能是180度分界线.")
            elif abs1 > 15:
                self.logger.warn(f"矫正角度与陀螺仪误差超过15度，不可信数据。陀螺仪角度: {yaw}, 测量角度: {new_yaw}")
            else:
                yaw = new_yaw

        if abs(self.point.yaw) < 5:
            x = self.point.x + x_buffer
            y = self.point.y + y_buffer
        elif abs(90 - self.point.yaw) < 5:
            x = self.point.x + y_buffer
            y = self.point.y + x_buffer
        elif abs(180 - self.point.yaw) < 5 or abs(180 - self.point.yaw) < 5:
            x = self.point.x - x_buffer
            y = self.point.y - x_buffer
        elif abs(-90 - self.point.yaw) < 5:
            x = self.point.x + y_buffer
            y = self.point.y - x_buffer

        req = ResetOdom.Request()
        req.clear_mode = 0
        req.x = float(x)
        req.y = float(y)
        radian = math.radians(yaw)
        req.theta = float(radian)

        Blackboard.set("odom/request", req)

        return Status.SUCCESS


class CorrectiveOdomFunction(Sequence):
    def __init__(self, point: CorrectivePoint):
        super().__init__("Corrective Odom Service", False)
        self.add_children([
            GetSensorDataFunction(point),
            InitAllFunction
        ])
