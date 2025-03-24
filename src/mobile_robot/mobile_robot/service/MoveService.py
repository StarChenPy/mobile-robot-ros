import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.MotionDao import MotionDao
from ..dao.NavigationDao import NavigationDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..util import Math
from ..util.Singleton import singleton


@singleton
class MoveService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__navigation = NavigationDao(node)
        self.__motion = MotionDao(node)
        self.__sensor = SensorDao(node)
        self.__odom = OdomDao(node)
        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)

    def __navigation_handle(self, path: list[NavigationPoint], speed: float, is_block: bool):
        ROTATION_ACCELERATION = 3
        ROTATION_DECELERATION = 3
        self.__navigation.navigation(path, speed, speed * 5, ROTATION_ACCELERATION, ROTATION_DECELERATION, False)
        if is_block:
            self.__navigation.wait_finish()

    def navigation(self, nav_path: list[NavigationPoint], speed = 0.4, is_block=True):
        """
        通过路径进行导航
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        """
        path = []
        previous_point = None

        for point in nav_path:
            if isinstance(point, CorrectivePoint):
                if path:
                    path.append(point)
                    self.__navigation_handle(path, speed, True)
                elif self.__odom.get_init():
                    self.__navigation_handle([point], speed, True)
                else:
                    self.__odom.init_yaw(point.yaw)

                self.corrective(point)
                path = []
                continue

            if previous_point is None:
                odom = self.__robot_data.get_robot_data().odom
                previous_point = NavigationPoint(odom.x, odom.y, odom.w)

            if previous_point.yaw is not None and Math.is_behind(previous_point, point, 45):
                # 如果这个点位在上个点位的后面，就倒车回去
                if path:
                    self.__navigation_handle(path, speed, True)
                    path = []
                if point.yaw is None:
                    point.yaw = previous_point.yaw
                self.__navigation.navigation([point], speed, speed * 5, 3, 3, True)
                self.__navigation.wait_finish()
            else:
                path.append(point)

            previous_point = point

        if path:
            self.__navigation_handle(path, speed, is_block)

    def corrective(self, point: CorrectivePoint):
        x_buffer = 0
        y_buffer = 0
        angle_from_wall = 0
        for corrective in point.corrective_data:
            match corrective.direction:
                case Direction.FRONT:
                    distance_from_wall = self.__radar.get_distance_from_wall(corrective.direction)
                    angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                    x_buffer = distance_from_wall - corrective.distance
                case Direction.BACK:
                    sonar = self.__robot_data.get_sonar()
                    distance_from_wall = Math.distance_from_origin(-5, sonar[0], 5, sonar[1]) + 0.222
                    x_buffer = distance_from_wall - corrective.distance
                case Direction.LEFT:
                    distance_from_wall = self.__radar.get_distance_from_wall(corrective.direction)
                    angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                    y_buffer = distance_from_wall - corrective.distance
                case Direction.RIGHT:
                    distance_from_wall = self.__radar.get_distance_from_wall(corrective.direction)
                    angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                    y_buffer = distance_from_wall - corrective.distance

        if angle_from_wall != 0:
            new_yaw = point.yaw - angle_from_wall
            abs1 = abs(self.__robot_data.get_robot_data().odom.w - new_yaw)
            # 陀螺仪不会歪那么多，角度超过10就是不可信的数据
            if abs1 > 170:
                self.__logger.warn("[MoveService] 矫正角度与陀螺仪误差超过170度, 可能是180度分界线.")
            elif abs1 > 15:
                self.__logger.warn(f"[MoveService] 矫正角度与陀螺仪误差超过15度，不可信数据。检测角度：{new_yaw}")
                return
            self.__odom.init_yaw(new_yaw)

        if point.yaw == 0:
            # 验证可用
            self.__odom.init_location(point.x + x_buffer, point.y + y_buffer)
        elif point.yaw == 90:
            # 验证可用
            self.__odom.init_location(point.x + y_buffer, point.y + x_buffer)
        elif point.yaw == 180 or point.yaw == -180:
            # 未验证可用
            self.__odom.init_location(point.x - x_buffer, point.y - y_buffer)
        elif point.yaw == -90:
            # 验证可用
            self.__odom.init_location(point.x + y_buffer, point.y - x_buffer)

        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)
        rclpy.spin_once(self.__node)

    def line(self, distance: float, speed: float = 0.4, is_block=True):
        self.__motion.line(distance, speed)

        if is_block:
            self.__motion.wait_finish()

    def rotate(self, angle: float, speed: float = 50, is_block=True):
        self.__motion.rotate(angle, speed)

        if is_block:
            self.__motion.wait_finish()

    def get_status(self):
        return self.__navigation.get_status()

    def stop_motion(self):
        self.__motion.stop()

    def stop_navigation(self):
        self.__navigation.cancel()
