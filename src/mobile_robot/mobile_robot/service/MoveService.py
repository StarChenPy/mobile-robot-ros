import time

import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.MotionDao import MotionDao
from ..dao.NavigationPtpDao import NavigationPtpDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..dao.SensorDao import SensorDao
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint
from ..util import Math
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class MoveService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.navigation_ptp = NavigationPtpDao(node)
        self.__motion = MotionDao(node)
        self.__sensor = SensorDao(node)
        self.__odom = OdomDao(node)
        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)

    def __navigation_handle(self, path: list[NavigationPoint], speed: float, is_block: bool):
        self.navigation_ptp.navigation(path, speed, speed * 5, False)
        if is_block:
            self.navigation_ptp.wait_finish()

    def rotation_correction(self):
        angle_by_front = self.__radar.get_angle_from_wall(Direction.FRONT)
        angle_by_right = self.__radar.get_angle_from_wall(Direction.RIGHT)
        angle_by_left = self.__radar.get_angle_from_wall(Direction.LEFT)
        angles = [angle_by_front, angle_by_right, angle_by_left]

        min_angle = min(angles, key=lambda x: (abs(x), -x))
        if abs(min_angle) > 1:
            self.rotate(min_angle)

    def navigation(self, nav_path: list[NavigationPoint], speed=0.63, is_block=True, corrective=True):
        """
        通过路径进行导航
        若目标点为矫正点且开启矫正功能，则在前往矫正点时强制阻塞
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        @param corrective 是否启用矫正
        """
        path = []
        previous_point = None

        if not nav_path:
            self.__logger.warn("导航为空路径")
            return

        for point in nav_path:
            if isinstance(point, CorrectivePoint) and corrective:
                # 如果是矫正点，先执行完已有的导航
                if path:
                    path.append(point)
                    self.__navigation_handle(path, speed, True)
                # 如果已经初始化过坐标，就直接往这个点走
                elif self.__odom.get_init():
                    self.__navigation_handle([point], speed, True)
                # 否则就初始化一下角度
                else:
                    self.__odom.init_yaw(point.yaw)
                # 然后矫正当前坐标
                self.corrective(point)
                path = []
                previous_point = point
                continue

            if previous_point is None:
                odom = self.__robot_data.get_robot_data().odom
                previous_point = NavigationPoint(odom.x, odom.y, odom.w)

            # 如果这个点位在上个点位的后面，就倒车回去
            if Math.is_behind(previous_point, point, 80):
                print(f"触发倒车了，分别是 {previous_point} 与 {point}")
                # 先清空导航
                if path:
                    self.__navigation_handle(path, speed, True)
                    path = []
                if point.yaw is None:
                    point.yaw = previous_point.yaw
                self.navigation_ptp.navigation([point], speed, speed * 5,  True)
                self.navigation_ptp.wait_finish()
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
                    if distance_from_wall:
                        angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                        x_buffer = distance_from_wall - corrective.distance
                case Direction.BACK:
                    sonar = self.__robot_data.get_sonar()
                    distance_from_wall = Math.distance_from_origin(-5, sonar[0], 5, sonar[1]) + 0.222
                    x_buffer = distance_from_wall - corrective.distance
                case Direction.LEFT:
                    distance_from_wall = self.__radar.get_distance_from_wall(corrective.direction)
                    if distance_from_wall:
                        angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                        y_buffer = corrective.distance - distance_from_wall
                case Direction.RIGHT:
                    distance_from_wall = self.__radar.get_distance_from_wall(corrective.direction)
                    if distance_from_wall:
                        angle_from_wall = self.__radar.get_angle_from_wall(corrective.direction)
                        y_buffer = distance_from_wall - corrective.distance

        if angle_from_wall != 0:
            new_yaw = point.yaw - angle_from_wall
            if new_yaw > 180:
                new_yaw -= 360
            if new_yaw < -180:
                new_yaw += 360
            odom_w = self.__robot_data.get_robot_data().odom.w
            abs1 = abs(odom_w - new_yaw)
            # 陀螺仪不会歪那么多，角度超过10就是不可信的数据
            if abs1 > 300:
                self.__logger.warn("矫正角度与陀螺仪误差超过300度, 可能是180度分界线.")
            elif abs1 > 30:
                self.__logger.warn(f"矫正角度与陀螺仪误差超过30度，不可信数据。陀螺仪角度: {odom_w}, 测量角度: {new_yaw}")
                return
            self.__logger.info(f"矫正当前角度为: {new_yaw}")
            self.__odom.init_yaw(new_yaw)
            self.__odom.init_yaw(new_yaw)

        x, y = 0, 0
        if abs(point.yaw) < 5:
            x = point.x + x_buffer
            y = point.y + y_buffer
        elif abs(90 - point.yaw) < 5:
            x = point.x - y_buffer
            y = point.y + x_buffer
        elif abs(180 - point.yaw) < 5 or abs(180 - point.yaw) < 5:
            x = point.x - x_buffer
            y = point.y - y_buffer
        elif abs(-90 - point.yaw) < 5:
            x = point.x + y_buffer
            y = point.y - x_buffer

        if x and y:
            self.__logger.info(f"矫正当前坐标为: x: {x}, y: {y}")
            self.__odom.init_location(x, y)
        else:
            self.__logger.warn("矫正失败")

        rclpy.spin_once(self.__node)

    def line(self, distance: float, speed: float = 0.2, is_block=True):
        self.__motion.line(distance, speed)
        time.sleep(1)

        if is_block:
            self.__motion.wait_finish()

    def rotate(self, angle: float, speed: float = 50, is_block=True):
        self.__motion.rotate(angle, speed)
        time.sleep(1)

        if is_block:
            self.__motion.wait_finish()

    def get_status(self):
        return self.navigation_ptp.get_status()

    def stop_motion(self):
        self.__motion.stop()

    def stop_navigation(self):
        self.navigation_ptp.cancel()
