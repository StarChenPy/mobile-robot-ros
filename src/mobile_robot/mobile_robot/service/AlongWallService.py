import threading
import time

import numpy
import rclpy

from ..dao.LaserRadarDao import LaserRadarDao
from ..dao.NavigationDao import NavigationDao
from ..dao.OdomDao import OdomDao
from ..dao.RobotDataDao import RobotDataDao
from ..popo.NavigationPoint import NavigationPoint
from ..util.Math import Math
from ..util.Singleton import singleton


@singleton
class AlongWallService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__radar = LaserRadarDao(node)
        self.__robot_data = RobotDataDao(node)
        self.__navigation = NavigationDao(node)
        self.__odom = OdomDao(node)

        self.__navigation_points: list[NavigationPoint] = []
        self.__odom_backup = None
        self.__walking_along_wall = False

    def __update_odom_thread(self, radar_angle: int, radar_multiplier: int = 1):
        # 导航过程中不断更新位置：利用雷达数据与当前 odom 数据进行融合更新
        while rclpy.ok() and self.__walking_along_wall:
            radar_data = radar_multiplier * self.__radar.get_radar_data(radar_angle)
            odom_data = self.__robot_data.get_robot_data().odom
            self.__odom.init_location(odom_data.x, radar_data)
            time.sleep(1)

    def __navigation_thread(self, speed):
        travel_distance = self.__navigation_points[-1].x

        # 执行导航点
        for point in self.__navigation_points[:]:
            self.__navigation.navigation([point], speed, speed * 4)
            self.__navigation.wait_finish()
            self.__navigation_points.remove(point)

            if not self.__walking_along_wall:
                return

        # 根据备份的 odom 计算最终目标点：此处 Math.get_target_coordinate 根据初始位置与行驶距离计算出新的坐标，
        new_point = Math.get_target_coordinate(self.__odom_backup, travel_distance)
        self.__odom.init_all(new_point)
        self.__walking_along_wall = False

    def __along_wall(self, travel_distance: float, distance_from_wall: float, speed, radar_angle: int, radar_multiplier: int = 1):
        """
        沿指定方向墙体行驶的通用方法
        :param travel_distance: 行驶总距离
        :param distance_from_wall: 与墙壁的距离（导航点中的 y 坐标）
        :param speed: 运动速度
        :param radar_angle: 获取雷达数据时使用的角度
        :param radar_multiplier: 雷达数据的符号修正（左墙为 1，右墙为 -1）
        """
        # 备份当前位置
        odom = self.__robot_data.get_robot_data().odom
        self.__odom_backup = NavigationPoint(odom.x, odom.y, odom.w)

        # 初始化时根据指定角度读取雷达数据，并乘以修正因子
        radar_data = radar_multiplier * self.__radar.get_radar_data(radar_angle)
        self.__odom.init_all(NavigationPoint(0, radar_data, 0))

        # 生成 10 个导航点构成的路径
        start = travel_distance / 10
        self.__navigation_points = [NavigationPoint(d, distance_from_wall, 0) for d in numpy.linspace(start, travel_distance, 10)]

        self.__walking_along_wall = True
        threading.Thread(target=self.__navigation_thread, args=(speed, ), daemon=True).start()
        threading.Thread(target=self.__update_odom_thread, args=(radar_angle, radar_multiplier), daemon=True).start()

    def along_left_wall(self, travel_distance: float, distance_from_wall: float, speed, is_block=False):
        """
        沿左墙行驶：使用雷达角度 180 度，不需要反向修正
        """
        self.__along_wall(travel_distance, distance_from_wall, speed, 180, 1)
        if is_block:
            self.wait_finish()

    def along_right_wall(self, travel_distance: float, distance_from_wall: float, speed, is_block=False):
        """
        沿右墙行驶：使用雷达角度 0 度，同时对雷达数据取反以满足方向需求
        """
        self.__along_wall(travel_distance, distance_from_wall, speed, 0, -1)
        if is_block:
            self.wait_finish()

    def wait_finish(self):
        while self.__walking_along_wall:
            pass

    def stop(self):
        self.__walking_along_wall = False
        self.__navigation.cancel()
        new_point = Math.get_target_coordinate(self.__odom_backup, self.__navigation_points[0].x)
        self.__odom.init_all(new_point)
