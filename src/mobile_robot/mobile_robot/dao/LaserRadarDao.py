import math
import rclpy
import rclpy.qos

from sensor_msgs.msg import LaserScan

from ..util.Singleton import singleton


@singleton
class LaserRadarDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()
        self.__radar_data = LaserScan()

        qos_profile = rclpy.qos.QoSProfile(reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,depth = 10)
        node.create_subscription(LaserScan, '/scan', self.__callback, qos_profile)

    def __callback(self, msg: LaserScan):
        self.__radar_data = msg

    def get_radar_data(self, target_angle: float) -> float:
        """
        获取雷达与某物体在某角度下的距离
        @param target_angle: 目标角度
        @return 距离
        """
        rclpy.spin_once(self.__node)
        # 检查雷达数据是否存在
        if self.__radar_data is None:
            return 0

        # 转换目标角度为弧度
        target_angle_rad = math.radians(target_angle)
        angle_min = self.__radar_data.angle_min
        angle_increment = self.__radar_data.angle_increment

        best_range = None
        best_index = None
        best_diff = float('inf')  # 初始设为无穷大

        for i, range_value in enumerate(self.__radar_data.ranges):
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

        # 如果没有找到有效数据，则返回0
        if best_index is None:
            return 0

        best_angle = angle_min + best_index * angle_increment
        self.__logger.debug(f"[激光雷达] 距离 {best_range} 米，对应 {math.degrees(best_angle)} 度")
        return best_range
