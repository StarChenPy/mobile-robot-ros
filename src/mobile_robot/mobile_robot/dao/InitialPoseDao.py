import math

import rclpy

from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation

from ..popo.NavigationPoint import NavigationPoint
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class InitialPoseDao(object):
    def __init__(self, node: rclpy.node.Node):
        self.init = False
        self.node = node
        self.logger = Logger()

        self.__topic = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def get_init(self):
        return self.init

    def set_init(self, init: bool):
        self.init = init

    def set_initial_pose(self, pose: NavigationPoint):
        """
        设置初始位置
        @param pose 初始位置
        """
        self.logger.info(f"设置初始位置: {pose}")

        pose_data = PoseWithCovarianceStamped()

        pose_data.header.frame_id = 'map'
        pose_data.header.stamp = self.node.get_clock().now().to_msg()
        pose_data.pose.pose.position.x = pose.x
        pose_data.pose.pose.position.y = pose.y
        pose_data.pose.pose.position.z = 0.0

        # 将航向角转换为四元数
        quat = Rotation.from_euler('z', pose.yaw, degrees=True).as_quat()
        pose_data.pose.pose.orientation.x = quat[0]
        pose_data.pose.pose.orientation.y = quat[1]
        pose_data.pose.pose.orientation.z = quat[2]
        pose_data.pose.pose.orientation.w = quat[3]

        # 协方差（根据需要可以调大或调小）
        # 只需关心第 0 位（x 方向方差）第 7 位（y 方向方差）第 35 位（yaw 方向方差）
        pose_data.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, math.radians(10) ** 2
        ]

        self.__topic.publish(pose_data)
        rclpy.spin_once(self.node)

        self.set_init(True)
