import math

import rclpy

import chassis_msgs.srv
from ..popo.NavigationPoint import NavigationPoint
from ..popo.ResetOdomMode import ResetOdomMode
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class OdomDao:
    def __init__(self, node: rclpy.node.Node):
        self.__init = False
        self.__node = node
        self.__logger = Logger()

        self.__service = node.create_client(chassis_msgs.srv.ResetOdom, '/chassis/reset_odom')

    def __call_service(self, pose: NavigationPoint, mode: ResetOdomMode):
        """初始化机器人位置，支持重置odom不同模式"""
        self.__logger.debug(f"初始化机器人位置 [{pose.x}, {pose.y}, {pose.yaw}] 模式为 {mode.name}")

        self.__service.wait_for_service()

        req = chassis_msgs.srv.ResetOdom.Request()
        req.clear_mode = mode.value
        req.x = float(pose.x)
        req.y = float(pose.y)
        radian = math.radians(pose.yaw)
        req.theta = float(radian)

        future = self.__service.call_async(req)
        self.__logger.debug("已请求服务")

        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=15.0)

        if not future.done():
            self.__logger.error("请求超时未响应")
            return

        result = future.result()
        if result.success:
            self.__logger.debug("重置成功")
        else:
            self.__logger.error("重置失败")

    def init_all(self, pose: NavigationPoint):
        self.__init = True
        self.__call_service(pose, ResetOdomMode.RESET_ALL)

    def init_location(self, x, y):
        self.__init = True
        self.__call_service(NavigationPoint(x, y, 0), ResetOdomMode.RESET_POSE)

    def init_yaw(self, yaw: float):
        self.__init = True
        self.__call_service(NavigationPoint(0, 0, yaw), ResetOdomMode.RESET_YAW)

    def get_init(self):
        return self.__init

    def set_init(self, init: bool):
        self.__init = init
