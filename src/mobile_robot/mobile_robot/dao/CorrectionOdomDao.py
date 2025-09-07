import time

import rclpy

from corn_robot_interfaces.srv import CorrectionOdom
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class CorrectionOdomDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__service = node.create_client(CorrectionOdom, '/correction_odom')
        self.future = None

    def send_correction_odom(self, waypoint_name: str):
        self.__logger.info(f"请求矫正: {waypoint_name}")
        self.future = self.__service.call_async(CorrectionOdom.Request(waypoint_name=waypoint_name))

    def wait_finish(self):
        if not self.future:
            self.__logger.warn("没有发起的矫正请求")
            return

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not self.future.done():
                time.sleep(0.2)
                continue

            if self.future.result().success:
                self.__logger.info(f"校正Odom成功")
            else:
                self.__logger.error(f"校正Odom失败.")

            return
