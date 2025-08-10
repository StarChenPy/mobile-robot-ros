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

    def send_correction_odom(self, waypoint_name: str):
        future = self.__service.call_async(CorrectionOdom.Request(waypoint_name=waypoint_name))

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            if future.result().success:
                self.__logger.info(f"校正Odom成功, 路径点: {waypoint_name}")
            else:
                self.__logger.error(f"校正Odom失败.")

            return
