import rclpy

import mnn_msgs.srv

from ..popo.MnnResult import MnnResult
from ..popo.Rectangle import Rectangle
from ..util.Singleton import singleton


@singleton
class MnnDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()
        self.__service = node.create_client(mnn_msgs.srv.MnnService, '/mnn/mnn_cmd')

    def call_service(self):
        """
        发送 MNN 请求，传递数据到服务端
        """
        request = mnn_msgs.srv.MnnService.Request()
        request.cmd = 2
        request.data = ""

        future = self.__service.call_async(request)
        self.__logger.debug("[视觉] MNN 请求已发送")

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if future is None:
                continue

            if future.done():
                return future.result()
