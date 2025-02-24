import rclpy

import mnn_msgs.srv

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
        self.__logger.debug("[MNN] 已请求服务")

        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=5.0)

        if not future.done():
            self.__logger.error("[MNN] 请求超时未响应")
            return []

        return future.result()
