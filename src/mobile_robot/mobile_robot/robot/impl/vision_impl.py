import time

import rclpy
from rclpy.node import Node
from mnn_msgs.srv import MnnService  # 导入 mnn 服务接口


class VisionImpl:
    def __init__(self, node: Node):
        """
        初始化 VisionImpl 类实例
        """
        self.__future = None
        self.__node = node
        self.__logger = node.get_logger()
        self.__mnn_client = node.create_client(MnnService, '/mnn/mnn_cmd')  # 创建服务客户端

    def send_mnn_request(self):
        """
        发送 MNN 请求，传递数据到服务端
        """
        self.__logger.info("[视觉] 等待服务端上线...")
        if not self.__mnn_client.wait_for_service(30):
            self.__logger.error("[视觉] 未检测到服务端")
            return

        request = MnnService.Request()
        request.cmd = 2
        request.data = ""

        self.__future = self.__mnn_client.call_async(request)
        self.__logger.info("[视觉] MNN 请求已发送")

    def mnn_result(self):
        if self.__future is None:
            return
        while rclpy.ok():
            rclpy.spin_once(self.__node)
            if self.__future.done():
                return self.__future.result()
            time.sleep(0.2)
