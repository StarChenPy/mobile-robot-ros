import rclpy
from rclpy.node import Node
from mnn_msgs.srv import MnnService  # 导入 mnn 服务接口
from ..util.data_type import Rectangle, MnnResult


class VisionImpl:
    def __init__(self, node: Node):
        """
        初始化 VisionImpl 类实例
        """
        self.__node = node
        self.__logger = node.get_logger()
        self.__mnn_client = node.create_client(MnnService, '/mnn/mnn_cmd')  # 创建服务客户端

    def send_mnn_request(self) -> list[MnnResult]:
        """
        发送 MNN 请求，传递数据到服务端
        """
        request = MnnService.Request()
        request.cmd = 2
        request.data = ""

        future = self.__mnn_client.call_async(request)
        self.__logger.debug("[视觉] MNN 请求已发送")

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if future is None:
                continue

            if future.done():
                result_list = []

                result = future.result()
                for label, confidence, x_min, y_min, x_max, y_max in zip(
                        result.label, result.confidence, result.xmin, result.ymin, result.xmax, result.ymax):
                    result_list.append(MnnResult(label, confidence, Rectangle(x_min, y_min, x_max, y_max)))

                self.__logger.debug(f"[视觉] 识别结果: {result}")

                return result_list
