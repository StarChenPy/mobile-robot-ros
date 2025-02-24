import rclpy

from ..dao.MnnDao import MnnDao
from ..popo.MnnResult import MnnResult
from ..popo.Rectangle import Rectangle
from ..util.Singleton import singleton


@singleton
class VisionService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__mnn = MnnDao()

    def get_identify_result(self) -> list[MnnResult]:
        result = self.__mnn.call_service()

        # 校验数据一致性
        attributes = [
            result.label,
            result.confidence,
            result.xmin,
            result.ymin,
            result.xmax,
            result.ymax
        ]

        # 使用列表推导式优化构建过程
        result_list = [
            MnnResult(label, conf, Rectangle(x1, y1, x2, y2)) for label, conf, x1, y1, x2, y2 in attributes
        ]

        return result_list