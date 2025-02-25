import rclpy

from ..dao.MnnDao import MnnDao
from ..popo.MnnResult import MnnResult
from ..popo.Rectangle import Rectangle
from ..util.Singleton import singleton


@singleton
class VisionService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__mnn = MnnDao(node)

    def get_identify_result(self) -> list[MnnResult]:
        result = self.__mnn.call_service()
        if not result:
            return []

        result_list = []
        for label, confidence, x_min, y_min, x_max, y_max in zip(
                result.label, result.confidence, result.xmin, result.ymin, result.xmax, result.ymax):
            result_list.append(MnnResult(label, confidence, Rectangle(x_min, y_min, x_max, y_max)))

        return result_list
