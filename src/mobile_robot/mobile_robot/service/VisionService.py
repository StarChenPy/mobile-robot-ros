import ament_index_python
import rclpy

from ..dao.CameraDao import CameraDao
from ..dao.MnnDao import MnnDao
from ..popo import Point
from ..popo.IdentifyResult import IdentifyResult
from ..popo.Rectangle import Rectangle
from ..util.Singleton import singleton
from ..util.Onnx import infer_onnx_model


@singleton
class VisionService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__mnn = MnnDao(node)
        self.__camera = CameraDao(node)

        share_directory = ament_index_python.packages.get_package_share_directory("mobile_robot")
        self.__weight_path = share_directory + "/weights/best-sim.onnx"
        self.__names_path = share_directory + "/weights/fruit.names"

    def get_mnn_identify_result(self) -> list[IdentifyResult]:
        result = self.__mnn.call_service()
        if not result:
            return []

        result_list = []
        for label, confidence, x_min, y_min, x_max, y_max in zip(
                result.label, result.confidence, result.xmin, result.ymin, result.xmax, result.ymax):
            result_list.append(IdentifyResult(label, confidence, Rectangle(x_min, y_min, x_max, y_max)))

        return result_list

    def get_onnx_identify_result(self) -> list[IdentifyResult]:
        photo = self.__camera.photograph_color(True)
        return infer_onnx_model(self.__weight_path, photo)

    def get_depth_data(self, point: Point) -> float:
        photograph_depth = self.__camera.photograph_depth(True)
        dis = photograph_depth[point.y, point.x] / 1000
        return dis.item()

