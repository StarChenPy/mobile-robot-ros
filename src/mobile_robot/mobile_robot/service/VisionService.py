import ament_index_python
import rclpy

from ..dao.CameraDao import CameraDao
from ..dao.MnnDao import MnnDao
from ..popo.IdentifyResult import IdentifyResult
from ..popo.Point import Point
from ..util.Logger import Logger
from ..util.Singleton import singleton
from ..util.Onnx import infer_onnx_model


@singleton
class VisionService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__mnn = MnnDao(node)
        self.__camera = CameraDao(node)

        share_directory = ament_index_python.packages.get_package_share_directory("mobile_robot")
        self.__weight_path = share_directory + "/weights/best-sim.onnx"
        self.__names_path = share_directory + "/weights/fruit.names"

    def get_mnn_identify_result(self) -> list[IdentifyResult]:
        self.__mnn.start("/home/vmx/WSR_HB_Robot/models/coco_Y3_1900_sim.mnn",
                         "/home/vmx/WSR_HB_Robot/models/coco_Y3_1900.names",
                         0.7)
        result = self.__mnn.detect()
        self.__mnn.stop()
        return result

    def get_onnx_identify_result(self) -> list[IdentifyResult]:
        photo, depth = self.__camera.photograph_all(True)
        result = infer_onnx_model(self.__weight_path, photo)
        for r in result:
            point = r.box.get_rectangle_center()
            r.distance = depth[int(point.y), int(point.x)] / 1000
        return result

    def get_depth_data(self, point: Point) -> float:
        """
        @param point: 图像的坐标点
        @return float: 深度距离，单位 cm
        """

        depth = self.__camera.photograph_depth(True)
        distance = depth[int(point.y), int(point.x)] / 1000
        return distance.item()
