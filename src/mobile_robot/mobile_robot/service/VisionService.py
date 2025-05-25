import ament_index_python
import cv2
import rclpy

from ..dao.CameraDao import CameraDao
from ..dao.MnnDao import MnnDao
from ..popo.IdentifyResult import IdentifyResult
from ..popo.Point import Point
from ..util.Logger import Logger
from ..util.Onnx import infer_onnx_model
from ..util.Singleton import singleton


@singleton
class VisionService:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = Logger()

        self.__mnn = MnnDao(node)
        self.__camera = CameraDao(node)

        share_directory = ament_index_python.packages.get_package_share_directory("mobile_robot")
        self.__weight_path = share_directory + "/weights/best.onnx"
        self.__names_path = share_directory + "/weights/fruit.names"

    def get_mnn_identify_result(self) -> list[IdentifyResult]:
        self.__mnn.start("/home/vmx/WSR_HB_Robot/models/coco_Y3_1900_sim.mnn",
                         "/home/vmx/WSR_HB_Robot/models/coco_Y3_1900.names",
                         0.7)
        result = self.__mnn.detect()
        self.__mnn.stop()
        return result

    def show_photo(self, photo) -> None:
        result = infer_onnx_model(self.__weight_path, photo)
        for r in result:
            cv2.rectangle(photo, (r.box.x1, r.box.y1), (r.box.x2, r.box.y2), (0, 255, 0), 2)
            cv2.putText(photo, '{} {:.3f}'.format(r.classId, r.confidence), (r.box.x1, r.box.y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow("123", photo)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def get_onnx_identify_result(self, inverted=False) -> list[IdentifyResult]:
        count = 0
        while True:
            photo, depth = self.__camera.photograph_all(True)
            if inverted:
                photo = cv2.rotate(photo, cv2.ROTATE_180)
                depth = cv2.rotate(depth, cv2.ROTATE_180)
            result = infer_onnx_model(self.__weight_path, photo)

            self.show_photo(photo)

            flag = False
            for r in result:
                point = r.box.get_rectangle_center()
                r.distance = depth[int(point.y), int(point.x)] / 1000
                if r.distance != 0:
                    flag = True

            if flag:
                break
            else:
                self.__logger.warn(f"没有检测到深度，重试第{count}次")
                count += 1

            if count > 15:
                self.__logger.error("深度检测失败，无深度信息")
                break

        return result

    def get_depth_data(self, point: Point) -> float:
        """
        @param point: 图像的坐标点
        @return float: 深度距离，单位 cm
        """

        depth = self.__camera.photograph_depth(True)
        distance = depth[int(point.y), int(point.x)] / 1000
        return distance.item()
