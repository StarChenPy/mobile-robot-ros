import ament_index_python
import cv2
import numpy as np
import rclpy

from ..dao.CameraDao import CameraDao
from ..dao.MnnDao import MnnDao
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.Point import Point
from ..popo.Rectangle import Rectangle
from ..util import Math
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

    def get_onnx_identify_depth(self, inverted=False) -> list[IdentifyResult]:
        count = 0
        while True:
            photo, depth = self.__camera.photograph_all(True)
            if inverted:
                photo = cv2.rotate(photo, cv2.ROTATE_180)
                depth = cv2.rotate(depth, cv2.ROTATE_180)
            result = infer_onnx_model(self.__weight_path, photo)

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

    def get_onnx_identify(self, inverted=False) -> list[IdentifyResult]:
        while True:
            photo = self.__camera.photograph_color(True)
            self.show_photo(photo)
            if inverted:
                photo = cv2.rotate(photo, cv2.ROTATE_180)
            return infer_onnx_model(self.__weight_path, photo)

    def get_depth_data(self, point: Point) -> float:
        """
        @param point: 图像的坐标点
        @return float: 深度距离，单位 cm
        """

        depth = self.__camera.photograph_depth(True)
        distance = depth[int(point.y), int(point.x)] / 1000
        return distance.item()

    def get_order_box(self) -> dict[int: list[Rectangle]]:
        image = self.__camera.photograph_color(True)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # 转换为HSV色彩空间
        in_range = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 40]))
        opening = cv2.morphologyEx(in_range, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 筛选轮廓以找出矩形（近似长方形）
        squares = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)  # 多边形拟合
            if len(approx) == 4 and cv2.isContourConvex(approx):  # 判断是否为凸四边形
                x, y, w, h = cv2.boundingRect(approx)
                ratio = w / float(h)
                # 判断长方形，且不是很小
                if 1.8 < ratio < 2.2 and w > 50 and h > 50:
                    squares = Math.split_rectangle(x, y, x + w, y + h)
                    break

        return squares

    def order_recognition(self) -> dict[int: list[FruitType]]:
        """
        识别订单
        """
        self.__logger.info("开始识别订单")
        order_box: dict = self.get_order_box()
        if not order_box:
            self.__logger.error("没有识别到订单板")
            return {}

        result = self.get_onnx_identify()
        if not result:
            self.__logger.error("没有识别到订单")
            return {}

        orders = {}
        for i in order_box.keys():
            order_center_points = [j.box.get_rectangle_center() for j in result]
            squares = Math.find_points_in_squares(order_center_points, order_box[i])
            orders[i] = squares

        return orders
