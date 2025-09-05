import time

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
        self.__weight_path = share_directory + "/weights/grape_and_apple.onnx"
        self.__names = ["red_apple", "green_apple", "yellow_apple", "purple_apple", "purple_grape", "green_grape", "yellow_grape"]

    def set_other_fruit_weight(self):
        share_directory = ament_index_python.packages.get_package_share_directory("mobile_robot")
        self.__weight_path = share_directory + "/weights/other_fruit.onnx"
        self.__names = ["apple", "pomegranate", "lemon", "snow_pear", "kiwifruit", "lotus_fruit", "custard_apple", "tangerine", "peach", "green_pepper"]

    def set_grape_and_apple_weight(self):
        share_directory = ament_index_python.packages.get_package_share_directory("mobile_robot")
        self.__weight_path = share_directory + "/weights/other_fruit.onnx"
        self.__names = ["red_apple", "green_apple", "yellow_apple", "purple_apple", "purple_grape", "green_grape", "yellow_grape"]

    def get_mnn_identify_result(self) -> list[IdentifyResult]:
        self.__mnn.start("/home/vmx/WSR_HB_Robot/models/coco_Y3_1900_sim.mnn",
                         "/home/vmx/WSR_HB_Robot/models/coco_Y3_1900.names",
                         0.7)
        result = self.__mnn.detect()
        self.__mnn.stop()
        return result

    def photograph(self):
        return self.__camera.photograph_color(True)

    def show_photo(self, photo, inverted=False) -> None:
        if inverted:
            photo = cv2.rotate(photo, cv2.ROTATE_180)
        result = infer_onnx_model(self.__weight_path, self.__names, photo)
        for r in result:
            cv2.rectangle(photo, (r.box.x1, r.box.y1), (r.box.x2, r.box.y2), (0, 255, 0), 2)
            cv2.putText(photo, '{} {:.3f}'.format(r.class_id, r.confidence), (r.box.x1, r.box.y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imshow("123", photo)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def get_onnx_identify_depth(self, inverted=False, kernel_size=11) -> list[IdentifyResult]:
        half_k = kernel_size // 2

        photo, depth = self.__camera.photograph_all(True)
        if inverted:
            photo = cv2.rotate(photo, cv2.ROTATE_180)
            depth = cv2.rotate(depth, cv2.ROTATE_180)

        result = infer_onnx_model(self.__weight_path, self.__names, photo)

        for r in result:
            point = r.box.get_rectangle_center()
            cx, cy = int(point.x), int(point.y)

            # 定义感兴趣区域（ROI），考虑图像边界
            x1 = max(0, cx - half_k)
            x2 = min(depth.shape[1], cx + half_k + 1)
            y1 = max(0, cy - half_k)
            y2 = min(depth.shape[0], cy + half_k + 1)

            roi = depth[y1:y2, x1:x2]
            valid_depths = roi[roi > 0]  # 自动忽略 0 深度
            valid_depths = valid_depths[valid_depths < 750]  # 自动忽略大于 750 的深度

            if valid_depths.size > 0:
                r.distance = float(np.median(valid_depths)) / 1000  # 使用中位数，避免离群值
            else:
                r.distance = -1

            if r.distance == -1:
                self.__logger.warn(f"{r.class_id} ({point.x}, {point.y}), 深度信息获取失败.")

        return result

    def get_onnx_identify(self, inverted=False) -> list[IdentifyResult]:
        while True:
            photo = self.__camera.photograph_color(True)
            if inverted:
                photo = cv2.rotate(photo, cv2.ROTATE_180)
            result = infer_onnx_model(self.__weight_path, self.__names, photo)
            return result

    def get_depth_data(self, point: Point, inverted=False, kernel_size=35) -> float:
        """
        @param point: 图像的坐标点
        @param inverted: 图像是否倒置
        @param kernel_size: 用于计算深度的感兴趣区域大小，必须为奇数

        @return float: 深度距离，单位 cm
        """
        depth = self.__camera.photograph_depth(True)
        if inverted:
            depth = cv2.rotate(depth, cv2.ROTATE_180)

        half_k = kernel_size // 2
        cx, cy = int(point.x), int(point.y)

        # 定义感兴趣区域（ROI），考虑图像边界
        x1 = max(0, cx - half_k)
        x2 = min(depth.shape[1], cx + half_k + 1)
        y1 = max(0, cy - half_k)
        y2 = min(depth.shape[0], cy + half_k + 1)

        roi = depth[y1:y2, x1:x2]
        valid_depths = roi[roi > 0]  # 自动忽略0深度
        valid_depths = valid_depths[valid_depths < 1000]  # 自动忽略大于1000的深度

        if valid_depths.size > 0:
            return float(np.median(valid_depths)) / 1000  # 使用中位数，避免离群值
        else:
            return -1

    def find_fruit(self, fruit: list[FruitType]=None, inverted=True, kernel_size=35) -> IdentifyResult | None:
        identify = self.get_onnx_identify_depth(inverted, kernel_size)
        if not fruit:
            return None

        for i in identify:
            if FruitType(i.class_id) not in fruit:
                continue
            return i
        return None

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
