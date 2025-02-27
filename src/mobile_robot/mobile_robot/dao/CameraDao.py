import time

import cv2
import cv_bridge
import rclpy

import camera_orbbec2.srv

from ..util.Singleton import singleton


@singleton
class CameraDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__cv_bridge = cv_bridge.CvBridge()
        self.__service = node.create_client(camera_orbbec2.srv.ReqImage, "/camera/req_image")

    def __call_service(self, request: camera_orbbec2.srv.ReqImage.Request) -> camera_orbbec2.srv.ReqImage.Response:
        self.__service.wait_for_service()
        future = self.__service.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                time.sleep(0.5)
                continue

            if future.result().success:
                return future.result()
            else:
                self.__logger.warning(f"[相机] 请求拍照失败，正在重试")

        self.__logger.error("[相机] 拍照失败")

    def photograph_color(self, is_sync: bool) -> cv2.Mat:
        """
        @param is_sync: true为同步模式, 拍摄实时最新的图像, 响应时间约100ms-200ms; false为异步模式, 为上一帧或当前帧, 响应时间短;
        @return 彩色图
        """
        request = camera_orbbec2.srv.ReqImage.Request()
        request.cmd = 1
        request.sync_snap = is_sync

        result = self.__call_service(request)
        return self.__cv_bridge.imgmsg_to_cv2(result.image_color, 'bgr8')

    def photograph_depth(self, is_sync: bool) -> cv2.Mat:
        """
        @param is_sync: true为同步模式, 拍摄实时最新的图像, 响应时间约100ms-200ms; false为异步模式, 为上一帧或当前帧, 响应时间短;
        @return 深度图
        """
        request = camera_orbbec2.srv.ReqImage.Request()
        request.cmd = 2
        request.sync_snap = is_sync

        result = self.__call_service(request)
        return self.__cv_bridge.imgmsg_to_cv2(result.image_depth, 'mono16')