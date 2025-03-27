import cv2
import cv_bridge
import rclpy

import camera_orbbec2.srv

from ..util.Singleton import singleton


# 相机FOV为110度

@singleton
class CameraDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__cv_bridge = cv_bridge.CvBridge()
        self.__service = node.create_client(camera_orbbec2.srv.ReqImage, "/camera/req_image")

        self.__service.wait_for_service()

    def __call_service(self, request: camera_orbbec2.srv.ReqImage.Request) -> camera_orbbec2.srv.ReqImage.Response:
        while rclpy.ok():
            future = self.__service.call_async(request)

            rclpy.spin_until_future_complete(self.__node, future, timeout_sec=15.0)

            if not future.done():
                self.__logger.warning(f"[CameraDao] 请求拍照失败，正在重试")
                continue

            if future.result().success:
                return future.result()
            else:
                self.__logger.warning(f"[CameraDao] 请求拍照失败，正在重试")
                continue

        self.__logger.error("[CameraDao] 拍照失败")

    def photograph_all(self, is_sync: bool) -> tuple[cv2.Mat, cv2.Mat]:
        """
        @param is_sync: true为同步模式, 拍摄实时最新的图像, 响应时间约100ms-200ms; false为异步模式, 为上一帧或当前帧, 响应时间短;
        @return 彩色图、深度图
        """
        self.__logger.debug(f"[CameraDao] 正在拍摄彩色图, 是否是最新图像: {is_sync}")
        request = camera_orbbec2.srv.ReqImage.Request()
        request.cmd = 0
        request.sync_snap = is_sync

        result = self.__call_service(request)

        color = self.__cv_bridge.imgmsg_to_cv2(result.image_color, 'bgr8')
        depth = self.__cv_bridge.imgmsg_to_cv2(result.image_depth, 'mono16')
        return color, depth

    def photograph_color(self, is_sync: bool) -> cv2.Mat:
        """
        @param is_sync: true为同步模式, 拍摄实时最新的图像, 响应时间约100ms-200ms; false为异步模式, 为上一帧或当前帧, 响应时间短;
        @return 彩色图
        """
        self.__logger.debug(f"[CameraDao] 正在拍摄彩色图, 是否是最新图像: {is_sync}")
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
        self.__logger.debug(f"[CameraDao] 正在拍摄深度图, 是否是最新图像: {is_sync}")
        request = camera_orbbec2.srv.ReqImage.Request()
        request.cmd = 2
        request.sync_snap = is_sync

        result = self.__call_service(request)
        return self.__cv_bridge.imgmsg_to_cv2(result.image_depth, 'mono16')
