from rclpy.node import Node

from orbbec_opencv.srv import ReqImage
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类

class CameraImpl:
    def __init__(self, node: Node):
        self.__logger = node.get_logger()

        self.__cv_bridge = CvBridge()
        self.__req_srv = node.create_client(ReqImage, '/request_image')

        self.__logger.info('[相机接口] 初始化完成.')

    # 拍照-彩色图
    def snap_color(self):
        #请求ros拍照
        res = self.__call_req_image()
        if res.success:
            print('[相机接口] 拍照请求完成.')
        else:
            print('[相机接口] 拍照请求失败.')
            return False, None

        image = self.__cv_bridge.imgmsg_to_cv2(res.image, 'bgr8')  # 将ROS的图像消息转化成OpenCV图像

        return image

    # 拍照-深度图
    def snap_depth(self):
        #请求ros拍照
        res = self.__call_req_image()
        if res.success:
            print('[相机接口] 拍照请求完成.')
        else:
            print('[相机接口] 拍照请求失败.')
            return False, None

        image = self.__cv_bridge.imgmsg_to_cv2(res.image)

        return image

    #调用拍照服务
    def __call_req_image(self):
        req = ReqImage.Request()

        res = self.__req_srv.call(req)
        return res
