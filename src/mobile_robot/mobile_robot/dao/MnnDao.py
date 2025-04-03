import cv2
import rclpy
import cv_bridge

import mnn_detector.srv
import mnn_detector.msg
from ..popo.IdentifyResult import IdentifyResult
from ..popo.MnnCmd import MnnCmd
from ..popo.Rectangle import Rectangle
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class MnnDao:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.__service = node.create_client(mnn_detector.srv.CmdImpl, '/mnn_detector/ctrl_impl')

    def __call_service(self, request: mnn_detector.srv.CmdImpl.Request):
        """
            #错误代码[code]
            int8 CODE_SUCCESS  = 0             #正常
            int8 CODE_LOAD_MODE_ERR = -1       #加载模型错误
            int8 CODE_DETECT_ERR = -2          #检测错误
            int8 CODE_IMAGE_ERR = -3           #获取图像错误
            int8 CODE_INTRINSICS_ERR = -4      #获取内参错误

            mnn_detector.srv.CmdImpl_Request(cmd=0, mode_param=mnn_detector.msg.ModelParam(mode_path='', label_path='', nms_thres=0.0))
            mnn_detector.srv.CmdImpl_Response(code=0, info='success', status=1)
        """
        #请求服务
        future = self.__service.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            return future.result()

    #启动检测(模型路径,标签路径,nms阈值)
    def start(self, mode_path: str, label_path: str, nms_threshold: float):
        #设置模型参数
        request = mnn_detector.srv.CmdImpl.Request()
        request.cmd = MnnCmd.CMD_LOAD_MODEL.value
        request.mode_param.mode_path = mode_path
        request.mode_param.label_path = label_path
        request.mode_param.nms_thres = nms_threshold

        res = self.__call_service(request)

        self.__logger.debug(f'模型路径:"{mode_path}" 标签路径:"{label_path}" NMS阈值:{nms_threshold}')
        if res.code != 0:
            self.__logger.error(f'启动检测器错误! 错误代码:{res.code} 信息:{res.info}')
            return False
        else:
            return True

    #关闭检测器
    def stop(self):
        request = mnn_detector.srv.CmdImpl.Request()
        request.cmd = MnnCmd.CMD_CLOSE_MODEL.value
        res = self.__call_service(request)
        if res.code != 0:
            self.__logger.error(f'关闭检测器错误! 错误代码:{res.code} 信息:{res.info}')
        else:
            self.__logger.debug('关闭检测器完成.')

    #检测
    def detect(self) -> list[IdentifyResult]:
        result = []

        request = mnn_detector.srv.CmdImpl.Request()
        request.cmd = MnnCmd.CMD_DETECT_ONCE.value
        res = self.__call_service(request)

        if res.code != 0:
            self.__logger.error(f'检测失败! 错误代码:{res.code} 信息:{res.info}')
            return []

        # 打包消息数据
        for info_ in res.result.category_infos:
            result.append(IdentifyResult(info_.label, info_.confidence,
                                         Rectangle(info_.rect[0], info_.rect[1], info_.rect[0] + info_.rect[2], info_.rect[1] + info_.rect[3])))

        return result

    #获取检测结果图片
    def get_result_image(self) -> cv2.Mat | None:
        request = mnn_detector.srv.CmdImpl.Request()
        request.cmd = MnnCmd.CMD_GET_RESULT_IMG.value
        res = self.__call_service(request)
        if res.code != 0:
            self.__logger.error(f'获取结果失败! 错误代码:{res.code} 信息:{res.info}')
            return None
        img = cv_bridge.CvBridge().imgmsg_to_cv2(res.image, 'bgr8')
        return img

    #检测器是否已经打开(加载完模型)
    def is_opened(self):
        request = mnn_detector.srv.CmdImpl.Request()
        request.cmd = MnnCmd.CMD_QUERY.value
        return self.__call_service(request).status
