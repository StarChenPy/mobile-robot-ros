import paramiko
import os
import cv2
# -----------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# -----------------------
from orbbec_opencv.srv import ReqImage
from nav_msgs.srv import SetMap
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类


def try_except(fn):
    def inner(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except:
            return -1

    return inner

class CameraImpl:
    def __init__(self, ros, web_trans, ip):
        self.__ros = ros
        self.__web_trans = web_trans
        self.__ip = ip
        self.__cv_bridge = CvBridge()
        self.__req_srv = self.__ros.create_client(ReqImage, '/request_image') # 'camera_orbbec/ReqImage'
        # -----------------------
        # # 创建订阅者
        # self.__subscription = self.__ros.create_subscription(Image,'/camera/color/image_raw',self.image_callback,1)  # 10是队列大小
        # self.__subscription 
        # self.__cv_bridge_a = CvBridge()
        # self.latest_cv_image = None  # 用于存储最新的OpenCV图像
        # -----------------------
        print('[相机接口] 初始化完成.')

# -----------------------
    # def image_callback(self, msg):
    #     # 将ROS的图像消息转化成OpenCV图像
    #     try:
    #         cv_image = self.__cv_bridge_a.imgmsg_to_cv2(msg, "bgr8")
    #         print('[相机接口] 收到图像.')
    #     except Exception as e:
    #         print(e)
    #         return
    #     # 更新最新的OpenCV图像
    #     self.latest_cv_image = cv_image

    # def process_image(self):
    #     # 返回最新的OpenCV图像
    #     return self.latest_cv_image is not None, self.latest_cv_image
# ----------------------


    # 拍照-彩色图
    def snap_color(self, image_name='test'):
        #请求ros拍照
        res = self.__call_req_image()
        if res.success:
            print('[相机接口] 拍照请求完成.', image_name)
        else:
            print('[相机接口] 拍照请求失败.')
            return False, None
        # #获取本地images文件夹路径
        # save_path = self.__get_image_path() + "/color_" + image_name + ".png"
        # #获取远端图片路径
        # remote_path_color = res['path_color']
        # #文件拷贝
        # if not self.copy_pictures(self.__ip, remote_path_color, save_path.__str__()) == -1:
        #     # 读取图片mat
        #     image = cv2.imread(save_path)
        #     return True, image
        
        image = self.__cv_bridge.imgmsg_to_cv2(res.image, 'bgr8')  # 将ROS的图像消息转化成OpenCV图像
        
        return True, image
        # return False, None
        
        



    # 拍照-深度图
    def snap_depth(self, image_name='test'):
        #请求ros拍照
        res = self.__call_req_image()
        if res['success']:
            print('[相机接口] 拍照请求完成.', image_name)
        else:
            print('[相机接口] 拍照请求失败.')
            return False, None
        #获取本地images文件夹路径
        save_path = self.__get_image_path() + "/depth_" + image_name + ".png"
        #获取远端图片路径
        remote_path_color = res['path_depth']
        #文件拷贝
        if not self.copy_pictures(self.__ip, remote_path_color, save_path.__str__()) == -1:
            # 读取图片mat
            image = cv2.imread(save_path)
            return True, image

        return False, None



    def snap(self, image_name='test'):
        #请求ros拍照
        res = self.__call_req_image()
        if res.success:
            print('[相机接口] 拍照请求完成.', image_name)
        else:
            print('[相机接口] 拍照请求失败.')
            return False, None
        #获取本地images文件夹路径
        save_path_color = self.__get_image_path() + "/color_" + image_name + ".png"
        save_path_depth = self.__get_image_path() + "/depth_" + image_name + ".png"

        #获取远端图片路径
        remote_path_color = res['path_color']
        remote_path_depth = res['path_depth']
        #文件拷贝
        if not self.copy_pictures(self.__ip, remote_path_color, save_path_color.__str__()) == -1:
            # 读取图片mat
            image_color = cv2.imread(save_path_color)
        else:
            return False, None, None
        if not self.copy_pictures(self.__ip, remote_path_depth, save_path_depth.__str__()) == -1:
            # 读取图片mat
            image_depth = cv2.imread(save_path_depth)
        else:
            return False, None, None

        return True, image_color, image_depth


    #调用拍照服务
    def __call_req_image(self):

        req = ReqImage.Request()
            
        res = self.__req_srv.call(req)
        return res

    @try_except
    def copy_pictures(self, host, remotepath, localpath):
        transport = paramiko.Transport((host, 22))
        transport.connect(username="pi", password="password")#raspberry
        sftp = paramiko.SFTPClient.from_transport(transport)
        sftp.get(remotepath, localpath)
        transport.close()


    def __get_image_path(self):
        target_path = os.path.dirname(os.path.abspath(__file__))
        target_path = os.path.dirname(target_path)
        target_path = os.path.dirname(target_path)
        target_path = os.path.join(target_path, 'images')
        return target_path




