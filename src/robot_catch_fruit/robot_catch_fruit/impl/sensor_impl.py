import time


"""
        传感器接口
    1: 红外传感器
    2: 超声波传感器
    3: 寻线传感器
    4: IMU传感器

2024-03-13 com.huibo.robot Yongjie.Hu
"""
class SensorImpl:
    __web_trans = None


    def __init__(self,  web_trans):
        self.__web_trans = web_trans

    #获取红外传感器数据(0-3) 返回:距离(m)
    def read_ir(self, port=0):
        return self.__web_trans.read().ir[port]

    #获取超声波传感器数据(0-1) 返回:距离(m)
    def read_sonar(self, port=0):
        return self.__web_trans.read().sonar[port]

    #读取imu数据
    def read_imu(self):
        return self.__web_trans.read().imu

    #读取寻线传感器数据
    def read_lsb(self):
        return self.__web_trans.read().lsb

