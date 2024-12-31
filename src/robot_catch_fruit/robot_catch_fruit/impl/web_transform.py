import rclpy
from std_msgs.msg import String
from web_message_transform_ros2.msg import RobotCtrl
from web_message_transform_ros2.msg import RobotData
from vmxpi_msgs.msg import Encoder
from concurrent.futures import ThreadPoolExecutor
import time
class WebTransform:
    __ros = None
    __robot_data_sub = None
    __robot_ctrl_pub = None

    __robot_data = None  # 机器人反馈数据订阅
    
    __robot_connect = False

    def __init__(self, rosNode):
        #等待ros连接
        self.__ros = rosNode
        
        self.__robot_ctrl_pub = self.__ros.create_publisher(RobotCtrl , '/web_transform_node/robot_ctrl', 10) #"web_message_transform/RobotCtrl"
        self.__robot_data_sub = self.__ros.create_subscription(RobotData, '/web_transform_node/robot_data', self.__robot_data_cb, 10)#"web_message_transform/RobotData"
        
        self.__robot_ctrl = RobotCtrl()
        self.__robot_ctrl.do0 = False
        self.__robot_ctrl.do1 = False
        self.__robot_ctrl.do2 = False
        self.__robot_ctrl.pwm0 = 0.0
        self.__robot_ctrl.pwm1 = 0.0
        self.__robot_ctrl.pwm2 = 0.0
        self.__robot_ctrl.pwm3 = 0.0
        self.__robot_ctrl.pwm4 = 0.0

        self.timer = self.__ros.create_timer(0.1, self.__write_handle)

    def read(self):
        return self.__robot_data

    #设置DO(0-2)
    def write_do(self, port=0, state=False):
        if port == 0:
            self.__robot_ctrl.do0 = state
        elif port == 1:
            self.__robot_ctrl.do1 = state
        elif port == 2:
            self.__robot_ctrl.do2 = state

    #设置pwm( 0-100%)
    def write_pwm(self, port=0, duty=0.0):
        if duty > 100.0:
            duty = 100.0
        if duty < 0.0:
            duty = 0.0
        key_list = ['pwm0', 'pwm1', 'pwm2', 'pwm3', 'pwm4']
        if port == 0:
            self.__robot_ctrl.pwm0 = duty
        elif port == 1:
            self.__robot_ctrl.pwm1 = duty
        elif port == 2:
            self.__robot_ctrl.pwm2 = duty
        elif port == 3:
            self.__robot_ctrl.pwm3 = duty
        elif port == 4:
            self.__robot_ctrl.pwm4 = duty

    def __write_handle(self):
        self.__robot_ctrl_pub.publish(self.__robot_ctrl)


    def close(self):
        self.timer.shutdown()

    # #机器人数据读取注册
    # def __read_handle(self):
    #     self.__robot_data_sub = roslibpy.Topic(self.__ros, '/web_message_transform/robot_data', 'web_message_transform/RobotData', queue_size=1,
    #                          throttle_rate=self.__update_rate)
    #     self.__robot_data_sub.subscribe(self.__robot_data_cb)

    def is_robot_connect(self):
        return self.__robot_connect

    #读取回调
    def __robot_data_cb(self, msg):
        # print("asdasdasd")
        self.__robot_connect = True
        self.__robot_data = msg