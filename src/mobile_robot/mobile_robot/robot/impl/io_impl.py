import threading
import time

from rclpy.node import Node

from web_message_transform_ros2.msg import RobotCtrl
from web_message_transform_ros2.msg import RobotData


class IoImpl(object):
    """
    由于 I/O 的特殊性，使用单例模式
    应使用 instance 方法创建对象
    """

    __robot_connect = False
    __robot_data = RobotData()
    _instance_lock = threading.Lock()

    def __init__(self, node: Node):
        self.__logger = node.get_logger()

        # 等待ros连接
        self.__robot_ctrl_pub = node.create_publisher(RobotCtrl, '/web_transform_node/robot_ctrl', 10)
        self.__robot_data_sub = node.create_subscription(RobotData, '/web_transform_node/robot_data',
                                                         self.__robot_data_callback, 10)

        self.__robot_ctrl = RobotCtrl()
        self.__robot_ctrl.do0 = False
        self.__robot_ctrl.do1 = False
        self.__robot_ctrl.do2 = False
        self.__robot_ctrl.pwm0 = 0.0
        self.__robot_ctrl.pwm1 = 0.0
        self.__robot_ctrl.pwm2 = 0.0
        self.__robot_ctrl.pwm3 = 0.0
        self.__robot_ctrl.pwm4 = 0.0

        threading.Thread(target=self.__write_handle, daemon=True).start()

        self.__logger.info('[I/O] 初始化完成')

    @classmethod
    def instance(cls, *args, **kwargs):
        if not hasattr(IoImpl, "_instance"):
            IoImpl._instance = IoImpl(*args, **kwargs)
        return IoImpl._instance

    # 读取回调
    def __robot_data_callback(self, msg):
        self.__robot_connect = True
        self.__robot_data = msg

    # 通过定时器自动发布话题
    def __write_handle(self):
        while True:
            self.__robot_ctrl_pub.publish(self.__robot_ctrl)
            time.sleep(0.2)

    # 设置DO输出(端口: 0-2, 电平: T/F)
    def write_do(self, port, state: bool):
        match port:
            case 0:
                self.__robot_ctrl.do0 = state
            case 1:
                self.__robot_ctrl.do1 = state
            case 2:
                self.__robot_ctrl.do2 = state

    # 设置 pwm (端口: 0-4, duty: 0-100%)
    def write_pwm(self, port, duty):
        duty = float(min(max(duty, 0), 100))

        match port:
            case 0:
                self.__robot_ctrl.pwm0 = duty
            case 1:
                self.__robot_ctrl.pwm1 = duty
            case 2:
                self.__robot_ctrl.pwm2 = duty
            case 3:
                self.__robot_ctrl.pwm3 = duty
            case 4:
                self.__robot_ctrl.pwm4 = duty

    def is_connect(self):
        return self.__robot_connect

    # 读取 DI 输入(端口: 0-1)
    def get_di(self, port: int):
        return self.__robot_data.di[port]

    # 读取 titan 限位开关输入
    def get_titan_sw(self, port: int):
        return self.__robot_data.titan_limit_sw[port]

    # 获取电量
    def get_battery_vol(self):
        return self.__robot_data.battery

    # 获取轮式 odom
    def get_odometry(self):
        return self.__robot_data.odom

    # 获取急停状态
    def get_emg_status(self):
        return self.__robot_data.emg

    # 读取按键
    def get_start_button(self):
        return self.get_di(1)

    # 设置 LED 灯
    def set_led(self, state: bool):
        self.write_do(0, state)

    def get_ir_claws(self) -> float:
        return self.__robot_data.ir[0]

    def get_ir_front(self) -> float:
        return self.__robot_data.ir[1]
