import rclpy

import web_message_transform_ros2.msg
from ..util.Singleton import singleton


@singleton
class RobotCtrlDao(object):
    def __init__(self, node: rclpy.node.Node):
        self.__topic = node.create_publisher(
            web_message_transform_ros2.msg.RobotCtrl,
            '/web_transform_node/robot_ctrl',
            10)

        self.__robot_ctrl = web_message_transform_ros2.msg.RobotCtrl()
        self.__robot_ctrl.do0 = False
        self.__robot_ctrl.do1 = False
        self.__robot_ctrl.do2 = False
        self.__robot_ctrl.pwm0 = 0.0
        self.__robot_ctrl.pwm1 = 0.0
        self.__robot_ctrl.pwm2 = 0.0
        self.__robot_ctrl.pwm3 = 0.0
        self.__robot_ctrl.pwm4 = 0.0

        self.__topic.publish(self.__robot_ctrl)

    # 设置DO输出(端口: 0-2, 电平: T/F)
    def write_do(self, port, state: bool):
        match port:
            case 0:
                self.__robot_ctrl.do0 = state
            case 1:
                self.__robot_ctrl.do1 = state
            case 2:
                self.__robot_ctrl.do2 = state
        self.__topic.publish(self.__robot_ctrl)

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
        self.__topic.publish(self.__robot_ctrl)