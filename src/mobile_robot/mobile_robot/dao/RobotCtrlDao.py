import rclpy

import web_message_transform_ros2.msg
from ..util.Singleton import singleton


@singleton
class RobotCtrlDao(object):
    def __init__(self, node: rclpy.node.Node):
        self.robotCtrlPub = node.create_publisher(web_message_transform_ros2.msg.RobotCtrl, '/web_transform_node/robot_ctrl', 10)

        self.robotCtrl = web_message_transform_ros2.msg.RobotCtrl()
        self.robotCtrl.do0 = False
        self.robotCtrl.do1 = False
        self.robotCtrl.do2 = False
        self.robotCtrl.pwm0 = 0.0
        self.robotCtrl.pwm1 = 0.0
        self.robotCtrl.pwm2 = 0.0
        self.robotCtrl.pwm3 = 0.0
        self.robotCtrl.pwm4 = 0.0

        self.robotCtrlPub.publish(self.robotCtrl)

    # 设置DO输出(端口: 0-2, 电平: T/F)
    def writeDo(self, port, state: bool):
        match port:
            case 0:
                self.robotCtrl.do0 = state
            case 1:
                self.robotCtrl.do1 = state
            case 2:
                self.robotCtrl.do2 = state
        self.robotCtrlPub.publish(self.robotCtrl)

    # 设置 pwm (端口: 0-4, duty: 0-100%)
    def writePwm(self, port, duty):
        duty = float(min(max(duty, 0), 100))

        match port:
            case 0:
                self.robotCtrl.pwm0 = duty
            case 1:
                self.robotCtrl.pwm1 = duty
            case 2:
                self.robotCtrl.pwm2 = duty
            case 3:
                self.robotCtrl.pwm3 = duty
            case 4:
                self.robotCtrl.pwm4 = duty
        self.robotCtrlPub.publish(self.robotCtrl)