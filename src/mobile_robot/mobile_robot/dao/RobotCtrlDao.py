import rclpy

from web_message_transform_ros2.msg import RobotCtrl
from web_message_transform_ros2.srv import RobotCtrlSrv
from ..util.Logger import Logger
from ..util.Singleton import singleton


@singleton
class RobotCtrlDao(object):
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = Logger()

        self.service = self.__node.create_client(RobotCtrlSrv, '/web_transform_node/robot_ctrl_srv')
        # self.__topic = node.create_publisher(RobotCtrl, '/web_transform_node/robot_ctrl', 10)

        self.__robot_ctrl = RobotCtrl()
        self.__robot_ctrl.do0 = False
        self.__robot_ctrl.do1 = False
        self.__robot_ctrl.do2 = False
        self.__robot_ctrl.pwm0 = 0.0
        self.__robot_ctrl.pwm1 = 0.0
        self.__robot_ctrl.pwm2 = 0.0
        self.__robot_ctrl.pwm3 = 0.0
        self.__robot_ctrl.pwm4 = 0.0

    def publish(self):
        if not self.service.wait_for_service(timeout_sec=10.0):
            self.__logger.error('机器人控制服务未启动...')

        req = RobotCtrlSrv.Request()
        req.ctrl = self.__robot_ctrl
        future = self.service.call_async(req)

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            result = future.result()
            if result.ret:
                self.__logger.debug("发布控制信息成功.")
            else:
                self.__logger.error("发布控制信息失败.")

            return

    # 设置DO输出(端口: 0-2, 电平: T/F)
    def write_do(self, port: int, state: bool):
        self.__logger.debug(f"操作DO端口 {port} 为 {state}")
        match port:
            case 0:
                self.__robot_ctrl.do0 = state
            case 1:
                self.__robot_ctrl.do1 = state
            case 2:
                self.__robot_ctrl.do2 = state
        self.publish()

    # 设置 pwm (端口: 0-4, duty: 0-100%)
    def write_pwm(self, port: int, duty: float, publish=True):
        duty = float(min(max(duty, 0), 100))
        if duty > 100:
            self.__logger.warn(f"操作PWM端口 {port} 占空比为 {duty} 超过最大值!")
            duty = 100
        elif duty < 0:
            self.__logger.warn(f"操作PWM端口 {port} 占空比为 {duty} 超过最小值!")
            duty = 0
        else:
            self.__logger.debug(f"操作PWM端口 {port} 占空比为 {duty}")

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

        if publish:
            self.publish()

    def read_pwm(self, port: int) -> float:
        """
        读取指定PWM端口的占空比
        @param port: PWM端口 (0-4)
        @return: 占空比 (0-100)
        """
        match port:
            case 0:
                return self.__robot_ctrl.pwm0
            case 1:
                return self.__robot_ctrl.pwm1
            case 2:
                return self.__robot_ctrl.pwm2
            case 3:
                return self.__robot_ctrl.pwm3
            case 4:
                return self.__robot_ctrl.pwm4
            case _:
                self.__logger.error(f"未知PWM端口: {port}")
                return 0
