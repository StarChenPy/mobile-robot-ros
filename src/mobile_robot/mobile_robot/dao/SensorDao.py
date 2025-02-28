import time

import rclpy

import user_sensor_msgs.srv
from ..util.Singleton import singleton


@singleton
class SensorDao(object):
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__service = self.__node.create_client(user_sensor_msgs.srv.SensorService, '/user/sensor_service_cmd')

    # 发送修正控制请求
    def __call_service(self, request: user_sensor_msgs.srv.SensorService.Request) -> bool:
        future = self.__service.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            result = future.result()

            if result.err_data == 0:
                return result.success

            match result.err_data:
                case 0:
                    pass
                case -1:
                    self.__logger.error("[修正控制接口] ping端口选择错误")
                case -2:
                    self.__logger.error("[修正控制接口] ir端口选择错误")
                case -3:
                    self.__logger.error("[修正控制接口] 传感器类型错误")
                case -101:
                    self.__logger.error("[修正控制接口] 安全控制触发(急停)")

            return result.success

    def ping_revise(self, distance: float):
        request = user_sensor_msgs.srv.SensorService.Request()

        request.set_revise_x = float(distance)  # x 修正设定值(cm)
        request.set_x0_comp = float(0)  # yaw 传感器0补偿参数(cm)
        request.deviation_x = 0.1  # 允许误差x(cm)
        request.deviation_w = 0.1  # 允许误差w(cm)
        request.sensor_type = 0  # 传感器类型 0: ping   1: ir
        request.correction_mode = 2  # 修正模式   0：传感器0  1：传感器1  2：传感器0 && 1
        request.speed_reversal = False
        request.start = True

        self.__logger.info(f"[传感器] 请求超声矫正.")
        self.__call_service(request)

    def ir_revise(self, distance: float):
        request = user_sensor_msgs.srv.SensorService.Request()

        request.set_revise_x = float(distance)
        request.set_x0_comp = float(0)
        request.deviation_x = 0.1
        request.deviation_w = 0.1
        request.sensor_type = 1
        request.correction_mode = 1
        request.speed_reversal = True
        request.start = True

        self.__logger.info(f"[传感器] 请求红外矫正.")
        self.__call_service(request)

    # 等待修正结束
    def wait_finish(self):
        request = user_sensor_msgs.srv.SensorService.Request()
        request.start = False

        while rclpy.ok():
            if self.__call_service(request):
                self.__logger.info(f"[传感器] 修正结束！")
                break
            time.sleep(0.2)
