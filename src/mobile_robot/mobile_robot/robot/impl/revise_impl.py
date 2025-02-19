import time

import rclpy

from rclpy.node import Node
from user_sensor_msgs.srv import SensorService
from ..util.data_type import SensorType


class ReviseImpl:
    def __init__(self, node: Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.user_sensor_client = self.__node.create_client(SensorService,'/user/sensor_service_cmd')

        self.__logger.info("[修正控制接口] 初始化完成.")

    # 发送修正控制请求
    def __call_user_sensor(self, request: SensorService.Request) -> bool:
        future = self.user_sensor_client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self.__node)

            if not future.done():
                continue

            result = future.result()

            if result.err_data != 0:
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

    def revise(self, distance, sensor_type: SensorType):
        request = SensorService.Request()
        request.set_revise_x = float(distance)      # x 修正设定值(cm)
        request.set_x0_comp = float(0)        # yaw 传感器0补偿参数(cm)
        request.deviation_x = 0.3           # 允许误差x(cm)
        request.deviation_w = 0.25          # 允许误差w(cm)

        request.sensor_type = int(0 if sensor_type == SensorType.PING0 or SensorType.PING1 or SensorType.PING else 1)          # 传感器类型 0: ping   1: ir
        if sensor_type == SensorType.PING0:
            request.correction_mode = int(0)  # 修正模式   0：传感器0  1：传感器1  2：传感器0 && 1
        elif sensor_type == SensorType.PING1 or sensor_type == SensorType.IR:
            request.correction_mode = int(1)
        elif sensor_type == SensorType.PING:
            request.correction_mode = int(2)

        if sensor_type == SensorType.PING0 or SensorType.PING1 or SensorType.PING:
            request.speed_reversal = False
        else: request.speed_reversal = True     # 修正速度取反

        request.start = True

        self.__call_user_sensor(request)

    # 等待修正结束
    def wait_revise(self):
        request = SensorService.Request()
        request.start = False

        while rclpy.ok():
            time.sleep(0.2)
            if self.__call_user_sensor(request):
                self.__logger.info(f"[修正控制接口] 修正结束！")
                break
