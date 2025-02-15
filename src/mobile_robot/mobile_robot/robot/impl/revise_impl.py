import time

import rclpy

from rclpy.node import Node
from revise_msgs.msg import ReviseData
from ..util.data_type import CorrectiveSensor


class ReviseImpl:
    __revise_data = None

    def __init__(self, node: Node):
        self.__node = node
        self.__logger = node.get_logger()

        # 发布控制数据 话题
        self.__revise_data_pub = node.create_publisher(ReviseData, '/robot/correct_cmd', 1)
        # 获取控制状态 话题
        self.__revise_data_sub = node.create_subscription(ReviseData, '/robot/correct_cmd', self.__revise_callback, 1)

    # 控制参数状态回调
    def __revise_callback(self, msg):
        self.__revise_data = msg

    # 控制状态发布
    def __pub_revise(self, distance, yaw, sensor_selection: CorrectiveSensor):
        msg_pub = ReviseData()
        msg_pub.set_revise_x = float(distance)
        msg_pub.set_revise_y = float(0)
        msg_pub.set_revise_yaw = float(yaw)
        msg_pub.sensor_selection = int(sensor_selection.value)
        msg_pub.start = True
        msg_pub.status = 1
        self.__revise_data_pub.publish(msg_pub)
        time.sleep(1)
        self.__revise_data_pub.publish(msg_pub)

    def ping_revise(self, dis: float, yaw: float):
        self.__logger.info("[矫正] 开始超声波矫正")
        self.__pub_revise(dis, yaw, CorrectiveSensor.PING)

    def ir_revise(self, dis: float, yaw: float):
        self.__logger.info("[矫正] 开始红外矫正")
        self.__pub_revise(dis, yaw, CorrectiveSensor.IR)

    # 等待修正结束
    def wait_controls_end(self):
        for i in range(0, 100):
            rclpy.spin_once(self.__node)
        while self.__revise_data.status != 0:
            rclpy.spin_once(self.__node)
        self.__logger.info("[矫正] 矫正已结束")
        time.sleep(1)
