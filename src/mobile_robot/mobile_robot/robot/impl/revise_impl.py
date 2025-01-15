import time
import rclpy

from enum import Enum
from rclpy.node import Node
from revise_msgs.msg import ReviseData


class ReviseSensorSelect(Enum):
    PING0 = 0
    PING1 = 1
    IR = 2
    PING = 3


class ReviseImpl:
    __revise_data = ReviseData()

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
    def __pub_revise(self, x, yaw, sensor_selection: ReviseSensorSelect):
        msg_pub = ReviseData()
        msg_pub.set_revise_x = float(x)
        msg_pub.set_revise_y = float(0)
        msg_pub.set_revise_yaw = float(yaw)
        msg_pub.sensor_selection = int(sensor_selection.value)
        msg_pub.start = True
        msg_pub.status = 1
        self.__revise_data_pub.publish(msg_pub)
        # time.sleep(0.5)
        # self.__revise_data_pub.publish(msg_pub)

    def ping_revise(self, dis: float, yaw: float):
        self.__pub_revise(dis, yaw, ReviseSensorSelect.PING)

    def ir_revise(self, dis: float, yaw: float):
        self.__pub_revise(dis, yaw, ReviseSensorSelect.IR)

    # 等待修正结束
    def wait_controls_end(self):
        while self.__revise_data.status != 0:
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            time.sleep(0.5)
        self.__logger.info("[矫正] 已结束")
