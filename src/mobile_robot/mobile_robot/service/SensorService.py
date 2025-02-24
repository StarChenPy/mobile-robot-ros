import rclpy

from ..dao.SensorDao import SensorDao
from ..util.Singleton import singleton


@singleton
class SensorService:
    def __init__(self, node: rclpy.node.Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__sensor = SensorDao()

    def ping_revise(self, dis: float, is_block=True):
        self.__sensor.ping_revise(dis)
        if is_block:
            self.__sensor.wait_finish()

    def ir_revise(self, dis: float, is_block=True):
        self.__sensor.ir_revise(dis)
        if is_block:
            self.__sensor.wait_finish()