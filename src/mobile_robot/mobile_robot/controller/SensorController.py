import rclpy

from ..service.SensorService import SensorService
from ..util.Singleton import singleton


@singleton
class SensorController:
    def __init__(self, node: rclpy.node.Node):
        self.__logger = node.get_logger()

        self.__sensor = SensorService()

    def ping_revise(self, dis: float, is_block=True):
        self.__sensor.ping_revise(dis, is_block)

    def ir_revise(self, dis: float, is_block=True):
        self.__sensor.ir_revise(dis, is_block)