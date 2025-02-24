import ament_index_python.packages
import yaml

from .Singleton import singleton


@singleton
class Config(object):
    def __init__(self):
        share_directory = ament_index_python.packages.get_package_share_directory("mobile_robot")

        self.__config_dir = share_directory + "/config"


    def get_lift_motor_config(self):
        with open(self.__config_dir + "/lift_motor_config.yaml", 'r') as stream:
            return yaml.safe_load(stream.read())

    def get_rotate_motor_config(self):
        with open(self.__config_dir + "/rotate_motor_config.yaml", 'r') as stream:
            return yaml.safe_load(stream.read())

    def get_servo_config(self):
        with open(self.__config_dir + "/servo_config.yaml", 'r') as stream:
            return yaml.safe_load(stream.read())

