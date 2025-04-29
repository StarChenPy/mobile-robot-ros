import ament_index_python.packages
import yaml

from .Singleton import singleton


@singleton
class Config:
    def __init__(self):
        share_directory = ament_index_python.packages.get_package_share_directory("mobile_robot")

        self.__config_dir = share_directory + "/config"
        self.__param_dir = share_directory + "/param"

    def get_lift_motor_config(self):
        with open(self.__config_dir + "/lift_motor_config.yml", 'r') as stream:
            return yaml.safe_load(stream.read())

    def get_rotate_motor_config(self):
        with open(self.__config_dir + "/rotate_motor_config.yml", 'r') as stream:
            return yaml.safe_load(stream.read())

    def get_servo_config(self):
        with open(self.__config_dir + "/servo_config.yml", 'r') as stream:
            return yaml.safe_load(stream.read())
