import ament_index_python.packages
import yaml

from . import Util
from .Singleton import singleton
from ..popo.Corrective import Corrective
from ..popo.CorrectivePoint import CorrectivePoint
from ..popo.Direction import Direction
from ..popo.NavigationPoint import NavigationPoint


@singleton
class ConfigAndParam:
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

    def get_navigation_point(self, point_name: str) -> NavigationPoint:
        """
        从参数文件中获取导航点与矫正点
        """
        with open(self.__param_dir + "/navigation_point.yml", 'r') as stream:
            points_for_param = yaml.safe_load(stream.read())

            if points_for_param is None:
                raise ValueError("文件为空")

            point_for_param = points_for_param.get(point_name)

            if point_for_param is None:
                raise ValueError(f"不存在的导航点: {point_name}")

            if point_for_param.get("x") is None or point_for_param.get("y") is None:
                raise ValueError(f"不完整的导航点: {point_for_param}")

            if point_for_param.get("corrective_data") is None:
                return NavigationPoint(point_for_param.get("x"), point_for_param.get("y"), point_for_param.get("yaw"))
            else:
                if point_for_param.get("yaw") is None:
                    raise ValueError(f"不完整的矫正点: {point_for_param}")

                corrective_data = []
                for i in point_for_param.get("corrective_data"):
                    direction = Direction.get_by_value(i.get("direction"))
                    if direction is None:
                        raise ValueError(f"错误的矫正数据: {point_for_param}")
                    corrective_data.append(Corrective(direction, i.get("distance")))

                return CorrectivePoint(point_for_param.get("x"), point_for_param.get("y"), point_for_param.get("yaw"), corrective_data)

    def set_navigation_point(self, point_name, navigation_point: NavigationPoint):
        """
        将导航点写入到yaml文件中
        """
        with open(self.__param_dir + "/navigation_point.yaml", 'r') as stream:
            data_for_file = yaml.safe_load(stream.read())

        if not isinstance(data_for_file, dict):
            data_for_file = {}

        with open(self.__param_dir + "/navigation_point.yaml", 'w') as stream:
            point = {"x": navigation_point.x, "y": navigation_point.y, "yaw": navigation_point.yaw}

            if isinstance(navigation_point, CorrectivePoint):
                corrective_data = []
                for i in navigation_point.corrective_data:
                    corrective_data.append({"direction": i.direction.value, "distance": i.distance})
                point["corrective_data"] = corrective_data

            data_for_file[point_name] = point

            # 生成原始YAML内容
            raw_yaml = yaml.safe_dump(data_for_file)

            # 通过函数添加空白行
            formatted_yaml = Util.add_blank_lines_between_top_level_blocks(raw_yaml)

            stream.write(formatted_yaml)
