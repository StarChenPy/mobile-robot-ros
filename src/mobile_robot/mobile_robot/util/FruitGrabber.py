from . import Math
from .Logger import Logger
from ..param import ArmMovement as Movement
from ..popo.ArmMovement import ArmMovement
from ..popo.Direction import Direction
from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..popo.NavigationPoint import NavigationPoint
from ..util import Util


class FruitGrabber:
    def __init__(self, robot, arm, move, sensor, vision):
        self.logger = Logger()
        self._previous_location = None
        self._current_direction = None
        self.__robot = robot
        self.__arm = arm
        self.__move = move
        self.sensor = sensor
        self.vision = vision
        # 定义区域和它们的基础参数
        self.__location_params = {
            FruitLocationOnTree.TOP_LEFT: {
                'direction': -90,  # 左侧
                'exact_direction': -95,  # 左侧
                'arm_height': 24,  # 较高位置
                'servo_angle': 0,  # 水平
                'gripper_size': 14,  # 夹爪打开距离
                'telescopic': 18,  # 伸缩距离
                'offset': 0.04  # 靠近树的偏移量
            },
            FruitLocationOnTree.BOTTOM_LEFT: {
                'direction': -90,
                'exact_direction': -83,
                'arm_height': 32,  # 较低位置
                'servo_angle': -20,  # 略向内
                'gripper_size': 12,
                'telescopic': 18,
                'offset': 0.04
            },
            FruitLocationOnTree.TOP_CENTER: {
                'direction': -90,
                'exact_direction': -90,
                'arm_height': 24,
                'servo_angle': 0,
                'gripper_size': 14,
                'telescopic': 8,
                'is_center': True,  # 中央特殊标记
                'center_offset': -0.13  # 中央位置的特殊偏移
            },
            FruitLocationOnTree.BOTTOM_CENTER: {
                'direction': -90,
                'exact_direction': -90,
                'arm_height': 32,
                'servo_angle': -175,  # 中央底部特殊角度
                'gripper_size': 14,
                'telescopic': 18,
                'is_center': True,
                'center_offset': -0.13
            },
            FruitLocationOnTree.TOP_RIGHT: {
                'direction': 90,    # 右侧
                'exact_direction': 95,
                'arm_height': 24,
                'servo_angle': 0,
                'gripper_size': 14,
                'telescopic': 18,
                'offset': 0.06
            },
            FruitLocationOnTree.BOTTOM_RIGHT: {
                'direction': 90,
                'exact_direction': 82,
                'arm_height': 32,
                'servo_angle': 30,  # 右下角特殊角度
                'gripper_size': 12,
                'telescopic': 18,
                'offset': 0.06
            }
        }
        # 定义区域组，用于优化路径
        self.__location_groups = {
            'left': [FruitLocationOnTree.TOP_LEFT, FruitLocationOnTree.BOTTOM_LEFT],
            'center': [FruitLocationOnTree.TOP_CENTER, FruitLocationOnTree.BOTTOM_CENTER],
            'right': [FruitLocationOnTree.TOP_RIGHT, FruitLocationOnTree.BOTTOM_RIGHT]
        }

    def rotation_correction(self):
        angle_by_front = self.sensor.get_angle_from_wall(Direction.FRONT)
        angle_by_right = self.sensor.get_angle_from_wall(Direction.RIGHT)
        angle_by_left = self.sensor.get_angle_from_wall(Direction.LEFT)
        angles = [angle_by_front, angle_by_right, angle_by_left]
        angles = [x for x in angles if x != 0]

        self.logger.info(f"直角矫正角度: {angles}")

        min_angle = min(angles, key=lambda x: (abs(x), -x))
        if abs(min_angle) < 15:
            self.__move.rotate(min_angle)

    def run(self, fruit_basket: dict[FruitType: int]):
        """
        根据水果位置列表执行抓取操作

        Args:
            fruit_basket: 字典，表示水果放到几号篮子
        """
        self.rotation_correction()
        Movement.recognition_orchard_tree(self.__arm)

        while True:
            result1 = self.vision.get_onnx_identify_result(True)
            result2 = self.vision.get_onnx_identify_result(True)

            if len(result1) == len(result2):
                result = result2
                break

        self.logger.info(f"识别到的水果数量{len(result)}")

        location_result = Util.get_fruit_location(result, [FruitType.RED_APPLE, FruitType.YELLOW_APPLE, FruitType.GREEN_APPLE])
        fruit_locations = location_result.keys()
        self.logger.info(f"识别到的水果位置: {fruit_locations}")

        if not fruit_locations:
            return

        # 初始化状态变量
        self._current_direction = None
        self._previous_location = None

        # 移动到初始抓取位置
        self.move_to_grab(result)

        # 优化抓取顺序
        optimized_locations = self._optimize_grab_sequence(fruit_locations)

        self.logger.info(f"优化的抓取顺序: {optimized_locations}")

        # 执行抓取
        end_local = None
        for location in optimized_locations:
            self._grab_fruit(location)
            # 放入篮子
            Movement.put_fruit_into_basket(self.__arm, fruit_basket[location_result[location]])
            end_local = location

        if end_local is not None:
            # 完成后返回初始位置
            self._return_to_initial_position(self._get_region_for_location(end_local))


    def move_to_grab(self, data: list[IdentifyResult]):
        """移动到初始抓取位置"""
        self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 10)))

        # 获取深度数据最小的水果
        min_depth_fruit = min(data, key=lambda x: x.distance)
        fruit_distance = min_depth_fruit.distance
        center = min_depth_fruit.box.get_rectangle_center()

        self.logger.info(f"最近的水果为: {min_depth_fruit.classId}, 中心点: {min_depth_fruit.box.get_rectangle_center()}, 深度: {fruit_distance}")

        if center.x < 240 or center.x > 360:
            fruit_distance = 0.36
            self.logger.info(f"需要前往抓取的前进距离: {fruit_distance}")
            self.__move.line(fruit_distance)
        else:
            if fruit_distance == 0:
                fruit_distance = 0.36
            else:
                fruit_distance += 0.11
            offset_distance = Math.pixel_to_horizontal_distance_x((320 - center.x), fruit_distance)
            rotate_yaw = Math.calculate_right_triangle_angle(offset_distance, fruit_distance + 0.24)
            self.logger.info(f"需要前往抓取的旋转角度: {rotate_yaw}")
            self.logger.info(f"需要前往抓取的前进距离: {fruit_distance}")
            # self.__move.rotate(rotate_yaw)
            # self.__move.line(fruit_distance)
            # self.__move.rotate(-rotate_yaw)
            odom_data = self.sensor.get_odom_data()
            self.__move.navigation([Math.get_target_coordinate(NavigationPoint(odom_data.x, odom_data.y, odom_data.w + rotate_yaw), fruit_distance)], 0.2)

    def _optimize_grab_sequence(self, fruit_locations):
        """
        优化抓取顺序，按区域分组并安排顺序

        原则:
        1. 将水果按位置分为左侧、中央、右侧
        2. 优先处理有多个水果的区域
        3. 如果一个区域只有一个水果，最后处理
        """
        if not fruit_locations:
            return []

        # 按区域对水果位置进行分组
        grouped_locations = {
            'left': [loc for loc in fruit_locations if loc in self.__location_groups['left']],
            'center': [loc for loc in fruit_locations if loc in self.__location_groups['center']],
            'right': [loc for loc in fruit_locations if loc in self.__location_groups['right']]
        }

        # 按数量排序，数量多的优先，相同时按center, right, left顺序
        sorted_groups = sorted(
            grouped_locations.items(),
            key=lambda x: (-len(x[1]), ['center', 'right', 'left'].index(x[0]) if len(x[1]) > 0 else 999)
        )

        # 对每个组内的水果进行排序（上下位置）
        result = []
        for group_name, locations in sorted_groups:
            if not locations:
                continue

            # 在每个组内部，先处理上面的水果，再处理下面的水果
            sorted_locations = sorted(locations, key=lambda x: x.value % 2)  # 偶数值是上面的位置
            result.extend(sorted_locations)

        return result

    def _get_region_for_location(self, location):
        """获取位置所属的区域"""
        if location is None:
            return None

        for region, locations in self.__location_groups.items():
            if location in locations:
                return region

        return None

    def _return_to_initial_position(self, location):
        """返回初始位置"""
        # 如果机械臂不在水平位置，先平移
        if self._current_direction is not None and self._current_direction != 0:
            # 先收回手臂到安全高度
            # self.__arm.control(ArmMovement(MotorMovement(self._current_direction, 10), ServoMotor(0, 0, 0, 10)))
            # 回正手臂方向
            self.__arm.control(ArmMovement(MotorMovement(0, 10), ServoMotor(0, 0, 0, 10)))
        if location == "center":
            # 如果当前区域是中央，先回到初始位置
            self.__move.line(-self.__location_params[FruitLocationOnTree.TOP_CENTER]["center_offset"])
            self.__move.rotate(-90)
            self.__move.line(0.1)
        else:
            # 返回抓取起始位置
            self.__move.line(-0.03)  # 适当调整移动距离
            # 旋转机器人回到初始朝向
            self.__move.rotate(self._current_direction)

    def _grab_fruit(self, location):
        """根据位置抓取水果"""
        params = self.__location_params[location]
        is_center = params.get('is_center', False)
        direction = params['direction']

        # 获取当前和上一个位置的区域信息
        current_region = self._get_region_for_location(location)
        previous_location = getattr(self, '_previous_location', None)
        previous_region = self._get_region_for_location(previous_location) if previous_location else None

        # 如果区域发生变化，需要回到初始位置再进行新区域的移动
        if previous_region is not None and previous_region != current_region:
            self._return_to_initial_position(previous_region)
            # 重置当前方向，因为已回到初始位置
            self._current_direction = None
        else:
            pass

        # 记录当前位置为上一个位置，供下次使用
        self._previous_location = location

        # 记录当前手臂方向，用于优化移动
        current_direction = getattr(self, '_current_direction', None)

        if current_direction != direction:
            # 中央位置的特殊处理
            if is_center:
                self.__move.line(-0.1)
                self.__arm.control(ArmMovement(MotorMovement(direction, 10), ServoMotor(0, 0, 0, params['gripper_size'])), is_block=False)
                self.__move.rotate(90)
                self.rotation_correction()
                self.__move.line(params['center_offset'])
            else:
                self.__arm.control(ArmMovement(MotorMovement(direction, 10), ServoMotor(0, 0, 0, 10)), is_block=False)
                self.__move.rotate(-direction)  # 旋转角度与方向相反
                self.rotation_correction()
                # 移动到适当位置
                self.__move.line(params['offset'])
                # 保存当前方向

            self._current_direction = direction

        # 执行抓取动作
        # 初始抬起手臂
        exact_direction = params.get('exact_direction', direction)

        self.__arm.control(ArmMovement(MotorMovement(direction, 10), ServoMotor(0, 0, 4, params['gripper_size'])))

        if location == FruitLocationOnTree.BOTTOM_CENTER:
            # 底部中央特殊处理
            self.__arm.control(ArmMovement(MotorMovement(direction, 10), ServoMotor(params['servo_angle'], 0, 4, params['gripper_size'])))

        # 调整手臂高度到抓取位置
        self.__arm.control(ArmMovement(MotorMovement(exact_direction, params['arm_height']),
                                       ServoMotor(params['servo_angle'], 0, 4, params['gripper_size'])))

        # 开爪准备抓取
        self.__arm.control(ArmMovement(servo=ServoMotor(params['servo_angle'], 0, params["telescopic"], params['gripper_size'])))

        # 降低抓爪高度
        self.__arm.control(ArmMovement(servo=ServoMotor(params['servo_angle'], 0, params["telescopic"], 6.5)))

        # 处理特殊情况
        if location == FruitLocationOnTree.BOTTOM_CENTER:
            # 底部中央特殊处理
            self.__arm.control(ArmMovement(servo=ServoMotor(params['servo_angle'], 0, 0, 6.5)))
            self.__arm.control(ArmMovement(MotorMovement(-105, 32), ServoMotor(params['servo_angle'], 0, 0, 6.5)))
            self.__arm.control(ArmMovement(MotorMovement(-105, 10), ServoMotor(0, 0, 0, 6.5)))
        else:
            # 一般情况
            self.__arm.control(ArmMovement(servo=ServoMotor(params['servo_angle'], 0, 0, 6.5)))

        # 恢复手臂位置
        self.__arm.control(ArmMovement(MotorMovement(direction, 10), ServoMotor(0, 0, 0, 6.5)))
