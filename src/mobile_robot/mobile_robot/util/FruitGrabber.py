from . import Math
from ..param import ArmMovement as Movement
from ..popo.ArmMovement import ArmMovement
from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.MotorMovement import MotorMovement
from ..popo.ServoMotor import ServoMotor
from ..util import Util


class FruitGrabber:
    def __init__(self, robot, arm, move):
        self._previous_location = None
        self._current_direction = None
        self.__robot = robot
        self.__arm = arm
        self.__move = move
        # 定义区域和它们的基础参数
        self.__location_params = {
            FruitLocationOnTree.TOP_LEFT: {
                'direction': -90,  # 左侧
                'exact_direction': -95,  # 左侧
                'arm_height': 26,  # 较高位置
                'servo_angle': 0,  # 水平
                'gripper_size': 14,  # 夹爪打开距离
                'telescopic': 15,  # 伸缩距离
                'offset': 0.05  # 靠近树的偏移量
            },
            FruitLocationOnTree.BOTTOM_LEFT: {
                'direction': -90,
                'exact_direction': -83,
                'arm_height': 32,  # 较低位置
                'servo_angle': -20,  # 略向内
                'gripper_size': 10,
                'telescopic': 14,
                'offset': 0.05
            },
            FruitLocationOnTree.TOP_CENTER: {
                'direction': -90,
                'exact_direction': -90,
                'arm_height': 26,
                'servo_angle': 0,
                'gripper_size': 14,
                'telescopic': 8,
                'is_center': True,  # 中央特殊标记
                'center_offset': -0.12  # 中央位置的特殊偏移
            },
            FruitLocationOnTree.BOTTOM_CENTER: {
                'direction': -90,
                'exact_direction': -90,
                'arm_height': 32,
                'servo_angle': -175,  # 中央底部特殊角度
                'gripper_size': 14,
                'telescopic': 15,
                'is_center': True,
                'center_offset': -0.12
            },
            FruitLocationOnTree.TOP_RIGHT: {
                'direction': 90,    # 右侧
                'exact_direction': 95,
                'arm_height': 26,
                'servo_angle': 0,
                'gripper_size': 14,
                'telescopic': 15,
                'offset': 0.05
            },
            FruitLocationOnTree.BOTTOM_RIGHT: {
                'direction': 90,
                'exact_direction': 81,
                'arm_height': 32,
                'servo_angle': 40,  # 右下角特殊角度
                'gripper_size': 10,
                'telescopic': 15,
                'offset': 0.05
            }
        }
        # 定义区域组，用于优化路径
        self.__location_groups = {
            'left': [FruitLocationOnTree.TOP_LEFT, FruitLocationOnTree.BOTTOM_LEFT],
            'center': [FruitLocationOnTree.TOP_CENTER, FruitLocationOnTree.BOTTOM_CENTER],
            'right': [FruitLocationOnTree.TOP_RIGHT, FruitLocationOnTree.BOTTOM_RIGHT]
        }

    def run(self, data: list[IdentifyResult], fruit_basket: dict[FruitType: int]):
        """
        根据水果位置列表执行抓取操作

        Args:
            data: 列表，表示识别到的的水果
            fruit_basket: 字典，表示水果放到几号篮子
        """
        result = Util.get_fruit_location(data, [FruitType.RED_APPLE, FruitType.YELLOW_APPLE, FruitType.GREEN_APPLE])
        fruit_locations = result.keys()

        if not fruit_locations:
            return

        # 初始化状态变量
        self._current_direction = None
        self._previous_location = None

        # 移动到初始抓取位置
        self.move_to_grab(data)

        # # 优化抓取顺序
        # optimized_locations = self._optimize_grab_sequence(fruit_locations)
        #
        # # 执行抓取
        # end_local = None
        # for location in optimized_locations:
        #     self._grab_fruit(location)
        #     # 放入篮子
        #     Movement.put_fruit_into_basket(self.__arm, fruit_basket[result[location]])
        #     end_local = location
        #
        # if end_local is not None:
        #     # 完成后返回初始位置
        #     self._return_to_initial_position(self._get_region_for_location(end_local))


    def move_to_grab(self, data: list[IdentifyResult]):
        """移动到初始抓取位置"""
        self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 10)))

        # 获取深度数据最小的水果
        min_depth_fruit = min(data, key=lambda x: x.distance)
        center = min_depth_fruit.box.get_rectangle_center()
        offset_distance = Math.pixel_to_horizontal_distance_x((320 - center.x), min_depth_fruit.distance)
        rotate_yaw = Math.calculate_right_triangle_angle(offset_distance, min_depth_fruit.distance + 0.24)
        self.__move.rotate(rotate_yaw)
        self.__move.line(0.37)
        self.__move.rotate(-rotate_yaw)

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
            self.__arm.control(ArmMovement(MotorMovement(self._current_direction, 3), ServoMotor(0, 0, 0, 10)))
            # 回正手臂方向
            self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 10)))
        if location == "center":
            # 如果当前区域是中央，先回到初始位置
            self.__move.line(0.12)
            self.__move.rotate(-90)
            self.__move.line(0.1)
        else:
            # 返回抓取起始位置
            self.__move.line(-0.05)  # 适当调整移动距离
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
                self.__arm.control(ArmMovement(MotorMovement(direction, 3), ServoMotor(0, 0, 0, params['gripper_size'])), is_block=False)
                self.__move.rotate(90)
                self.__move.line(params['center_offset'])
            else:
                self.__arm.control(ArmMovement(MotorMovement(direction, 3), ServoMotor(0, 0, 0, 10)), is_block=False)
                self.__move.rotate(-direction)  # 旋转角度与方向相反
                # 移动到适当位置
                self.__move.line(params['offset'])
                # 保存当前方向

            self._current_direction = direction

        # 执行抓取动作
        # 初始抬起手臂
        exact_direction = params.get('exact_direction', direction)

        self.__arm.control(ArmMovement(MotorMovement(direction, 3), ServoMotor(0, 0, 0, params['gripper_size'])))

        # 调整手臂高度到抓取位置
        self.__arm.control(ArmMovement(MotorMovement(exact_direction, params['arm_height']),
                                       ServoMotor(params['servo_angle'], 0, 0, params['gripper_size'])))

        # 开爪准备抓取
        self.__arm.control(ArmMovement(servo=ServoMotor(params['servo_angle'], 0, params["telescopic"], params['gripper_size'])))

        # 降低抓爪高度
        self.__arm.control(ArmMovement(servo=ServoMotor(params['servo_angle'], 0, params["telescopic"], 6.5)))

        # 处理特殊情况
        if location == FruitLocationOnTree.BOTTOM_CENTER:
            # 底部中央特殊处理
            self.__arm.control(ArmMovement(servo=ServoMotor(params['servo_angle'], 0, 0, 6.5)))
            self.__arm.control(ArmMovement(MotorMovement(-110, 32), ServoMotor(params['servo_angle'], 0, 0, 6.5)))
            self.__arm.control(ArmMovement(MotorMovement(-110, 3), ServoMotor(0, 0, 0, 6.5)))
        else:
            # 一般情况
            self.__arm.control(ArmMovement(servo=ServoMotor(params['servo_angle'], 0, 0, 6.5)))

        # 恢复手臂位置
        self.__arm.control(ArmMovement(MotorMovement(direction, 3), ServoMotor(0, 0, 0, 6.5)))
        self.__arm.control(ArmMovement(MotorMovement(0, 3), ServoMotor(0, 0, 0, 6.5)))