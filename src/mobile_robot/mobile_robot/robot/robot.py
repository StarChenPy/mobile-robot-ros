import enum
import time
import rclpy

from rclpy.node import Node

from .util.data_type import SensorType, MotorCmd, NavigationPoint, CorrectivePoint
from .impl import arm_impl, io_impl, navigation_impl, revise_impl, vision_impl
from .param.arm_movement import ArmMovementParam
from .param.arm_param import Motor
from .param.navigation_path import NavPath
from .util.math import calculate_rectangle_center


class FruitHeight(enum.Enum):
    TALL = 0
    MIDDLE = 1
    LOW = 2


def get_fruit_height(height: float) -> FruitHeight:
    if height < 150:
        return FruitHeight.TALL
    elif height < 280:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.LOW


class MobileRobot:
    def __init__(self, node: Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__navigation = navigation_impl.NavigationImpl(self.__node)
        self.__arm = arm_impl.ArmImpl(self.__node)
        self.__io = io_impl.IoImpl.instance(self.__node)
        self.__revise = revise_impl.ReviseImpl(self.__node)
        self.__vision = vision_impl.VisionImpl(self.__node)

        self.__logger.info("[机器人] 等待连接")
        while rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0.2)
            if self.__io.is_connect():
                self.__logger.info("[机器人] 连接成功")
                break
            else:
                time.sleep(1)

    # 等待启动按键被按下
    def with_start_button(self):
        self.__io.set_led(False)
        self.__logger.info('[机器人] 等待按键按下!')
        # 等待按键按下
        while rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0.2)
            val = self.__io.get_start_button()

            if not val:
                self.__logger.info('[机器人] 按键按下!')
                self.__io.set_led(True)
                break
            time.sleep(0.2)

    def arm_control(self, movement: ArmMovementParam, is_block=False):
        self.__logger.info(f"[机器人] 机械臂控制 {movement.name}")
        if movement.value.motor is not None:
            if movement.value.motor.lift == -1:
                self.__arm.ctrl_lift_motor(MotorCmd.BACK_ORIGIN, is_block=is_block)  # 回原点
            else:
                self.__arm.ctrl_lift_motor(MotorCmd.SET_POSITION, movement.value.motor.lift, is_block=is_block)

            if movement.value.motor.rotate == -1:
                self.__arm.ctrl_rotate_motor(MotorCmd.BACK_ORIGIN, is_block=True)  # 回原点
                self.__arm.ctrl_rotate_motor(MotorCmd.SET_POSITION, 0.5, 50, is_block=is_block)
            else:
                self.__arm.ctrl_rotate_motor(MotorCmd.SET_POSITION, movement.value.motor.rotate, is_block=is_block)

            if not is_block:
                self.__arm.wait_motor_finish(Motor.LIFT)
                self.__arm.wait_motor_finish(Motor.ROTATE)

        if movement.value.servo is not None:
            self.__arm.telescopic_servo(movement.value.servo.telescopic)
            self.__arm.nod_servo(movement.value.servo.nod)
            self.__arm.gripper_servo(movement.value.servo.gripper)
            self.__arm.rotary_servo(movement.value.servo.rotary)

        time.sleep(1)

    def arm_reset(self):
        self.__logger.info("[机械臂] 开始复位！")
        self.arm_control(ArmMovementParam.RESET)
        self.__logger.info("[机械臂] 复位完成！")

    def init_pose(self, pose: NavigationPoint):
        self.__navigation.init_pose(pose)

    def navigation(self, nav_path: NavPath, speed=0.5, is_block=True):
        """
        通过路径进行导航
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        """

        path = []

        for point in nav_path.value:
            if isinstance(point, NavigationPoint):
                path.append(point)
            elif isinstance(point, CorrectivePoint):
                correctivePoint1 = NavigationPoint(point.x, point.y, point.yaw1)
                if path:
                    path.append(correctivePoint1)
                    self.__navigation.navigation(path, speed)
                    path = []
                    self.__navigation.wait_navigation_finish()
                self.ping_revise(point.distance1)
                self.init_pose(correctivePoint1)

                correctivePoint2 = NavigationPoint(point.x, point.y, point.yaw2)
                self.__navigation.navigation([correctivePoint2])
                self.__navigation.wait_navigation_finish()
                self.ping_revise(point.distance2)
                self.init_pose(correctivePoint2)

        self.__navigation.navigation(path, speed, is_block=is_block)


    def cancel_navigation(self):
        self.__navigation.cancel_navigation()

    def get_navigation_state(self) -> bool:
        return self.__navigation.get_navigation_state()

    def move(self, distance: float, speed=0.3):
        self.__navigation.base_motion_line(distance, speed)

    def rotate(self, angle: float, speed=120):
        self.__navigation.base_motion_rotate(angle, speed)

    def ping_revise(self, dis: float, is_block=True):
        self.__revise.revise(dis, SensorType.PING)
        if is_block:
            self.__revise.wait_revise()

    def ir_revise(self, dis: float, is_block=True):
        self.__revise.revise(dis, SensorType.IR)
        if is_block:
            self.__revise.wait_revise()

    def vision(self):
        return self.__vision.send_mnn_request()

    def grab_fruits(self, nav_path: NavPath, task: dict[int, list[str]], direction = "left" or "right") -> bool:
        """
        扫描并抓取水果，主控制流程，带篮子
        """
        self._prepare_for_recognition(direction)
        self.navigation(nav_path)

        flag = False
        while rclpy.ok() and self.get_navigation_state():
            results = self._get_valid_detections()

            for result in results:
                if self._process_single_fruit(result, task):
                    flag = True  # 遇到特殊篮子ID时提前终止当前检测循环

                if flag:
                    break
            if flag:
                break

            self._prepare_for_recognition(direction)
            self.navigation(nav_path)

        self.arm_control(ArmMovementParam.MOVING, True)
        return flag

    # region 辅助方法
    def _prepare_for_recognition(self, direction = "left" or "right"):
        """准备视觉识别状态"""
        if direction == "left":
            self.arm_control(ArmMovementParam.RECOGNITION_ORCHARD_LEFT)
        elif direction == "right":
            self.arm_control(ArmMovementParam.RECOGNITION_ORCHARD_RIGHT)

    def _get_valid_detections(self) -> list:
        """获取有效检测结果并进行初步过滤"""
        return [
            result for result in self.vision()
            if 180 < calculate_rectangle_center(result.box)[0] < 400
        ]

    def _process_single_fruit(self, result, task: dict) -> bool:
        """处理单个水果的抓取流程，返回是否需要终止检测循环"""
        # 篮子选择逻辑
        basket_id = 0
        for bid, target_classes in task.items():
            if result.classId in target_classes:
                task[bid].remove(result.classId)
                basket_id = bid
                break

        if basket_id == 0:
            return False

        # 执行核心操作序列
        self.cancel_navigation()
        self._execute_grab_sequence(result.box)
        # return self._execute_placement_sequence(basket_id)
        return True

    def _execute_grab_sequence(self, box):
        """执行抓取动作序列"""
        _, center_y = calculate_rectangle_center(box)
        match get_fruit_height(center_y):
            case FruitHeight.TALL:
                self.arm_control(ArmMovementParam.READY_GRAB_APPLE_TALL)
                self.arm_control(ArmMovementParam.GRAB_APPLE_TALL)
            case FruitHeight.MIDDLE:
                self.arm_control(ArmMovementParam.READY_GRAB_APPLE_MIDDLE)
                self.arm_control(ArmMovementParam.GRAB_APPLE_MIDDLE)
            case FruitHeight.LOW:
                self.arm_control(ArmMovementParam.READY_GRAB_APPLE_LOW)
                self.arm_control(ArmMovementParam.GRAB_APPLE_LOW)

    def _execute_placement_sequence(self, basket_id: int) -> bool:
        """执行放置动作序列，返回是否需要终止检测循环"""
        self.arm_control(ArmMovementParam.READY_PUT_FRUIT_INTO_BASKET, True)

        match basket_id:
            case 1:
                self.arm_control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_1)
            case 2:
                self.arm_control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_2)
            case 3:
                self.arm_control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_3)
            case 4:
                return True  # 特殊终止信号

        return False
