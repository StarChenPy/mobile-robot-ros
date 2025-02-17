import enum
import time
import rclpy

from rclpy.node import Node

from .util.data_type import CorrectiveSensor, MotorCmd, Pose
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

    def init_pose(self, pose: Pose):
        self.__navigation.init_pose(pose)

    def navigation(self, nav_path: NavPath, speed=0.4, is_block=True):
        """
        通过路径进行导航
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        """

        path1 = []

        for path in nav_path.value:
            corrective = path.corrective

            # 如果该导航点需要矫正
            if corrective is not None:
                # 如果已经有一条路径待出发，就先出发
                if path1:
                    print(path1)
                    self.__navigation.navigation(path1, speed)
                    path1 = []

                # 矫正
                if corrective.sensor == CorrectiveSensor.PING:
                    self.ping_revise(corrective.distance)
                elif corrective.sensor == CorrectiveSensor.IR:
                    self.ir_revise(corrective.distance)
                self.init_pose(path.pose)
            # 如果不需要矫正，说明是普通路径点，加入列表以待导航
            else:
                path1.append(path.pose)

        # 路径结束，将剩余导航走完
        if path1:
            self.__navigation.navigation(path1, speed, is_block=is_block)

    def cancel_navigation(self):
        self.__navigation.cancel_navigation()

    def get_navigation_state(self) -> bool:
        return self.__navigation.get_navigation_state()

    def move(self, distance: float, speed=0.3):
        self.__navigation.base_motion_line(distance, speed)

    def rotate(self, angle: float, speed=120):
        self.__navigation.base_motion_rotate(angle, speed)

    def ping_revise(self, dis: float, is_block=True):
        self.__revise.ping_revise(dis, 0.1)
        if is_block:
            self.__revise.wait_controls_end()

    def ir_revise(self, dis: float, is_block=True):
        self.__revise.ir_revise(dis, 0.1)
        if is_block:
            self.__revise.wait_controls_end()

    def vision(self):
        self.__vision.send_mnn_request()
        return self.__vision.mnn_result()

    def grab_fruits(self, nav_path: NavPath, task: dict[int, list[str]]):
        """
        扫描并抓取水果
        """
        self.arm_control(ArmMovementParam.READY_RECOGNITION_ORCHARD)
        self.arm_control(ArmMovementParam.RECOGNITION_ORCHARD)

        self.navigation(nav_path, 0.05, False)

        while rclpy.ok() and self.get_navigation_state():
            result_list = self.vision()
            for result in result_list:
                center_x, center_y = calculate_rectangle_center(result.box)

                if not 180 < center_x < 400:
                    continue

                select = 0
                for key in task:
                    if result.classId not in task[key]:
                        continue
                    else:
                        select = key
                        task[key].remove(result.classId)
                        break

                if select == 0:
                    continue

                self.cancel_navigation()

                # 调用不同高度下的抓取程序
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

                # 收回夹爪，放水果到框子里
                match select:
                    case 1:
                        self.arm_control(ArmMovementParam.READY_PUT_FRUIT_INTO_BASKET, True)
                        self.arm_control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_1)
                    case 2:
                        self.arm_control(ArmMovementParam.READY_PUT_FRUIT_INTO_BASKET, True)
                        self.arm_control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_2)
                    case 3:
                        self.arm_control(ArmMovementParam.READY_PUT_FRUIT_INTO_BASKET, True)
                        self.arm_control(ArmMovementParam.PUT_FRUIT_INTO_BASKET_3)
                    case 4:
                        break

                self.arm_control(ArmMovementParam.READY_RECOGNITION_ORCHARD)
                self.arm_control(ArmMovementParam.RECOGNITION_ORCHARD)

                self.navigation(nav_path, 0.05, False)

        self.arm_control(ArmMovementParam.MOVING, True)
