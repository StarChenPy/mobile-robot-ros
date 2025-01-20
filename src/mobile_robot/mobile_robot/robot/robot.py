import time
import rclpy

from rclpy.node import Node

from .impl import arm_impl, io_impl, navigation_impl, revise_impl, vision_impl
from .param.arm_movement import ArmMovementParam
from .param.arm_param import Motor
from .param.navigation_path import NavPath, Pose


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
                self.__arm.ctrl_lift_motor(arm_impl.MotorCmd.BACK_ORIGIN, is_block=is_block)  # 回原点
            else:
                self.__arm.ctrl_lift_motor(arm_impl.MotorCmd.SET_POSITION, movement.value.motor.lift, is_block=is_block)

            if movement.value.motor.rotate == -1:
                self.__arm.ctrl_rotate_motor(arm_impl.MotorCmd.BACK_ORIGIN, is_block=True)  # 回原点
                self.__arm.ctrl_rotate_motor(arm_impl.MotorCmd.SET_POSITION, 0.5, 50, is_block=is_block)
            else:
                self.__arm.ctrl_rotate_motor(arm_impl.MotorCmd.SET_POSITION, movement.value.motor.rotate, is_block=is_block)

            if not is_block:
                self.__arm.wait_motor_finish(Motor.LIFT)
                self.__arm.wait_motor_finish(Motor.ROTATE)

        if movement.value.servo is not None:
            self.__arm.telescopic_servo(movement.value.servo.telescopic)
            self.__arm.nod_servo(movement.value.servo.nod)
            self.__arm.gripper_servo(movement.value.servo.gripper)
            self.__arm.rotary_servo(movement.value.servo.rotary)

        time.sleep(2)

    def arm_reset(self):
        self.__logger.info("[机械臂] 开始复位！")
        self.arm_control(ArmMovementParam.RESET)
        self.__logger.info("[机械臂] 复位完成！")

    def navigation(self, nav_path: NavPath, speed=0.5, is_block=True, init_pose=True):
        """
        通过路径进行导航
        @param nav_path 路径列表
        @param speed 移送速度
        @param is_block 是否阻塞
        @param init_pose 若为True，路径中的第一个坐标会被视为精确点赋给odom
        """
        if init_pose:
            start_point = nav_path.value[0].value
            self.__navigation.init_pose(start_point.x, start_point.y, start_point.yaw)

        path = nav_path.value[1:]
        self.__navigation.navigation(path, speed, is_block=is_block)

    def cancel_navigation(self):
        self.__navigation.cancel_navigation()

    def get_navigation_state(self) -> bool:
        return self.__navigation.get_navigation_state()

    def move(self, distance: float, speed=0.3):
        self.__navigation.base_motion_line(distance, speed)

    def rotate(self, angle: float, speed=120):
        self.__navigation.base_motion_rotate(angle, speed)

    def ping_revise(self, dis: float, yaw: float, is_block=True):
        self.__revise.ping_revise(dis, yaw)
        if is_block:
            self.__revise.wait_controls_end()

    def ir_revise(self, dis: float, is_block=True):
        self.__revise.ir_revise(dis, 0)
        if is_block:
            self.__revise.wait_controls_end()

    def vision(self):
        self.__vision.send_mnn_request()
        return self.__vision.mnn_result()
