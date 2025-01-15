import time
import rclpy

from rclpy.node import Node

from .impl import arm_impl, io_impl, navigation_impl, revise_impl
from .param.arm_movement import ArmMovementParam
from .param.navigation_path import NavPath


class MobileRobot:
    def __init__(self, node: Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.__navigation = navigation_impl.NavigationImpl(self.__node)
        self.__arm = arm_impl.ArmImpl(self.__node)
        self.__io = io_impl.IoImpl.instance(self.__node)
        self.__revise = revise_impl.ReviseImpl(self.__node)

        self.__logger.info("[机器人] 等待连接")
        while rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0.1)
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

            self.__logger.debug(f"[机器人] 开始按钮的值为: {val}")

            if not val:
                self.__logger.info('[机器人] 按键按下!')
                self.__io.set_led(True)
                break
            time.sleep(0.2)

    def arm_control(self, movement: ArmMovementParam):
        self.__logger.info(f"[机器人] 机械臂控制 {movement.name}")
        if movement.value.motor is not None:
            if movement.value.motor.lift == 0:
                self.__arm.ctrl_lift_motor(arm_impl.RotateMotorCmd.BACK_ORIGIN)  # 回原点
            else:
                self.__arm.ctrl_lift_motor(arm_impl.RotateMotorCmd.SET_POSITION, movement.value.motor.lift)

            if movement.value.motor.rotate == 0:
                self.__arm.ctrl_rotate_motor(arm_impl.RotateMotorCmd.BACK_ORIGIN)  # 回原点
                self.__arm.ctrl_rotate_motor(arm_impl.RotateMotorCmd.SET_POSITION, 0.5, 50)
            else:
                self.__arm.ctrl_rotate_motor(arm_impl.RotateMotorCmd.SET_POSITION, movement.value.motor.rotate)

        if movement.value.servo is not None:
            self.__arm.telescopic_servo(ArmMovementParam.RESET.value.servo.telescopic)
            self.__arm.nod_servo(ArmMovementParam.RESET.value.servo.nod)
            self.__arm.gripper_servo(ArmMovementParam.RESET.value.servo.gripper)
            self.__arm.rotary_servo(ArmMovementParam.RESET.value.servo.rotary)

    def arm_reset(self):
        self.__logger.info("[机械臂] 开始复位！")
        self.arm_control(ArmMovementParam.RESET)
        time.sleep(1)
        self.arm_control(ArmMovementParam.MOVING)
        self.__logger.info("[机械臂] 复位完成！")

    def navigation(self, nav_path: NavPath):
        start_point = nav_path.value[0]
        self.__navigation.init_pose(start_point[0], start_point[1], start_point[2])

        path = nav_path.value[1:]
        self.__navigation.navigation(path)

    def ping_revise(self, dis: float, yaw: float):
        self.__revise.ping_revise(dis, yaw)

    def ir_revise(self, dis: float, yaw: float):
        self.__revise.ir_revise(dis, yaw)
