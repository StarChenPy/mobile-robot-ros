import time
import rclpy

from rclpy.node import Node

from .impl import arm_impl, io_impl, navigation_impl, vision_impl
from .param import robot_param


class MobileRobot:
    def __init__(self, node: Node):
        self.__node = node
        self.__logger = node.get_logger()

        self.navigation = navigation_impl.NavigationImpl(self.__node)
        self.arm = arm_impl.ArmImpl(self.__node)
        self.io = io_impl.IoImpl.instance(self.__node)
        self.camera = vision_impl.CameraImpl(self.__node)

        self.__logger.info("[机器人] 等待连接")
        while rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            if self.io.is_connect():
                self.__logger.info("[机器人] 连接成功")
                break
            else:
                time.sleep(1)

    # 等待启动按键被按下
    def with_start_button(self):
        self.io.set_led(False)
        self.__logger.info('[机器人] 等待按键按下!')
        # 等待按键按下
        while rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            val = self.io.get_start_button()

            self.__logger.debug(f"[机器人] 开始按钮的值为: {val}")

            if not val:
                self.__logger.info('[机器人] 按键按下!')
                self.io.set_led(True)
                break
            time.sleep(0.2)

    def arm_reset(self):
        self.__logger.info("[机械臂] 开始复位！")
        # 升降电机回原点
        self.arm.ctrl_lift_motor(arm_impl.RotateMotorCmd.BACK_ORIGIN)  # 回原点
        self.arm.ctrl_lift_motor(arm_impl.RotateMotorCmd.SET_POSITION, 0.5, 50) # 升降控制

        # 旋转电机回原点
        self.arm.ctrl_rotate_motor(arm_impl.RotateMotorCmd.BACK_ORIGIN)  # 回原点

        self.arm.telescopic_servo(robot_param.TelescopicParams.复位.value) # 伸缩控制
        self.arm.nod_servo(robot_param.GripperRyParams.复位.value) # 摆臂控制
        self.arm.gripper_servo(robot_param.GripperParams.复位.value) # 卡爪控制
        self.arm.rotary_servo(robot_param.GripperRzParams.复位.value) # 旋臂控制

        self.__logger.info("[机械臂] 复位完成！")
