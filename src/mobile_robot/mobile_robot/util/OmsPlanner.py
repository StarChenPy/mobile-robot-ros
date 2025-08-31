import asyncio

import rclpy.node

from .Logger import Logger
from .Singleton import singleton
from ..dao.LiftMotorDao import LiftMotorDao
from ..dao.RobotCtrlDao import RobotCtrlDao
from ..dao.RotateMotorDao import RotateMotorDao
from ..popo.OmsGoal import OmsGoal
from ..popo.Servo import Servo


@singleton
class OmsPlanner:
    def __init__(self, node: rclpy.node.Node):
        self.logger = Logger()

        self.robot_ctrl = RobotCtrlDao(node)
        self.rotary_motor = RotateMotorDao(node)
        self.lift_motor = LiftMotorDao(node)
        self.loop = asyncio.get_event_loop()
        self.task = None

    async def plan(self, goal: OmsGoal, speed=70):
        self.logger.info(f"开始运动到目标点: {goal}")

        # 电机控制
        if goal.motor_rotary is not None:
            self.rotary_motor.ctrl_motor(goal.motor_rotary, speed)

        if goal.motor_lift is not None:
            self.lift_motor.ctrl_motor(goal.motor_lift, speed)

        # 舵机控制（不需要等待）
        if goal.servo_rotary is not None:
            self.robot_ctrl.write_pwm_no_pub(Servo.ROTARY, goal.servo_rotary)

        if goal.servo_nod is not None:
            self.robot_ctrl.write_pwm_no_pub(Servo.NOD, goal.servo_nod)

        if goal.servo_telescopic is not None:
            self.robot_ctrl.write_pwm_no_pub(Servo.TELESCOPIC, goal.servo_telescopic)

        if goal.servo_gripper is not None:
            self.robot_ctrl.write_pwm_no_pub(Servo.GRIPPER, goal.servo_gripper)

        # 统一下发命令
        self.robot_ctrl.publish()

    def plan_list(self, goals: list[OmsGoal], speed=70, block=True):
        async def run_plan():
            for goal in goals:
                await self.plan(goal, speed)
                self.wait_once_finish()
        self.task = self.loop.create_task(run_plan())

        if block:
            self.wait_plan_finish()

    def plan_once(self, goal: OmsGoal, speed=70, block=True):
        self.loop.run_until_complete(self.plan(goal, speed))
        if block:
            self.wait_once_finish()

    def wait_plan_finish(self):
        if self.task:
            self.loop.run_until_complete(self.task)
            self.task = None
        else:
            self.logger.warn("没有任务在执行")

    def wait_once_finish(self):
        self.lift_motor.wait_finish()
        self.rotary_motor.wait_finish()
