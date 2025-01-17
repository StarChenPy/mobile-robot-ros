from dataclasses import dataclass
from enum import Enum


@dataclass
class MotorMovement:
    rotate: float
    lift: float


@dataclass
class ServoMotor:
    rotary: float  # 原 gripper_rz
    nod: float  # 原 gripper_ry
    telescopic: float
    gripper: float


@dataclass
class ArmMovement:
    motor: MotorMovement | None
    servo: ServoMotor | None


class ArmMovementParam(Enum):
    # 基础动作
    RESET = ArmMovement(MotorMovement(-1, -1), ServoMotor(0, 0, 0, 3.5))
    MOVING = ArmMovement(MotorMovement(0, 15), ServoMotor(0, 9, 0, 10))

    # 抓篮子用
    READY_GRAB_BASKET = ArmMovement(MotorMovement(90, 0.5), ServoMotor(0, 0, 10, 24))
    GRAB_BASKET = ArmMovement(None, ServoMotor(0, 0, 10, 19))

    # 放篮子动作
    READY_PUT_BASKET = ArmMovement(MotorMovement(0, 0.5), ServoMotor(90, -90, 10, 19))
    PUT_BASKET_CENTER = ArmMovement(MotorMovement(0, 10), ServoMotor(90, -90, 10, 24))

