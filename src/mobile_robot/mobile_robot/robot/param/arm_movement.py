from dataclasses import dataclass
from enum import Enum


@dataclass
class MotorMovement:
    rotate: float
    lift: float


@dataclass
class ServoMotor:
    rotary: float
    nod: float
    telescopic: float
    gripper: float


@dataclass
class ArmMovement:
    motor: MotorMovement | None
    servo: ServoMotor | None


class ArmMovementParam(Enum):
    RESET = ArmMovement(MotorMovement(0, 0), ServoMotor(0, 0, 0, 3.5))
