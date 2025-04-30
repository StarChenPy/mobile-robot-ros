from dataclasses import dataclass

from .MotorMovement import MotorMovement
from .ServoMotor import ServoMotor


@dataclass
class ArmMovement:
    motor: MotorMovement = None
    servo: ServoMotor = None