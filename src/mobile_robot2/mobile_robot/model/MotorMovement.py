from dataclasses import dataclass


@dataclass
class MotorMovement:
    rotate: float
    lift: float
