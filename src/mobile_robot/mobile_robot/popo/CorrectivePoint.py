from dataclasses import dataclass


@dataclass
class CorrectivePoint:
    x: float
    y: float
    yaw1: float
    distance1: float
    yaw2: float = 0
    distance2: float = 0
