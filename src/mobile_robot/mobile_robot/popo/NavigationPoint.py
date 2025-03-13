from dataclasses import dataclass


@dataclass
class NavigationPoint:
    x: float
    y: float
    yaw: float | None
