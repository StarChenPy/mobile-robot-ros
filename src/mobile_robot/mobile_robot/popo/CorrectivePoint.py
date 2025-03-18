from dataclasses import dataclass

from .Corrective import Corrective
from .NavigationPoint import NavigationPoint


@dataclass
class CorrectivePoint(NavigationPoint):
    corrective_data: list[Corrective]

    @classmethod
    def form_point(cls, point: NavigationPoint, corrective_data: list[Corrective]):
        return cls(point.x, point.y, point.yaw, corrective_data)
