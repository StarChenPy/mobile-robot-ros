from dataclasses import dataclass

from .Corrective import Corrective
from .NavigationPoint import NavigationPoint


@dataclass
class CorrectivePoint(NavigationPoint):
    corrective_data: list[Corrective]
