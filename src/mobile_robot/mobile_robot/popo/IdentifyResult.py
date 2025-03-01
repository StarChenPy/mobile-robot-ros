from dataclasses import dataclass

from .FruitType import FruitType
from .Rectangle import Rectangle


@dataclass
class IdentifyResult:
    fruit_type: FruitType
    confidence: float
    box: Rectangle
