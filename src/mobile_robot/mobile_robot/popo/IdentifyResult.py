from dataclasses import dataclass

from .Rectangle import Rectangle


@dataclass
class IdentifyResult:
    class_id: str
    confidence: float
    box: Rectangle
    distance: float = 0.4
