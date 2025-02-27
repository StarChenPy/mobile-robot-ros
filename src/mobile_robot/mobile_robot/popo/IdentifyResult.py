from dataclasses import dataclass

from .Rectangle import Rectangle


@dataclass
class IdentifyResult:
    classId: str
    confidence: float
    box: Rectangle
