from dataclasses import dataclass

from Rectangle import Rectangle


@dataclass
class MnnResult:
    classId: str
    confidence: float
    box: Rectangle