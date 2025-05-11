from dataclasses import dataclass

from .Direction import Direction


@dataclass
class Corrective:
    direction: Direction
    distance: float
