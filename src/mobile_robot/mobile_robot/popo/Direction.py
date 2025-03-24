import enum


class Direction(enum.Enum):
    FRONT = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3

    def invert(self):
        if self == Direction.FRONT:
            return Direction.BACK

        if self == Direction.BACK:
            return Direction.FRONT

        if self == Direction.LEFT:
            return Direction.RIGHT

        if self == Direction.RIGHT:
            return Direction.LEFT

        return None
