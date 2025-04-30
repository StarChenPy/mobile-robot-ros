import enum

class Direction(enum.Enum):
    FRONT = "front"
    BACK = "back"
    LEFT = "left"
    RIGHT = "right"

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

    @classmethod
    def get_by_value(cls, value) -> 'Direction':
        for member in cls.__members__.values():
            if member.value == value:
                return member
