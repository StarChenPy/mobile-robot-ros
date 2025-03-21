import enum


class FruitType(enum.Enum):
    RED_APPLE = "red_apple"
    YELLOW_APPLE = "yellow_apple"
    GREEN_APPLE = "green_apple"

    @classmethod
    def get_by_value(cls, value) -> 'FruitType':
        for member in cls.__members__.values():
            if member.value == value:
                return member