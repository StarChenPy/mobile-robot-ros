import enum


class FruitType(enum.Enum):
    RED_APPLE = "red_apple"
    YELLOW_APPLE = "yellow_apple"
    GREEN_APPLE = "green_apple"

    @classmethod
    def by_value(cls, value):
        for member in cls.__members__.values():
            if member.value == value:
                return member
        raise ValueError(f"未找到具有值的成员 {value}")
