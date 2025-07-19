import enum


class FruitType(enum.Enum):
    RED_APPLE = ("red_apple", 7)
    YELLOW_APPLE = ("yellow_apple", 7)
    GREEN_APPLE = ("green_apple", 7)

    def __init__(self, label, size):
        self.label = label
        self.size = size

    def __new__(cls, label, size):
        obj = object.__new__(cls)
        obj._value_ = label  # 决定 `FruitType("red_apple")` 是怎么查找的
        obj.label = label
        obj.size = size
        return obj
