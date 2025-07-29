import enum


class FruitType(enum.Enum):
    RED_APPLE = "red_apple"
    YELLOW_APPLE = "yellow_apple"
    GREEN_APPLE = "green_apple"
    PURPLE_APPLE = "purple_apple"
    GREEN_GRAPE = "green_grape"
    YELLOW_GRAPE = "yellow_grape"
    PURPLE_GRAPE = "purple_grape"

    @classmethod
    def apples(cls):
        return [
            cls.RED_APPLE,
            cls.YELLOW_APPLE,
            cls.GREEN_APPLE,
            cls.PURPLE_APPLE
        ]

    @classmethod
    def grapes(cls):
        return [
            cls.GREEN_GRAPE,
            cls.YELLOW_GRAPE,
            cls.PURPLE_GRAPE
        ]

    @classmethod
    def all(cls):
        return cls.apples() + cls.grapes()
