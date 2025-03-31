from ..popo.FruitHeight import FruitHeight


def get_fruit_height(height: float) -> FruitHeight:
    if height > 0.40:
        return FruitHeight.LOW
    elif height > 0.32:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.TALL
