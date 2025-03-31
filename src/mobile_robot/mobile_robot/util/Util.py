from ..popo.FruitHeight import FruitHeight
from ..popo.IdentifyResult import IdentifyResult


def get_fruit_height(height: float) -> FruitHeight:
    if height > 40:
        return FruitHeight.LOW
    elif height > 32:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.TALL
