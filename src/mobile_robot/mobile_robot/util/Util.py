from ..popo.FruitHeight import FruitHeight
from ..popo.IdentifyResult import IdentifyResult


def get_fruit_height(data: IdentifyResult) -> FruitHeight:
    # 32000-33000
    if data.distance < 0.3:
        return FruitHeight.TALL
    # 15000-16000
    elif data.distance < 0.44:
        return FruitHeight.MIDDLE
    # 9000-10000
    else:
        return FruitHeight.LOW
