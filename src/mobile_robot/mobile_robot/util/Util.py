from ..popo.FruitHeight import FruitHeight
from ..popo.Rectangle import Rectangle


def get_fruit_height(box: Rectangle) -> FruitHeight:
    area = box.get_area()

    if area > 18000:
        return FruitHeight.TALL
    elif area > 13000:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.LOW