from ..popo.Direction import Direction
from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.FruitHeight import FruitHeight
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult
from ..popo.NavigationPoint import NavigationPoint


def get_fruit_location(data: list[IdentifyResult], fruits: list[FruitType]) -> dict[FruitLocationOnTree: FruitType]:
    """
    获取水果在树上的位置
    :param data: 识别结果
    :param fruits: 需要识别的水果类型
    """

    locations = {}

    for i in data:
        fruit_type = FruitType(i.class_id)
        if fruit_type not in fruits:
            continue

        center = i.box.get_rectangle_center()

        y_threshold = 160
        x_threshold = 320
        if center.x < (x_threshold - 20):
            if center.y < y_threshold:
                locations[FruitLocationOnTree.TOP_LEFT] = fruit_type
            else:
                locations[FruitLocationOnTree.BOTTOM_LEFT] = fruit_type
        elif center.x > (x_threshold + 20):
            if center.y < y_threshold:
                locations[FruitLocationOnTree.TOP_RIGHT] = fruit_type
            else:
                locations[FruitLocationOnTree.BOTTOM_RIGHT] = fruit_type
        else:
            if center.y < y_threshold:
                locations[FruitLocationOnTree.TOP_CENTER] = fruit_type
            else:
                locations[FruitLocationOnTree.BOTTOM_CENTER] = fruit_type

    return locations


def add_blank_lines_between_top_level_blocks(yaml_content: str) -> str:
    """
    在YAML的顶层键之间插入空白行以提高可读性
    参数:
        yaml_content: 原始YAML字符串内容
    返回:
        处理后的带空行的YAML字符串
    """
    lines = yaml_content.split('\n')
    new_lines = []
    first_top_key = True

    for line in lines:
        stripped = line.strip()
        # 检测顶层键（非缩进行且包含冒号）
        if stripped and ':' in stripped and not line.startswith(' '):
            if not first_top_key:
                new_lines.append('')  # 插入空行
            else:
                first_top_key = False
            new_lines.append(line)
        else:
            new_lines.append(line)

    return '\n'.join(new_lines)
