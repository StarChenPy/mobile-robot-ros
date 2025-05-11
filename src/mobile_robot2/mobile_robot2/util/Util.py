from ..model.FruitLocationOnTree import FruitLocationOnTree
from ..model.FruitHeight import FruitHeight
from ..model.FruitType import FruitType


def get_fruit_height(height: float) -> FruitHeight:
    if height > 0.40:
        return FruitHeight.LOW
    elif height > 0.32:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.TALL

def get_fruit_location_on_tree(vision_service, fruit: FruitType) -> FruitLocationOnTree:
    """
    @param vision_service: Vision服务
    @param fruit: 水果类型
    @return 水果在树上的类型
    """
    result = vision_service.get_onnx_identify_result()

    while not result:
        result = vision_service.get_onnx_identify_result()

    for i in result:
        if i.classId != fruit.value:
            continue

        center = i.box.get_rectangle_center()

        y_threshold = 200
        if i.distance < 0.35:
            if center.y < y_threshold:
                return FruitLocationOnTree.TOP_CENTER
            else:
                return FruitLocationOnTree.BOTTOM_CENTER
        else:
            if center.x < 300:
                if center.y < y_threshold:
                    return FruitLocationOnTree.TOP_LEFT
                else:
                    return FruitLocationOnTree.BOTTOM_LEFT
            elif center.x > 340:
                if center.y < y_threshold:
                    return FruitLocationOnTree.TOP_RIGHT
                else:
                    return FruitLocationOnTree.BOTTOM_RIGHT
            else:
                return FruitLocationOnTree.TOP_CENTER


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
