from ..popo.FruitHeight import FruitHeight


def get_fruit_height(height: float) -> FruitHeight:
    if height > 0.40:
        return FruitHeight.LOW
    elif height > 0.32:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.TALL


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
