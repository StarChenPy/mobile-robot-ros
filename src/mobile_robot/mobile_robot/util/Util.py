from ..popo.FruitLocationOnTree import FruitLocationOnTree
from ..popo.FruitHeight import FruitHeight
from ..popo.FruitType import FruitType
from ..popo.IdentifyResult import IdentifyResult


def get_fruit_height(height: float) -> FruitHeight:
    if height > 0.40:
        return FruitHeight.LOW
    elif height > 0.32:
        return FruitHeight.MIDDLE
    else:
        return FruitHeight.TALL


def classify_fruit_locations(data: list[IdentifyResult], fruits: list[FruitType]) -> dict[FruitLocationOnTree: FruitType]:
    if len(data) < 5:
        raise ValueError("点数量应为大于5个")

    # 定义全部枚举位置
    all_positions = {
        "左上": FruitLocationOnTree.TOP_LEFT,
        "左下": FruitLocationOnTree.BOTTOM_LEFT,
        "中上": FruitLocationOnTree.TOP_CENTER,
        "中下": FruitLocationOnTree.BOTTOM_CENTER,
        "右上": FruitLocationOnTree.TOP_RIGHT,
        "右下": FruitLocationOnTree.BOTTOM_RIGHT,
    }

    xs = [p.box.get_rectangle_center().x for p in data]
    ys = [p.box.get_rectangle_center().y for p in data]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # 三列划分
    x1 = min_x + (max_x - min_x) / 3
    x2 = min_x + (max_x - min_x) * 2 / 3

    # 两行划分
    y_mid = (min_y + max_y) / 2

    col_set = set()
    row_set = set()
    existing_positions = dict()

    for item in data:
        p = item.box.get_rectangle_center()
        # 列判断
        if p.x <= x1:
            col = "左"
        elif p.x >= x2:
            col = "右"
        else:
            col = "中"
        col_set.add(col)

        # 行判断
        row = "上" if p.y <= y_mid else "下"
        row_set.add(row)

        pos = col + row
        fruit_type = FruitType.get_by_value(item.classId)
        if pos in all_positions and fruit_type in fruits:
            existing_positions[all_positions[pos]] = fruit_type

    return existing_positions


def get_fruit_location(data: list[IdentifyResult], fruits: list[FruitType]) -> dict[FruitLocationOnTree: FruitType]:
    """
    获取水果在树上的位置
    :param data: 识别结果
    :param fruits: 需要识别的水果类型
    """

    if len(data) >= 5:
        return classify_fruit_locations(data, fruits)

    locations = {}

    for i in data:
        fruit_type = FruitType.get_by_value(i.classId)
        if fruit_type not in fruits:
            continue

        center = i.box.get_rectangle_center()

        y_threshold = 160
        if center.x < 270:
            if center.y < y_threshold:
                locations[FruitLocationOnTree.TOP_LEFT] = fruit_type
            else:
                locations[FruitLocationOnTree.BOTTOM_LEFT] = fruit_type
        elif center.x > 330:
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


def get_fruit_location_on_tree(vision_service, fruit: FruitType) -> FruitLocationOnTree:
    """
    @param vision_service: Vision服务
    @param fruit: 水果类型
    @return 水果在树上的类型
    """
    result = vision_service.get_onnx_identify_depth()

    while not result:
        result = vision_service.get_onnx_identify_depth()

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
