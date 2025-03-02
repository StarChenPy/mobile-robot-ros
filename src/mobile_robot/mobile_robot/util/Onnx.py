import cv2
import numpy as np
import onnxruntime as ort

from ..popo.IdentifyResult import IdentifyResult
from ..popo.Rectangle import Rectangle


names = ["red_apple", "green_apple", "yellow_apple"]


def __preprocess(image: np.ndarray, target_size: int = 320) -> tuple[np.ndarray, tuple]:
    """预处理图像并返回处理后的张量和元信息"""
    # BGR图像转换为RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    original_h, original_w = image.shape[:2]

    # 计算缩放比例
    scale = min(target_size / original_h, target_size / original_w)
    new_h, new_w = int(original_h * scale), int(original_w * scale)

    # 缩放图像并填充
    resized = cv2.resize(image, (new_w, new_h))
    padded = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
    pad_h, pad_w = (target_size - new_h) // 2, (target_size - new_w) // 2
    padded[pad_h:pad_h+new_h, pad_w:pad_w+new_w] = resized

    # 归一化并转换维度
    input_tensor = padded.astype(np.float32) / 255.0
    input_tensor = input_tensor.transpose(2, 0, 1)[np.newaxis, ...]

    return input_tensor, (scale, pad_w, pad_h, original_w, original_h)

def __non_max_suppression(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list:
    """非极大值抑制实现"""
    x1, y1 = boxes[:, 0], boxes[:, 1]
    x2, y2 = boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)

    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        intersection = w * h

        iou = intersection / (areas[i] + areas[order[1:]] - intersection + 1e-10)
        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]
    return keep

def infer_onnx_model(onnx_path: str, image: np.ndarray, confidence_threshold: float = 0.5, iou_threshold: float = 0.5) -> list[IdentifyResult]:
    """ONNX模型推理主函数"""

    # 初始化ONNX Runtime会话
    session = ort.InferenceSession(onnx_path)
    input_name = session.get_inputs()[0].name

    # 预处理图像
    input_tensor, (scale, pad_w, pad_h, orig_w, orig_h) = __preprocess(image)

    # 执行推理
    outputs = session.run(None, {input_name: input_tensor})
    output = outputs[0][0]  # 假设输出形状为(num_boxes, 85)

    # 解析检测结果
    boxes, scores, class_ids = [], [], []
    for det in output:
        obj_conf = det[4]
        if obj_conf < confidence_threshold:
            continue

        class_scores = det[5:]
        class_id = np.argmax(class_scores)
        score = obj_conf * class_scores[class_id]
        if score < confidence_threshold:
            continue

        # 转换坐标格式
        cx, cy, w, h = det[:4]
        x1 = (cx - w/2)
        y1 = (cy - h/2)
        x2 = (cx + w/2)
        y2 = (cy + h/2)

        boxes.append([x1, y1, x2, y2])
        scores.append(score)
        class_ids.append(class_id)

    if not boxes:
        return []

    # 转换为NumPy数组
    boxes = np.array(boxes)
    scores = np.array(scores)
    class_ids = np.array(class_ids)

    # 执行非极大值抑制
    keep = __non_max_suppression(boxes, scores, iou_threshold)
    boxes = boxes[keep]
    scores = scores[keep]
    class_ids = class_ids[keep]

    # 坐标转换到原始图像
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_w) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_h) / scale

    # 限制坐标范围
    np.clip(boxes[:, [0, 2]], 0, orig_w, out=boxes[:, [0, 2]])
    np.clip(boxes[:, [1, 3]], 0, orig_h, out=boxes[:, [1, 3]])

    # 生成结果
    results = []
    for cls_id, score, (x1, y1, x2, y2) in zip(class_ids, scores, boxes):
        results.append(
            IdentifyResult(names[int(cls_id)],
                           float(score),
                           Rectangle(float(x1), float(y1), float(x2), float(y2))))

    return results
