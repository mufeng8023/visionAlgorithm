"""
Author       : gxs
Date         : 2026-05-28 17:03:44
LastEditors  : gxs
LastEditTime : 2026-05-28 17:21:28
FilePath     : /visionAlgorithm/scripts/run_onnx_img.py
Description  :

如果需要添加新的模型YOLOv12等或者新的任务:seg/OBB,在当前脚本搜索"todo:添加新功能"直接定位到对应位置

Copyright (c) 2026 by gxs, All Rights Reserved.
"""

import argparse
import json
import os
import platform
import time
import warnings

import cv2
import numpy as np
import onnxruntime
import pkg_resources as pkg
import torch
import torchvision
from tqdm import tqdm

### =============================================================================================
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)

# 支持的图片格式
IMG_FORMATS = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
    ".dng",
    ".mpo",
    ".tif",
    ".tiff",
    ".webp",
    ".pfm",
    ".heic",
}


def get_color(index, is_rgb=False):
    # 将输入的索引转化为一个基于 HSL 模型的 RGB 颜色
    # HSL 转 RGB 核心逻辑
    # 计算色相（H）,以索引值乘以 137.508 后取余360,再除以360得到一个 0 到 1 之间的值
    h = (index * 137.508) % 360 / 360.0

    # 设置饱和度（S）和亮度（L）
    l, s = 0.6, 0.95

    # 定义辅助函数：根据 HSL 转 RGB
    # p 和 q 是根据亮度 l 和饱和度 s 计算得出的中间值
    def hue_to_rgb(p, q, t):
        # t 对应的色相值,循环在 0 到 1 之间
        t = t % 1.0
        # 根据 t 的范围计算 RGB 值
        if t < 1 / 6:
            return p + (q - p) * 6 * t
        if t < 1 / 2:
            return q
        if t < 2 / 3:
            return p + (q - p) * (2 / 3 - t) * 6
        return p

    # 当饱和度为 0 时,颜色为灰色
    if s == 0:
        r = g = b = l
    else:
        # q 和 p 是用于生成 RGB 的临时中间值
        # q 根据亮度和饱和度的关系来计算
        q = l * (1 + s) if l < 0.5 else l + s - l * s
        p = 2 * l - q
        # 使用 hue_to_rgb 函数根据色相计算 RGB 的每个通道
        r = hue_to_rgb(p, q, h + 1 / 3)
        g = hue_to_rgb(p, q, h)
        b = hue_to_rgb(p, q, h - 1 / 3)

    # 返回 BGR 值,并确保值在 0 到 255 之间
    b = int(max(0, min(b, 1)) * 255)
    g = int(max(0, min(g, 1)) * 255)
    r = int(max(0, min(r, 1)) * 255)
    if is_rgb:
        return r, g, b
    else:
        return b, g, r


def emojis(line=""):
    # Return platform-dependent emoji-safe version of string
    return (
        line.encode().decode("ascii", "ignore")
        if platform.system() == "Windows"
        else line
    )


def _red_str(lines):
    return f"\033[31m{lines}\033[0m"


def check_version(
    current="0.0.0",
    minimum="0.0.0",
    name="version ",
    pinned=False,
    hard=False,
    verbose=False,
):
    # Check version vs. required version
    current, minimum = (pkg.parse_version(x) for x in (current, minimum))
    result = (current == minimum) if pinned else (current >= minimum)  # bool
    s = f"WARNING ⚠️ {name}{minimum} is required by YOLOv5, but {name}{current} is currently installed"  # string
    if hard:
        assert result, emojis(s)  # assert min requirements met
    if verbose and not result:
        print(_red_str(s))

    return result


def check_dir(_dir, increment=True, remove=False):
    """

    @param _dir:要检测的目录
    @param increment: True:存在就在之后递增,不存在就创建;False:不存在创建,存在直接退出函数
    @param remove: 存在就删除目录,重新创建
    @return:
    """
    if remove and os.path.exists(_dir):
        os.remove(_dir)
        os.makedirs(_dir)
        return _dir

    if increment:
        count = 1
        new_dir = _dir
        while os.path.exists(new_dir):
            count += 1
            new_dir = f"{_dir}{count}"
        os.makedirs(new_dir)
        return new_dir
    else:
        if not os.path.exists(_dir):
            os.makedirs(_dir)
        return _dir


def pre_process_resize_img(image, new_shape):
    """
    宽高等比例缩放
    @param image:
    @param new_shape: (target_width, target_height)
    @return:
    """
    # 获取原始图像的宽度和高度
    height, width = image.shape[:2]

    # 目标宽度和高度
    target_width, target_height = new_shape

    # 计算宽度和高度的缩放比例
    ratio = min(target_width / width, target_height / height)

    # 计算缩放后的宽度和高度
    new_width = int(width * ratio)
    new_height = int(height * ratio)

    # 等比例缩放图像
    resized_img = cv2.resize(image, (new_width, new_height))

    # 计算需要填充的宽度和高度
    dw = (target_width - new_width) // 2
    dh = (target_height - new_height) // 2

    # 创建一个白色底图
    padded_img = 144 * np.ones(shape=[target_height, target_width, 3], dtype=np.float32)

    # 将缩放后的图像放置在中心位置
    padded_img[dh : dh + new_height, dw : dw + new_width, :] = resized_img

    return padded_img, ratio, dw, dh


def clip_boxes(boxes, shape):
    """
    保证预测的box在图片内

    Args:
      boxes (torch.Tensor): the bounding boxes to clip
      shape (tuple): the shape of the image
    """
    if isinstance(boxes, torch.Tensor):  # faster individually
        boxes[..., 0].clamp_(0, shape[1])  # x1
        boxes[..., 1].clamp_(0, shape[0])  # y1
        boxes[..., 2].clamp_(0, shape[1])  # x2
        boxes[..., 3].clamp_(0, shape[0])  # y2

    else:  # np.array (faster grouped)
        boxes[..., [0, 2]] = boxes[..., [0, 2]].clip(0, shape[1])  # x1, x2
        boxes[..., [1, 3]] = boxes[..., [1, 3]].clip(0, shape[0])  # y1, y2


def clip_coords(coords, shape):
    """
    保证预测的关键点坐标在图片内

    Args:
        coords (torch.Tensor | numpy.ndarray): A list of line coordinates.
        shape (tuple): A tuple of integers representing the size of the image in the format (height, width).

    Returns:
        (None): The function modifies the input `coordinates` in place,
                by clipping each coordinate to the image boundaries.
    """
    if isinstance(coords, torch.Tensor):  # faster individually
        coords[..., 0].clamp_(0, shape[1])  # x
        coords[..., 1].clamp_(0, shape[0])  # y

    else:  # np.array (faster grouped)
        coords[..., 0] = coords[..., 0].clip(0, shape[1])  # x
        coords[..., 1] = coords[..., 1].clip(0, shape[0])  # y


def scale_coords(
    img1_shape, coords, img0_shape, ratio_pad=None, normalize=False, padding=True
):
    """
    将线段坐标 （xy） 从 img1_shape 重新缩放为 img0_shape  \n
    Rescale segment coordinates (xy) from img1_shape to img0_shape.

    Args:
        img1_shape (tuple): The shape of the image that the coords are from.
        coords (torch.Tensor): the coords to be scaled of shape n,2.
        img0_shape (tuple): the shape of the image that the segmentation is being applied to.
        ratio_pad (tuple): the ratio of the image size to the padded image size.
        normalize (bool): If True, the coordinates will be normalized to the range [0, 1]. Defaults to False.
        padding (bool): If True, assuming the boxes is based on image augmented by yolo style. If False then do regular
            rescaling.

    Returns:
        coords (torch.Tensor): The scaled coordinates.
    """
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(
            img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1]
        )  # gain  = old / new
        pad = (
            (img1_shape[1] - img0_shape[1] * gain) / 2,
            (img1_shape[0] - img0_shape[0] * gain) / 2,
        )  # wh padding

    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    if padding:
        coords[..., 0] -= pad[0]  # x padding
        coords[..., 1] -= pad[1]  # y padding

    coords[..., 0] /= gain
    coords[..., 1] /= gain

    clip_coords(coords, img0_shape)

    if normalize:
        coords[..., 0] /= img0_shape[1]  # width
        coords[..., 1] /= img0_shape[0]  # height

    return coords


def scale_boxes(img1_shape, boxes, img0_shape, ratio_pad=None, padding=True):
    """
    从最初指定边界框的图像形状重新缩放边界框(格式为 xyxy)img1_shape到不同图像(img0_shape)的形状。

    Args:
        img1_shape (tuple): The shape of the image that the bounding boxes are for, in the format of (height, width).
        boxes (torch.Tensor): the bounding boxes of the objects in the image, in the format of (x1, y1, x2, y2)
        img0_shape (tuple): the shape of the target image, in the format of (height, width).
        ratio_pad (tuple): a tuple of (ratio, pad) for scaling the boxes. If not provided, the ratio and pad will be
            calculated based on the size difference between the two images.
        padding (bool): If True, assuming the boxes is based on image augmented by yolo style. If False then do regular
            rescaling.

    Returns:
        boxes (torch.Tensor): The scaled bounding boxes, in the format of (x1, y1, x2, y2)
    """
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(
            img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1]
        )  # gain  = old / new
        pad = (
            round((img1_shape[1] - img0_shape[1] * gain) / 2 - 0.1),
            round((img1_shape[0] - img0_shape[0] * gain) / 2 - 0.1),
        )  # wh padding
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    if padding:
        boxes[..., [0, 2]] -= pad[0]  # x padding
        boxes[..., [1, 3]] -= pad[1]  # y padding

    boxes[..., :4] /= gain
    clip_boxes(boxes, img0_shape)

    return boxes


def xyxy2xywh(x):
    """
    Convert bounding box coordinates from (x1, y1, x2, y2) format to (x, y, width, height) format where (x1, y1) is the
    top-left corner and (x2, y2) is the bottom-right corner.

    Args:
        x (np.ndarray | torch.Tensor): The input bounding box coordinates in (x1, y1, x2, y2) format.

    Returns:
        y (np.ndarray | torch.Tensor): The bounding box coordinates in (x, y, width, height) format.
    """
    assert x.shape[-1] == 4, (
        f"input shape last dimension expected 4 but input shape is {x.shape}"
    )
    y = (
        torch.empty_like(x) if isinstance(x, torch.Tensor) else np.empty_like(x)
    )  # faster than clone/copy
    y[..., 0] = (x[..., 0] + x[..., 2]) / 2  # x center
    y[..., 1] = (x[..., 1] + x[..., 3]) / 2  # y center
    y[..., 2] = x[..., 2] - x[..., 0]  # width
    y[..., 3] = x[..., 3] - x[..., 1]  # height
    return y


def xywh2xyxy(x):
    """
    Convert bounding box coordinates from (x, y, width, height) format to (x1, y1, x2, y2) format where (x1, y1) is the
    top-left corner and (x2, y2) is the bottom-right corner.

    Args:
        x (np.ndarray | torch.Tensor): The input bounding box coordinates in (x, y, width, height) format.

    Returns:
        y (np.ndarray | torch.Tensor): The bounding box coordinates in (x1, y1, x2, y2) format.
    """
    assert x.shape[-1] == 4, (
        f"input shape last dimension expected 4 but input shape is {x.shape}"
    )
    y = (
        torch.empty_like(x) if isinstance(x, torch.Tensor) else np.empty_like(x)
    )  # faster than clone/copy
    dw = x[..., 2] / 2  # half-width
    dh = x[..., 3] / 2  # half-height
    y[..., 0] = x[..., 0] - dw  # top left x
    y[..., 1] = x[..., 1] - dh  # top left y
    y[..., 2] = x[..., 0] + dw  # bottom right x
    y[..., 3] = x[..., 1] + dh  # bottom right y
    return y


def _get_covariance_matrix(boxes):
    """
    从定向边界框生成协方差矩阵。

    Args:
        boxes (torch.Tensor): 形状为(N, 5)的张量,表示旋转边界框,格式为xywhr。

    Returns:
        (torch.Tensor): 对应原始旋转边界框的协方差矩阵。
    """
    # Gaussian bounding boxes, ignore the center points (the first two columns) because they are not needed here.
    gbbs = torch.cat((boxes[:, 2:4].pow(2) / 12, boxes[:, 4:]), dim=-1)
    a, b, c = gbbs.split(1, dim=-1)
    cos = c.cos()
    sin = c.sin()
    cos2 = cos.pow(2)
    sin2 = sin.pow(2)
    return a * cos2 + b * sin2, a * sin2 + b * cos2, (a - b) * cos * sin


def batch_probiou(obb1, obb2, eps=1e-7):
    """
    计算定向边界框之间的概率IoU。

    Args:
        obb1 (torch.Tensor | np.ndarray): 形状为(N, 5)的张量,表示真实obb,格式为xywhr。
        obb2 (torch.Tensor | np.ndarray): 形状为(M, 5)的张量,表示预测obb,格式为xywhr。
        eps (float, optional): 避免除零的小值。

    Returns:
        (torch.Tensor): 形状为(N, M)的张量,表示obb相似度。

    References:
        https://arxiv.org/pdf/2106.06072v1.pdf
    """
    obb1 = torch.from_numpy(obb1) if isinstance(obb1, np.ndarray) else obb1
    obb2 = torch.from_numpy(obb2) if isinstance(obb2, np.ndarray) else obb2

    x1, y1 = obb1[..., :2].split(1, dim=-1)
    x2, y2 = (x.squeeze(-1)[None] for x in obb2[..., :2].split(1, dim=-1))
    a1, b1, c1 = _get_covariance_matrix(obb1)
    a2, b2, c2 = (x.squeeze(-1)[None] for x in _get_covariance_matrix(obb2))

    t1 = (
        ((a1 + a2) * (y1 - y2).pow(2) + (b1 + b2) * (x1 - x2).pow(2))
        / ((a1 + a2) * (b1 + b2) - (c1 + c2).pow(2) + eps)
    ) * 0.25
    t2 = (
        ((c1 + c2) * (x2 - x1) * (y1 - y2))
        / ((a1 + a2) * (b1 + b2) - (c1 + c2).pow(2) + eps)
    ) * 0.5
    t3 = (
        ((a1 + a2) * (b1 + b2) - (c1 + c2).pow(2))
        / (
            4
            * ((a1 * b1 - c1.pow(2)).clamp_(0) * (a2 * b2 - c2.pow(2)).clamp_(0)).sqrt()
            + eps
        )
        + eps
    ).log() * 0.5
    bd = (t1 + t2 + t3).clamp(eps, 100.0)
    hd = (1.0 - (-bd).exp() + eps).sqrt()
    return 1 - hd


def nms_rotated(boxes, scores, threshold=0.45, use_triu=True):
    """
    使用probiou和快速NMS对定向边界框进行非极大值抑制。

    Args:
        boxes (torch.Tensor): 旋转边界框,形状(N, 5),格式xywhr。
        scores (torch.Tensor): 置信度分数,形状(N,)。
        threshold (float): IoU阈值。
        use_triu (bool): 是否使用`torch.triu`运算符。在导出obb模型到不支持`torch.triu`的格式时禁用它很有用。

    Returns:
        (torch.Tensor): NMS后要保留的边界框索引。
    """
    sorted_idx = torch.argsort(scores, descending=True)
    boxes = boxes[sorted_idx]
    ious = batch_probiou(boxes, boxes)
    if use_triu:
        ious = ious.triu_(diagonal=1)
        # pick = torch.nonzero(ious.max(dim=0)[0] < threshold).squeeze_(-1)
        # NOTE: handle the case when len(boxes) hence exportable by eliminating if-else condition
        pick = torch.nonzero((ious >= threshold).sum(0) <= 0).squeeze_(-1)
    else:
        n = boxes.shape[0]
        row_idx = torch.arange(n, device=boxes.device).view(-1, 1).expand(-1, n)
        col_idx = torch.arange(n, device=boxes.device).view(1, -1).expand(n, -1)
        upper_mask = row_idx < col_idx
        ious = ious * upper_mask
        # Zeroing these scores ensures the additional indices would not affect the final results
        scores[~((ious >= threshold).sum(0) <= 0)] = 0
        # NOTE: return indices with fixed length to avoid TFLite reshape error
        pick = torch.topk(scores, scores.shape[0]).indices
    return sorted_idx[pick]


def non_max_suppression(
    prediction,
    conf_thres=0.25,
    iou_thres=0.45,
    classes=None,
    agnostic=False,
    multi_label=False,
    labels=(),
    max_det=300,
    nc=0,  # number of classes (optional)
    max_time_img=0.05,
    max_nms=30000,
    max_wh=7680,
    in_place=True,
    rotated=False,
    end2end=False,
):
    """
    对一组边界框执行非极大值抑制(NMS),支持掩码和每个框的多个标签。

    Args:
        prediction (torch.Tensor): 形状为(batch_size, num_classes + 4 + num_masks, num_boxes)的张量,
            包含预测的边界框、类别和掩码。张量应为模型输出的格式,如YOLO。
        conf_thres (float): 置信度阈值,低于此值的边界框将被过滤掉。
            有效值在0.0到1.0之间。
        iou_thres (float): IoU阈值,在NMS期间低于此值的边界框将被过滤掉。
            有效值在0.0到1.0之间。
        classes (List[int]): 要考虑的类别索引列表。如果为None,则考虑所有类别。
        agnostic (bool): 如果为True,模型对类别数量不敏感,所有类别将被视为一个。
        multi_label (bool): 如果为True,每个框可能有多个标签。
        labels (List[List[Union[int, float, torch.Tensor]]]): 列表的列表,其中每个内部列表包含给定图像的先验标签。
            列表应为数据加载器输出的格式,每个标签是(class_index, x1, y1, x2, y2)的元组。
        max_det (int): NMS后要保留的最大边界框数量。
        nc (int): 模型输出的类别数量。此后的任何索引将被视为掩码。
        max_time_img (float): 处理一张图像的最大时间(秒)。
        max_nms (int): 输入torchvision.ops.nms()的最大边界框数量。
        max_wh (int): 最大边界框宽度和高度(像素)。
        in_place (bool): 如果为True,输入预测张量将被原地修改。
        rotated (bool): 如果正在传递定向边界框(OBB)进行NMS。
        end2end (bool): 如果模型不需要NMS。

    Returns:
        (List[torch.Tensor]): 长度为batch_size的列表,其中每个元素是形状为(num_boxes, 6 + num_masks)的张量,
            包含保留的边界框,列包括(x1, y1, x2, y2, confidence, class, mask1, mask2, ...)。
    """
    # Checks
    assert 0 <= conf_thres <= 1, (
        f"Invalid Confidence threshold {conf_thres}, valid values are between 0.0 and 1.0"
    )
    assert 0 <= iou_thres <= 1, (
        f"Invalid IoU {iou_thres}, valid values are between 0.0 and 1.0"
    )
    if isinstance(
        prediction, (list, tuple)
    ):  # YOLOv8 model in validation model, output = (inference_out, loss_out)
        prediction = prediction[0]  # select only inference output
    if classes is not None:
        classes = torch.tensor(classes, device=prediction.device)

    # !end2end不能在这里写, 和原始模型不一样的地方是原始模型end2end输出的是已经获取了nc的最大分数的conf且排序好的
    # !而这里输出的是所有类别的预测结果,需要根据conf_thres和nc进行筛选和排序
    # if prediction.shape[-1] == 6:  # end-to-end model (BNC, i.e. 1,300,6)
    #     output = [pred[pred[:, 4] > conf_thres][:max_det] for pred in prediction]
    #     if classes is not None:
    #         output = [pred[(pred[:, 5:6] == classes).any(1)] for pred in output]
    #     return output

    bs = prediction.shape[0]  # batch size (BCN, i.e. 1,84,6300)
    nc = nc or (prediction.shape[1] - 4)  # number of classes
    nm = prediction.shape[1] - nc - 4  # number of masks
    mi = 4 + nc  # mask start index
    xc = prediction[:, 4:mi].amax(1) > conf_thres  # candidates

    # Settings
    # min_wh = 2  # (pixels) minimum box width and height
    time_limit = 2.0 + max_time_img * bs  # seconds to quit after
    multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)

    prediction = prediction.transpose(-1, -2)  # shape(1,84,6300) to shape(1,6300,84)
    if not rotated:
        if in_place:
            prediction[..., :4] = xywh2xyxy(prediction[..., :4])  # xywh to xyxy
        else:
            prediction = torch.cat(
                (xywh2xyxy(prediction[..., :4]), prediction[..., 4:]),
                dim=-1,
            )  # xywh to xyxy

    t = time.time()
    output = [torch.zeros((0, 6 + nm), device=prediction.device)] * bs
    for xi, x in enumerate(prediction):  # image index, image inference
        # Apply constraints
        # x[((x[:, 2:4] < min_wh) | (x[:, 2:4] > max_wh)).any(1), 4] = 0  # width-height
        x = x[xc[xi]]  # confidence

        # Cat apriori labels if autolabelling
        if labels and len(labels[xi]) and not rotated:
            lb = labels[xi]
            v = torch.zeros((len(lb), nc + nm + 4), device=x.device)
            v[:, :4] = xywh2xyxy(lb[:, 1:5])  # box
            v[range(len(lb)), lb[:, 0].long() + 4] = 1.0  # cls
            x = torch.cat((x, v), 0)

        # If none remain process next image
        if not x.shape[0]:
            continue

        # Detections matrix nx6 (xyxy, conf, cls)
        box, cls, mask = x.split((4, nc, nm), 1)

        if end2end:
            # !如果是end2end multi_label 必须是false
            multi_label = False

        if multi_label:
            i, j = torch.where(cls > conf_thres)
            x = torch.cat((box[i], x[i, 4 + j, None], j[:, None].float(), mask[i]), 1)
        else:  # best class only
            conf, j = cls.max(1, keepdim=True)
            x = torch.cat((box, conf, j.float(), mask), 1)[conf.view(-1) > conf_thres]

        # Filter by class
        if classes is not None:
            x = x[(x[:, 5:6] == classes).any(1)]

        # !如果end2end, 将上面获取的 x 根据 conf 从大到小排序, 之后只要前[:max_det]个
        if end2end:
            # !且最后结果大于max_det
            # 根据conf排序
            x = x[x[:, 4].argsort(descending=True)[:max_det]]
            # 只取前max_det个
            x = x[x[:, 4] > conf_thres]
            # 最后预测结果个数大于0
            if len(x) > 0:
                # 将结果添加到output中
                output[xi] = x
            continue

        # Check shape
        n = x.shape[0]  # number of boxes
        if not n:  # no boxes
            continue
        if n > max_nms:  # excess boxes
            x = x[
                x[:, 4].argsort(descending=True)[:max_nms]
            ]  # sort by confidence and remove excess boxes

        # Batched NMS
        c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
        scores = x[:, 4]  # scores
        if rotated:
            boxes = torch.cat((x[:, :2] + c, x[:, 2:4], x[:, -1:]), dim=-1)  # xywhr
            i = nms_rotated(boxes, scores, iou_thres)
        else:
            boxes = x[:, :4] + c  # boxes (offset by class)
            i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
        i = i[:max_det]  # limit detections

        output[xi] = x[i]
        if (time.time() - t) > time_limit:
            warnings.warn(f"WARNING ⚠️ NMS time limit {time_limit:.3f}s exceeded")
            break  # time limit exceeded

    return output


def plot_box(image, boxes, names: list):
    """

    @param image:
    @param boxes: 边界框[n, 6] [x, y, x, y, conf, cls]
    @param names: 类别名
    @return:
    """
    # 自适应图片大小计算线宽和文字大小
    # 图片大小
    img_h, img_w = image.shape[:2]
    # box线宽
    box_thickness = max(round(sum((img_h, img_w)) / 2 * 0.003), 2)
    # font_size = max(round(sum((img_h, img_w)) / 2 * 0.035), 12)
    # text线宽
    text_thickness = max(box_thickness - 1, 1)
    # text字体大小
    fontScale = box_thickness / 4.0
    # 字体
    fontFace = 0

    # 画框
    for x1, y1, x2, y2, conf, cls in boxes:
        x1, y1, x2, y2, cls = list(map(int, [x1, y1, x2, y2, cls]))
        # 画框
        cv2.rectangle(
            image,
            pt1=(x1, y1),
            pt2=(x2, y2),
            color=get_color(cls),
            thickness=box_thickness,
            lineType=cv2.LINE_AA,
        )

        # 在框上显示的text
        if len(names):
            text = f"{names[cls]}:{conf:.2f}"
        else:
            text = f"{int(cls)}:{conf:.2f}"
        # 用于计算特定文本字符串在给定字体和大小下的尺寸
        text_w, text_h = cv2.getTextSize(
            text=text,
            fontFace=fontFace,
            fontScale=fontScale,
            thickness=text_thickness,
        )[0]
        # 防止写的text超过了上边界,导致看不到,如果能写外面就写外面
        inside_x = (
            img_w - x1 - text_w
        ) >= 5  # 如果横着会导致文本超过图像边界,text向左挪一挪
        inside_y = (y1 - text_h) >= 8
        # 计算字体左下角坐标
        text_l = x1 if inside_x else x1 - (x1 + text_w + 3 - img_w)
        text_b = (y1 - 4) if inside_y else y1 + text_h + 5

        # 写文字
        cv2.rectangle(
            image,
            pt1=(text_l, text_b - text_h),
            pt2=(text_l + text_w, text_b + 4),
            color=(255, 255, 255),
            thickness=-1,
        )
        cv2.putText(
            image,
            text=text,
            org=(text_l, text_b),  # 防止超过上边界
            fontFace=fontFace,
            fontScale=fontScale,
            thickness=text_thickness,
            color=get_color(cls),
        )
    return image


def plot_kpts(image, kpts, radius=5, conf_thres=0.5):
    """
    绘制关键点
    Args:
        image:
        kpts: (n, 17, 3)
        radius: 关键点的半径
        conf_thres: 关键点可见的置信度

    Returns:

    """
    # 肢体颜色
    # limb_color = pose_palette[[9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16]]
    limb_color = [
        get_color(i)
        for i in [9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16]
    ]
    # 肢体颜色19
    # kpt_color = pose_palette[[16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9]]
    kpt_color = [
        get_color(i) for i in [16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9]
    ]
    # 肢体两个端点定义19
    skeleton = [
        [16, 14],
        [14, 12],
        [17, 15],
        [15, 13],
        [12, 13],
        [6, 12],
        [7, 13],
        [6, 7],
        [6, 8],
        [7, 9],
        [8, 10],
        [9, 11],
        [2, 3],
        [1, 2],
        [1, 3],
        [2, 4],
        [3, 5],
        [4, 6],
        [5, 7],
    ]

    # kpts个数
    n, nkpt, ndim = kpts.shape

    # image.shape:h,w
    img_h, img_w = image.shape[:2]

    # 有n个目标
    for i in range(n):
        # 画点
        # 每个目标的所有关键点
        for j in range(nkpt):
            # 关键点的可见度
            v = kpts[i, j, 2]
            if v < conf_thres:  # 可见度太低就不要了
                continue

            # 特定关键点有特定颜色
            try:
                color_k = kpt_color[j]
            except IndexError:
                # 如果索引超出了范围,就使用默认颜色
                color_k = get_color(j)

            x = kpts[i, j, 0]
            y = kpts[i, j, 1]

            cv2.circle(
                image,
                center=(int(x), int(y)),
                radius=radius,
                color=color_k,
                thickness=-1,
                lineType=cv2.LINE_AA,
            )

        try:  # 不同关键点可能需要自定义,为了兼容性,线段不绘制了
            # 画每个肢体的线段
            for s, sk in enumerate(skeleton):
                # 获取该肢体的两个端点(x, y)
                pos1 = int(kpts[i, sk[0] - 1, 0]), int(kpts[i, sk[0] - 1, 1])
                pos2 = int(kpts[i, sk[1] - 1, 0]), int(kpts[i, sk[1] - 1, 1])
                # 关键点的置信度
                v1 = kpts[i, sk[0] - 1, 2]
                v2 = kpts[i, sk[1] - 1, 2]

                if v1 < conf_thres or v2 < conf_thres:
                    continue

                # 确定两个端点都存在,保证端点坐标值不会出现大于wh和小于0的情况
                if (
                    pos1[0] % img_w == 0
                    or pos1[1] % img_h == 0
                    or pos1[0] < 0
                    or pos1[1] < 0
                ):
                    continue
                if (
                    pos2[0] % img_w == 0
                    or pos2[1] % img_h == 0
                    or pos2[0] < 0
                    or pos2[1] < 0
                ):
                    continue

                try:
                    color_s = limb_color[s]
                except IndexError:
                    # 如果索引超出了范围,就使用默认颜色
                    color_s = get_color(s)

                # 两个端点之间划线
                cv2.line(
                    image,
                    pt1=pos1,
                    pt2=pos2,
                    color=color_s,
                    thickness=2,
                    lineType=cv2.LINE_AA,
                )
        except Exception:
            print(
                "线段绘制失败, 请检查 skeleton 的设置, "
                "代码中设置的 skeleton 是针对 coco-pose 的 17 关键点的, "
                "如果是其他关键点, 请自行修改 skeleton 的设置"
            )

    return image


class YOLOv8PostProcess:
    def __init__(self, nc, kpt_shape=(0, 3), device=-1):
        """
        后处理类,模型的输出可以直接输入进来,将预测结果映射回输入图中
        Args:
            nc: 类别个数
            kpt_shape: 关键点的维度(17, 3)
            inplace:
        """
        self.nc = nc  # 类别个数
        self.no = 4 + nc + kpt_shape[0] * kpt_shape[1]  # 56 模型输出的通道数量
        self.nl = 3  # 网络输出特征个数
        self.kpt_shape = kpt_shape  # 关键点

        # 因为是anchor-free的算法没有anchor,只有锚点,为了和anchor-Base的代码复用,定义anchor-free的个数为1
        self.na = 1
        if device == -1:
            self.device = torch.device("cpu")
        else:
            self.device = torch.device(f"cuda:{device}")

        # 一个特征层对应一个anchor-grid
        self.grid = [torch.empty(0, device=self.device) for _ in range(self.nl)]

        # 步距
        self.stride = torch.Tensor([8, 16, 32]).to(device=self.device)
        # todo:添加新功能,直接定位到对应位置
        # 需要添加seg和OBB的后处理

    def post_process(self, x, xywh=True):
        """
        将网络预测的结果和anchor计算将结果映射回输入图
        包括关键点和anchor
        Args:
            x: 网络的三个输出层
            xywh: True返回结果是xywh, False返回结果是x1y1x2y2

        Returns:

        """
        assert len(x) == self.nl, "ERROR"
        z = []
        kpt_num = self.kpt_shape[0] * self.kpt_shape[1]

        for i in range(len(x)):
            bs, _, ny, nx = x[i].shape

            stride = self.stride[i]

            # (b, no, h, w) -> (b, 1, no, h, w) -> (b, 1, h, w, no)
            # no: 4 + 1 + 17*3
            out = (
                x[i]
                .view(bs, self.na, self.no, ny, nx)
                .permute(0, 1, 3, 4, 2)
                .contiguous()
            )

            # 没有锚点的时候,先生成锚点
            if self.grid[i].shape[2:4] != out.shape[2:4]:
                self.grid[i] = self._make_grid(nx, ny)

            # (b, 1, h, w, 2)
            grid = self.grid[i]  # (x, y)

            # 将预测的坐标映射回来
            lt = out[..., 0:2]  # (b, 1, h, w, 2)
            rb = out[..., 2:4]  # (b, 1, h, w, 2)

            # 最后预测对应输入图的边界框
            x1y1 = (grid - lt) * stride
            x2y2 = (grid + rb) * stride

            # 处理box的返回值类型是什么格式现在是xyxy
            if xywh:  # 返回值类型如果是xywh需要转换一下
                c_xy = (x1y1 + x2y2) / 2.0
                wh = x2y2 - x1y1
                out[..., 0:2] = c_xy
                out[..., 2:4] = wh
            else:
                out[..., 0:2] = x1y1
                out[..., 2:4] = x2y2

            # 处理关键点的事情
            if kpt_num > 0:
                # 输出中 关键点开始 和 结束 的索引
                kpt_start = 4 + self.nc
                kpt_end = self.no + 1
                kpt_stride = self.kpt_shape[1]
                # 关键点
                kpts_x = out[..., kpt_start:kpt_end:kpt_stride]
                kpts_y = out[..., (kpt_start + 1) : kpt_end : kpt_stride]

                kpts_x = (kpts_x * 2.0 + (grid[..., 0:1] - 0.5)) * stride  # x
                kpts_y = (kpts_y * 2.0 + (grid[..., 1:2] - 0.5)) * stride  # y

                out[..., kpt_start:kpt_end:kpt_stride] = kpts_x
                out[..., (kpt_start + 1) : kpt_end : kpt_stride] = kpts_y

            # todo:添加新功能,直接定位到对应位置
            # 需要要添加seg和OBB的后处理

            # 处理好的结果先保存到list中 (b, n, no)
            out = out.view(bs, -1, self.no)
            z.append(out)

        # (b, 1*(h1*w1 + h2*w2 +h3*w3), 56)
        return torch.cat(z, dim=1)

    def _make_grid(self, nx, ny, grid_cell_offset=0.5, dtype=torch.float32):
        # 锚点的shape
        shape = 1, self.na, ny, nx, 2

        sx = torch.arange(end=nx, dtype=dtype)  # shift x
        sy = torch.arange(end=ny, dtype=dtype)  # shift y

        sy, sx = torch.meshgrid(sy, sx, indexing="ij")
        # (nx, ny, 2)
        grid = torch.stack((sx, sy), -1) + grid_cell_offset
        # (ny, nx, 2) -> (1, 1, ny, nx, 2)
        grid = grid.expand(shape)

        return grid.to(device=self.device)


class YOLO26PostProcess(YOLOv8PostProcess):
    def __init__(self, nc=1, kpt_shape=(0, 3), device=-1):
        super().__init__(nc, kpt_shape, device)
        # 需要添加seg和OBB的后处理

        pass

    def post_process(self, x, xywh=True):
        """
        将网络预测的结果和anchor计算将结果映射回输入图
        包括关键点和anchor
        Args:
            x: 网络的三个输出层
            xywh: True返回结果是xywh, False返回结果是x1y1x2y2

        Returns:

        """
        assert len(x) == self.nl, "ERROR"
        z = []
        kpt_num = self.kpt_shape[0] * self.kpt_shape[1]

        for i in range(len(x)):
            bs, _, ny, nx = x[i].shape

            stride = self.stride[i]

            # (b, no, h, w) -> (b, 1, no, h, w) -> (b, 1, h, w, no)
            # no: 4 + 1 + 17*3
            out = (
                x[i]
                .view(bs, self.na, self.no, ny, nx)
                .permute(0, 1, 3, 4, 2)
                .contiguous()
            )

            # 没有锚点的时候,先生成锚点
            if self.grid[i].shape[2:4] != out.shape[2:4]:
                self.grid[i] = self._make_grid(nx, ny)

            # (b, 1, h, w, 2)
            grid = self.grid[i]  # (x, y)

            # 将预测的坐标映射回来
            lt = out[..., 0:2]  # (b, 1, h, w, 2)
            rb = out[..., 2:4]  # (b, 1, h, w, 2)

            # 最后预测对应输入图的边界框
            x1y1 = (grid - lt) * stride
            x2y2 = (grid + rb) * stride

            # 处理box的返回值类型是什么格式现在是xyxy
            if xywh:  # 返回值类型如果是xywh需要转换一下
                c_xy = (x1y1 + x2y2) / 2.0
                wh = x2y2 - x1y1
                out[..., 0:2] = c_xy
                out[..., 2:4] = wh
            else:
                out[..., 0:2] = x1y1
                out[..., 2:4] = x2y2

            # 处理关键点的事情
            if kpt_num > 0:
                # 输出中 关键点开始 和 结束 的索引
                kpt_start = 4 + self.nc
                kpt_end = self.no + 1
                kpt_stride = self.kpt_shape[1]
                # 关键点
                kpts_x = out[..., kpt_start:kpt_end:kpt_stride]
                kpts_y = out[..., (kpt_start + 1) : kpt_end : kpt_stride]

                # NOTE: YOLO26-Pose 和YOLOv8-Pose/YOLO11-Pose 关键点的区别就是在这里
                kpts_x = (kpts_x + (grid[..., 0:1])) * stride  # x
                kpts_y = (kpts_y + (grid[..., 1:2])) * stride  # y

                out[..., kpt_start:kpt_end:kpt_stride] = kpts_x
                out[..., (kpt_start + 1) : kpt_end : kpt_stride] = kpts_y

            # todo:添加新功能,直接定位到对应位置
            # 需要要添加seg和OBB的后处理

            # 处理好的结果先保存到list中 (b, n, no)
            out = out.view(bs, -1, self.no)
            z.append(out)

        # (b, 1*(h1*w1 + h2*w2 +h3*w3), 56)
        return torch.cat(z, dim=1)


class YOLOv5PostProcess:
    def __init__(self, nc, anchors, kpt_shape=(0, 3), has_conf=True, device=-1):
        """
        @description:
        @param self {} :
        @param nc {} :
        @param anchors {} :
        @param kpt_shape {} :
        @param has_conf {} : 输出是否包括conf,如果不包含会影响no的计算,yolov5-face就不包含conf
        @param device {} :
        @return {}
        """
        self.nc = nc
        assert nc > 0, "类别nc必须大于0"
        self.kpt_shape = kpt_shape
        self.has_conf = has_conf
        if has_conf:
            self.no = 4 + 1 + nc + kpt_shape[0] * kpt_shape[1]
        else:
            self.no = 4 + nc + kpt_shape[0] * kpt_shape[1]
        self.nl = len(anchors)
        self.na = len(anchors[0]) // 2

        if device == -1:
            self.device = torch.device("cpu")
        else:
            self.device = torch.device(f"cuda:{device}")

        #
        self.anchors = (
            torch.tensor(anchors, device=self.device).float().view(self.nl, -1, 2)
        )  # shape(nl,na,2)
        # 计算xy
        self.grid = [
            torch.empty(0, device=self.device) for _ in range(self.nl)
        ]  # init grid
        # 计算wh
        self.anchor_grid = [
            torch.empty(0, device=self.device) for _ in range(self.nl)
        ]  # init anchor grid

        # 网络输出层的步距
        self.stride = torch.Tensor([8, 16, 32]).to(device=self.device)
        # NOTE:因为这里输入的anchor是没有处理过的对应原图的anchor,所以需要转成对应特征图大小
        self.anchors /= self.stride.view(-1, 1, 1)

    def post_process(self, x, xywh=True):
        """
        将网络预测的结果和anchor计算将结果映射回输入图
        包括关键点和anchor
        Args:
            x: 网络的三个输出层
            xywh: True返回结果是xywh, False返回结果是x1y1x2y2

        Returns:

        """
        z = []  # inference output

        kpt_num = self.kpt_shape[0] * self.kpt_shape[1]

        for i in range(len(x)):
            bs, _, ny, nx = x[i].shape  # x(bs,255,20,20) to x(bs,3,20,20,85)
            # (b, na*no, h, w) -> (b, na, h, w, no)
            out = (
                x[i]
                .view(bs, self.na, self.no, ny, nx)
                .permute(0, 1, 3, 4, 2)
                .contiguous()
            )

            if self.grid[i].shape[2:4] != out.shape[2:4]:
                # grid: (1, na, h, w, xy)
                # anchor_grid: (1, na, h, w, wh)
                self.grid[i], self.anchor_grid[i] = self._make_grid(nx, ny, i)

            # 结果拆开
            xy = out[..., 0:2]
            wh = out[..., 2:4]
            # grid: (1, na, h, w, xy)
            # anchor_grid: (1, na, h, w, wh)
            # 计算预测的box
            xy = (xy * 2 + self.grid[i] - 0.5) * self.stride[i]  # xy
            wh = (wh * 2) ** 2 * self.anchor_grid[i]  # wh

            # 在out中替换计算好的值
            if xywh:
                out[..., 0:2] = xy
                out[..., 2:4] = wh
            else:
                # x1y1x2y2
                x1y1 = xy - wh / 2.0
                x2y2 = xy + wh / 2.0
                out[..., 0:2] = x1y1
                out[..., 2:4] = x2y2

            # 处理关键点
            if kpt_num > 0:
                # 输出中 关键点开始 和 结束 的索引
                if self.has_conf:
                    kpt_start = 4 + 1 + self.nc
                else:
                    kpt_start = 4 + self.nc
                kpt_end = self.no + 1
                kpt_stride = self.kpt_shape[1]
                # todo:待验证
                # 计算预测的kpt
                kpts_x = out[..., kpt_start:kpt_end:kpt_stride]
                kpts_y = out[..., (kpt_start + 1) : kpt_end : kpt_stride]

                kpts_x = (
                    kpts_x * self.anchor_grid[i][..., 0:1]
                    + self.grid[i][..., 0:1] * self.stride[i]
                )
                kpts_y = (
                    kpts_y * self.anchor_grid[i][..., 1:2]
                    + self.grid[i][..., 1:2] * self.stride[i]
                )

                # 在out中替换计算好的值
                out[..., kpt_start:kpt_end:kpt_stride] = kpts_x
                out[..., (kpt_start + 1) : kpt_end : kpt_stride] = kpts_y

            # 处理好的结果先保存到list中 (b, n, no)
            out = out.view(bs, -1, self.no)
            z.append(out)

        return torch.cat(z, dim=1)

    def _make_grid(self, nx=20, ny=20, i=0):
        d = self.anchors[i].device
        t = self.anchors[i].dtype

        shape = 1, self.na, ny, nx, 2  # grid shape
        y, x = torch.arange(ny, device=d, dtype=t), torch.arange(nx, device=d, dtype=t)

        try:
            yv, xv = torch.meshgrid(y, x, indexing="ij")  # torch>=1.10
        except Exception as _:  # NOTE:老版本默认就是ij格式
            yv, xv = torch.meshgrid(y, x)  # torch>=0.7 compatibility

        # add grid offset, i.e. y = 2.0 * x - 0.5
        # grid = torch.stack((xv, yv), 2).expand(shape) - 0.5
        grid = torch.stack((xv, yv), 2).expand(shape)

        anchor_grid = (
            (self.anchors[i] * self.stride[i]).view((1, self.na, 1, 1, 2)).expand(shape)
        )
        # grid: (1, na, h, w, xy)
        # anchor_grid: (1, na, h, w, wh)
        return grid.to(device=self.device), anchor_grid.to(device=self.device)


### =============================================================================================


### =============================================================================================
class RunModel:
    def __init__(
        self,
        onnx_path,
        input_bchw,
        input_type=np.float32,
        mean=0.0,
        std=1.0,
        device=-1,
    ):
        """

        @param onnx_path:
        @param input_bchw:
        @param input_type: 模型输入的类型
        @param mean: 均值
        @param std: 方差
        @param device: 是否使用GPU
        """
        super().__init__()
        if device >= 0:
            # 设置使用CUDA推理
            providers = (("CUDAExecutionProvider", {"device_id": device}),)
        else:
            # 设置使用CPU推理
            providers = ("CPUExecutionProvider",)

        # 模型
        self.model = onnxruntime.InferenceSession(onnx_path, providers=providers)
        self.input_bchw = tuple(input_bchw)
        self.input_type = input_type

        self.mean = mean
        self.std = std

    def __call__(self, image):
        if image.ndim != 4 and image.shape != self.input_bchw:
            print(
                "ERROR: image.ndim != 4 and image.shape != self.input_bchw, please check it."
            )
            return list()

        if isinstance(image, torch.Tensor):
            image = image.detach().cpu().numpy()

        image = image.astype(self.input_type)
        # 准备输入数据
        # 假设模型的输入是一个形状为(1, 3, 224, 224)的图像
        input_name = self.model.get_inputs()[0].name

        if self.input_type == np.float32:
            image = (image.copy() - self.mean) / self.std

        # 执行推理 outputs:list
        outputs = self.model.run(None, {input_name: image})

        return outputs


### =============================================================================================
# 推理函数,测试mAP
def run(
    model_name,
    model: RunModel,
    anchors,
    names,
    input_bchw,
    data_path,
    save_root,
    kpt_shape=(0, 3),
    device=-1,
    conf_thres=0.3,
    iou_thres=0.45,
    max_det=300,
    kpt_v_thres=0.5,
    has_conf=True,
    scale_outputs=(1.0, 1.0, 1.0),
    is_save_no_det=True,
    is_save_txt=True,
    end2end=False,
):
    """
    @description: 主体推理函数
    @param executor {} : Knight包装的模型
    @param scale_outputs {} : 反量化系数
    @param is_quant {bool} :
    @return {}
    """
    # 获取输入的bchw
    # 类别数量
    nc = len(names)
    print(f"scale_outputs: {scale_outputs}")
    # 关键点个数
    kpt_num = kpt_shape[0] * kpt_shape[1]

    # todo:添加新功能,直接定位到对应位置
    if "yolov5" in model_name.lower() and len(anchors) > 0:
        # 带anchors的yolov5
        post_process = YOLOv5PostProcess(
            nc=nc,
            anchors=anchors,
            kpt_shape=kpt_shape,
            has_conf=has_conf,  # !如果不包含置信度,设置为False,更改较多,用到时自己根据实际情况分析
            device=device,  # 量化过程中一直是cpu
        )
    elif "yolov5" in model_name.lower() and len(anchors) == 0:
        # 不带anchors的yolov5, 类似yolov8的处理
        post_process = YOLOv8PostProcess(
            nc=nc,
            kpt_shape=kpt_shape,
            device=device,  # 量化过程中一直是cpu
        )
    # todo:添加新功能,直接定位到对应位置
    # !可能需要添加更多的判断
    elif "yolov8" in model_name.lower():
        post_process = YOLOv8PostProcess(
            nc=nc,
            kpt_shape=kpt_shape,
            device=device,  # 量化过程中一直是cpu
        )
    elif "yolov11" in model_name.lower() or "yolo11" in model_name.lower():
        post_process = YOLOv8PostProcess(
            nc=nc,
            kpt_shape=kpt_shape,
            device=device,  # 量化过程中一直是cpu
        )
    elif "yolo26" in model_name.lower():
        post_process = YOLO26PostProcess(
            nc=nc,
            kpt_shape=kpt_shape,
            device=device,  # 量化过程中一直是cpu
        )
    else:
        raise ValueError("model_name is not supported")

    # 遍历数据跑结果
    if os.path.isdir(data_path):
        # 遍历目录
        for root, _, files in os.walk(data_path):
            if len(files):
                bar = tqdm(
                    sorted(files),
                    desc=data_path,
                    bar_format="{l_bar}{bar:10}{r_bar}",
                )
            else:
                bar = list()

            for file in bar:
                file_name, suffix = os.path.splitext(file)
                if suffix.lower() not in IMG_FORMATS:
                    continue

                # 图片路径
                image_path = os.path.join(root, f"{file}")
                image_bgr = cv2.imread(image_path)
                if image_bgr is None:
                    continue

                # 处理图片成输入尺寸
                target_height, target_width = input_bchw[2], input_bchw[3]
                # 前处理  BGR -> RGB && resize
                img, ratio, dw, dh = pre_process_resize_img(
                    image_bgr.copy()[..., ::-1],
                    (target_width, target_height),
                )
                # (h, w, c) -> (c, h, w);
                img = np.transpose(img, (2, 0, 1))
                img = np.ascontiguousarray(img).astype(dtype=np.uint8)  # 数据连续

                # 推理
                preds_list = model(np.expand_dims(img, axis=0))

                # 乘以反量化系数
                for i, scale in enumerate(scale_outputs):
                    preds_list[i] = torch.tensor(preds_list[i]) * scale
                    if device >= 0:
                        preds_list[i] = preds_list[i].to(
                            device=torch.device(f"cuda:{device}")
                        )

                # 后处理 (b, 5040, 56):[x y w h conf x y v x y v ...]
                outputs = post_process.post_process(preds_list, xywh=True)
                outputs = outputs.permute(0, 2, 1).contiguous()

                # !如果是yolov5模型输出是(b, N, 4+1+nc+nm),为了和yolov8兼容在送入到NMS前需要将conf和类别分数合并
                # !yolov5-face只有一个类别,只有conf,没有类别分数,只有yolov5输出的是
                # 只有yolov5带有anchors时才会出现这种情况
                if hasattr(post_process, "anchors") and getattr(
                    post_process, "has_conf", False
                ):
                    # 将conf和cls_score拆开
                    conf = outputs[:, 4:5, :]  # (b, 4+1+nc+nm, 1)
                    cls_scores = outputs[:, 5 : 5 + nc, :]  # (b, 4+1+nc+nm, nc)
                    cls_scores *= conf  # (b, 4+nc+nm, N)

                    outputs = torch.cat(
                        [outputs[:, :4, :], cls_scores, outputs[:, 4 + 1 + nc :, :]],
                        dim=1,
                    )

                # NMS [(n, 56), ...]:[x y x y conf cls_id x y v x y v ...]
                outputs = non_max_suppression(
                    prediction=outputs,
                    conf_thres=conf_thres,
                    iou_thres=iou_thres,
                    agnostic=False,
                    max_det=max_det,
                    nc=nc,
                    end2end=end2end,
                )

                # ================================
                # (n, 4+1+1+nm)
                pred = outputs[0].detach().cpu()

                # bbox:(x y x y conf cls_id); (n, 6)
                bbox = pred[..., :6].clone()
                # 将预测结果映射到原图尺寸(x1, y1, x2, y2)
                ratio_pad = ((ratio, ratio), (dw, dh))
                bbox[..., :4] = scale_boxes(
                    img.shape[1:],
                    bbox[..., :4],
                    image_bgr.shape,
                    ratio_pad,
                ).round()

                if kpt_num > 0:
                    # kpts:(x y v x y v x y v ...); (n, 51) -> (n, 17, 3)
                    kpts = pred[..., 6:].clone().view(len(pred), *kpt_shape)
                    # 将关键点结果映射回原图
                    kpts = scale_coords(img.shape[1:], kpts, image_bgr.shape)
                    outputs = torch.cat(
                        (
                            bbox.clone(),
                            kpts.clone().view(-1, kpt_shape[0] * kpt_shape[1]),
                        ),
                        dim=-1,
                    )
                # todo:添加新功能
                # 如果支持别的功能需要填到这里
                else:
                    kpts = torch.empty((0, *kpt_shape), device=bbox.device)
                    outputs = bbox.clone()

                # 在原图上画框和画关键点
                annotated_frame = plot_box(
                    image_bgr.copy(),
                    boxes=bbox.detach().cpu().numpy(),
                    names=names,
                )
                if kpt_num > 0:
                    annotated_frame = plot_kpts(
                        annotated_frame,
                        kpts=kpts.detach().numpy(),
                        conf_thres=kpt_v_thres,
                    )
                # todo:添加新功能
                # 新功能绘制新的box在这里添加

                # 保存图片
                if len(outputs) > 0:
                    save_image_root = os.path.join(save_root, "img")
                    save_image_path = image_path.replace(data_path, save_image_root)
                    os.makedirs(os.path.dirname(save_image_path), exist_ok=True)
                    cv2.imwrite(save_image_path, annotated_frame)

                    if is_save_txt:
                        # 保存txt
                        save_txt_root = os.path.join(save_root, "txt")
                        save_txt_path = image_path.replace(
                            data_path, save_txt_root
                        ).replace(suffix, ".txt")
                        os.makedirs(os.path.dirname(save_txt_path), exist_ok=True)
                        # 保存格式:cls_id x y w h conf x y v x y v ...
                        img_h, img_w = image_bgr.shape[:2]
                        with open(save_txt_path, "w", encoding="utf-8") as wFile:
                            for out in outputs:
                                out = out.detach().cpu().numpy()
                                # bbox:(x y x y conf cls_id); (n, 6) 转 (cls_id x y w h conf)
                                out[..., [0, 2]] /= img_w
                                out[..., [1, 3]] /= img_h
                                bbox_txt = (
                                    f"{int(out[5])} "
                                    + " ".join([f"{x:.6f}" for x in out[:4]])
                                    + f" {out[4]:.3f}"
                                )

                                if kpt_num > 0:
                                    # 归一化
                                    out[..., 6 :: kpt_shape[1]] /= img_w
                                    out[..., 7 :: kpt_shape[1]] /= img_h
                                    kpts_txt = ""
                                    for _idx, kpt_v in enumerate(out[6:]):
                                        if (_idx + 1) % kpt_shape[1] == 0:
                                            kpts_txt += f" {kpt_v:.3f}"
                                        else:
                                            kpts_txt += f" {kpt_v:.6f}"
                                    wFile.write(f"{bbox_txt} {kpts_txt}\n")
                                else:
                                    wFile.write(f"{bbox_txt}\n")
                else:
                    # 没有检测目标
                    if is_save_no_det:
                        save_image_root = os.path.join(save_root, "no_det_img")
                        save_image_path = image_path.replace(data_path, save_image_root)
                        os.makedirs(os.path.dirname(save_image_path), exist_ok=True)
                        cv2.imwrite(save_image_path, image_bgr)

    else:
        print(f"{data_path} is not exists")
        exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="YOLO Run Quant Onnx",
        formatter_class=type(
            "",
            (
                argparse.ArgumentDefaultsHelpFormatter,
                argparse.RawTextHelpFormatter,
            ),
            {},
        ),
    )
    parser.add_argument(
        "--config",
        required=True,
        type=str,
        help="量化后生成的json配置文件",
    )
    parser.add_argument(
        "--onnx_path",
        required=True,
        type=str,
        help="量化后onnx路径",
    )
    parser.add_argument(
        "--conf_thres",
        type=float,
        default=0.25,
        help="置信度阈值,default=0.25",
    )
    parser.add_argument(
        "--iou_thres",
        type=float,
        default=0.45,
        help="iou阈值,default=0.45",
    )
    parser.add_argument(
        "--max_det",
        type=int,
        default=300,
        help="每张图片最多检测目标个数,default=300",
    )
    parser.add_argument(
        "--data_path",
        required=True,
        type=str,
        help="一个目录路径",
    )
    parser.add_argument(
        "--save_root",
        required=True,
        type=str,
        help="保存结果的路径",
    )
    parser.add_argument(
        "--input_type",
        type=str,
        choices=["uint8", "float32"],
        help="输入图片数据类型,如果是float32,会自动对输入进行归一化;",
    )
    parser.add_argument(
        "--device",
        default=-1,
        type=int,
        choices=list(range(-1, 8)),
        help="GPU:0~7;CPU:-1;default=-1",
    )
    parser.add_argument(
        "--is_save_no_det",
        action="store_true",
        help="是否保存没有检测到目标的图片,default=False",
    )
    parser.add_argument(
        "--is_save_txt",
        action="store_true",
        help="是否保存txt,default=False",
    )

    args = parser.parse_args()

    # 确定输入的路径都存在
    if os.path.exists(args.config):
        with open(args.config, "r", encoding="utf-8") as rFile:
            information = json.load(rFile)
    else:
        print(f"{args.config} is not exists")
        exit(1)
    if os.path.exists(args.onnx_path):
        onnx_path = os.path.abspath(args.onnx_path)
    else:
        print(f"{args.onnx_path} is not exists")
        exit(1)
    if os.path.exists(args.data_path):
        data_path = os.path.abspath(args.data_path)
    else:
        print(f"{args.data_path} is not exists")
        exit(1)
    save_root = os.path.abspath(args.save_root)

    # 确定输入类型
    if args.input_type == "uint8":
        input_type = np.uint8
    elif args.input_type == "float32":
        input_type = np.float32
    else:
        raise ValueError("input_type is not supported")
    # 解析配置信息
    is_save_no_det = args.is_save_no_det
    is_save_txt = args.is_save_txt

    ## 当前工程的名字
    ## chip
    ## project_name
    ## model_name
    ## version
    ## algo_version
    ## scale_outputs
    ## names
    ## anchors : yolov5是一个双层list,yolov8是一个空list
    ## input_bchw : (1, 3, 384, 640)
    ## use_anchors : bool
    ## kpt_shape : (N, 3)
    ## kpt_v_thres : 0.5,
    ## end2end : yolo26
    project_name = information["project_name"]
    version = information["version"]
    model_name = information["model_name"]
    algo_version = information["algo_version"]
    scale_outputs = information["scale_outputs"]
    names = information["names"]
    anchors = information["anchors"]
    input_bchw = information["input_bchw"]
    use_anchors = information["use_anchors"]
    use_keypoints = information["use_keypoints"]
    kpt_shape = information["kpt_shape"]
    kpt_v_thres = information["kpt_v_thres"]
    has_conf = information["has_conf"]
    end2end = information["end2end"]

    save_root = os.path.join(
        save_root, f"{algo_version}_{project_name}_{model_name}_results"
    )

    # 加载模型
    onnx_model = RunModel(
        onnx_path=onnx_path,
        input_bchw=input_bchw,
        input_type=input_type,
        mean=0.0,
        std=255.0,
        device=args.device,
    )

    # 运行模型
    run(
        model_name=model_name,
        model=onnx_model,
        anchors=anchors,
        names=names,
        input_bchw=input_bchw,
        kpt_shape=kpt_shape,
        device=args.device,
        data_path=data_path,
        save_root=save_root,
        conf_thres=args.conf_thres,
        iou_thres=args.iou_thres,
        max_det=args.max_det,
        kpt_v_thres=kpt_v_thres,
        has_conf=has_conf,
        scale_outputs=scale_outputs,
        is_save_no_det=is_save_no_det,
        is_save_txt=is_save_txt,
    )


### =============================================================================================
