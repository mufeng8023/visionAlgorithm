"""
Author       : gxs
Date         : 2026-05-28 17:03:44
LastEditors  : gxs
LastEditTime : 2026-05-28 17:03:47
FilePath     : /visionAlgorithm/scripts/pt2onnx.py
Description  :

Copyright (c) 2026 by gxs, All Rights Reserved.
"""

import argparse
import os
import sys
import traceback

import onnxsim
import torch
import torch.nn as nn
from tabulate import tabulate

import onnx

# yolov5工程路径
YOLOV5_ROOT = "/mnt/E/CodeFiles/Python/YOLO/yolov5"
# ultralytics工程路径
ULTRALYTICS_ROOT = "/mnt/E/CodeFiles/Python/YOLO/ultralytics"

# 将yolov5工程路径添加到sys.path中
if YOLOV5_ROOT not in sys.path:
    sys.path.insert(0, YOLOV5_ROOT)
    print(f"{YOLOV5_ROOT} added to sys.path")

if ULTRALYTICS_ROOT not in sys.path:
    sys.path.insert(0, ULTRALYTICS_ROOT)
    print(f"{ULTRALYTICS_ROOT} added to sys.path")


### =============================================================================================
from models.common import DetectMultiBackend as YOLOV5
from models.yolo import Detect as YoloV5Detect
from ultralytics import YOLO
from ultralytics.models.yolo.model import DetectionModel
from ultralytics.nn.modules.head import (
    OBB,
    OBB26,
    Detect,
    Pose,
    Pose26,
    Segment,
    Segment26,
)


### =============================================================================================
class YOLOv5DetectNew(nn.Module):
    def __init__(self, head_old: YoloV5Detect):
        super().__init__()
        # 更新所有状态
        self.__dict__.update(head_old.__dict__)
        # 显示获取指定变量
        self.nl = head_old.nl
        self.m = head_old.m

    def forward(self, x):
        for i in range(self.nl):
            x[i] = torch.sigmoid(self.m[i](x[i]))

        return x


class DetectNew(nn.Module):
    """
    description: YOLOV8 / YOLOV11 / YOLO26 的检测头
    """

    def __init__(self, head_old: Detect):
        super().__init__()
        # 更新所有状态
        self.__dict__.update(head_old.__dict__)

        # 显示获取指定变量
        # bool
        self.end2end = head_old.end2end
        # 获取两个字典的函数
        if self.end2end:
            # 只有是True是才存在的属性
            self.one2one = head_old.one2one

        self.one2many = head_old.one2many
        self.dfl = head_old.dfl

    def forward(self, x):
        """
        Concatenates and returns predicted bounding boxes and class probabilities.
        Args:
            x: [tensor(b, c1, h1, w1), tensor(b, c2, h2, w2), tensor(b, c2, h3, w3)]

        Returns:
        """
        dets = []

        # 看看是哪种头
        if self.end2end:  # 端到端的
            box_head = self.one2one["box_head"]
            cls_head = self.one2one["cls_head"]
        else:  # 原来的yolov8和yolov11
            box_head = self.one2many["box_head"]
            cls_head = self.one2many["cls_head"]
        # box_head: 都是 nn.ModuleList
        # cls_head : 都是 nn.ModuleList

        # shapes = []  # (b, self.no, h, w)
        for i in range(self.nl):
            # 这里拼接起来了，下面还得拆开，cv2的输出是box，cv3的输出是score
            b, _, H, W = x[i].shape  # 检测和分类不会更改H、W
            # out = torch.cat((self.cv2[i](x[i]), self.cv3[i](x[i])), dim=1)
            box = box_head[i](x[i])  # (b, 64, h, w)
            cls = torch.sigmoid(cls_head[i](x[i]))  # (b, nc, h, w)

            ### ===============================================================================
            ### NOTE:将dfl分别在每个特征图上计算，不合并后计算了，
            if not isinstance(self.dfl, nn.Identity):
                box = box.view(b, -1, H * W)  # (b, 64, H*W)
                box = self.dfl(box)  # (b, 1, 4, H*W)
                box = box.view(b, -1, H, W)  #

            # (b, 4 + nc, h, w)
            out = torch.cat([box, cls], dim=1)
            ### ===============================================================================

            dets.append(out)

        return dets


class YOLOv8PoseNew(nn.Module):
    """
    description: YOLOV8 / YOLOV11 的Pose头
    """

    def __init__(self, head_old: Pose):
        super().__init__()
        # 更新所有状态
        self.__dict__.update(head_old.__dict__)

        # 显示获取一些状态
        # bool
        self.end2end = head_old.end2end
        # 获取两个字典的函数
        if self.end2end:
            # 只有是True是才存在的属性
            self.one2one = head_old.one2one
        self.one2many = head_old.one2many

        # 检测头
        self.detect = DetectNew.forward
        # 其余关键信息
        self.kpt_shape = head_old.kpt_shape
        # 关键点信息数量 nk = kpt_num * kpt_dim
        self.nk = head_old.nk

    def forward(self, x):
        """
        Perform forward pass through YOLO model and return predictions.
        Args:
            x: [tensor(b, c1, h1, w1), tensor(b, c2, h2, w2), tensor(b, c2, h3, w3)]

        Returns:

        """
        # [out1(b, 5, h1, w1), out2(b, 5, h2, w2), out3(b, 5, h3, w3)]
        det_list = self.detect(self, x)

        # 关键点的计算
        # 这三个变量都是 nn.ModuleList
        if self.end2end:
            pose_head = self.one2one["pose_head"]
        else:
            pose_head = self.one2many["pose_head"]

        # 三个特征层的输出
        kpt_list = []
        for i in range(self.nl):
            # 将一层检测的关键点特征图保存起来
            # (b, 17*3, h, w)
            kpt = pose_head[i](x[i])

            b, c, h, w = kpt.shape

            # (b, 17*3, h, w) -> (b, 17*3, h*w)
            kpt = kpt.view(b, c, h * w)
            # (b, 17*3, h*w) -> (b, h*w, 17*3)
            kpt = kpt.transpose(2, 1)
            # (b, h*w, 17*3) -> (b, h*w, 17, 3)
            kpt = kpt.view(b, h * w, *self.kpt_shape)

            # (b, h*w, 17, 2), (b, h*w, 17, 1)
            # kpt_xy, kpt_v = torch.split(kpt, [2, 1], dim=2)
            kpt_xy, kpt_v = kpt.split([2, 1], dim=-1)

            kpt_v = torch.sigmoid(kpt_v)

            # (b, h*w, 17, 2), (b, h*w, 17, 1) -> (b, h*w, 17, 3)
            kpt = torch.cat([kpt_xy, kpt_v], dim=-1)
            # (b, h*w, 17, 3) -> (b, h*w, 17*3)
            kpt = kpt.view(b, h * w, c)
            # (b, h*w, 17*3) -> (b, 17*3, h*w)
            kpt = kpt.transpose(2, 1).contiguous()

            # (b, 17*3, h*w) -> (b, 17*3, h, w)
            # kpt = kpt.view(b, c, h * w)
            kpt = kpt.view(b, c, h, w)

            kpt_list.append(kpt)

        outputs = []
        for det, kpt in zip(det_list, kpt_list):
            outputs.append(torch.cat((det, kpt), dim=1))

        # [out1(b, 5 + 17*3, h1, w1), out2(b, 5 + 17*3, h2, w2), out3(b, 5 + 17*3, h3, w3)]
        return outputs


class YOLO26PoseNew(nn.Module):
    def __init__(self, head_old: Pose26) -> None:
        super().__init__()
        # 更新所有状态
        self.__dict__.update(head_old.__dict__)
        self.flow_model = head_old.flow_model

        # 显示获取一些状态
        # bool
        self.end2end = head_old.end2end
        # 获取两个字典的函数
        if self.end2end:
            # 只有是True是才存在的属性
            self.one2one = head_old.one2one

        self.one2many = head_old.one2many
        # 检测头
        self.detect = DetectNew.forward
        # 其余关键信息
        self.kpt_shape = head_old.kpt_shape
        # 关键点信息数量 nk = kpt_num * kpt_dim
        self.nk = head_old.nk
        # NOTE: 猜测是每个关键点的重要程度
        self.nk_sigma = head_old.nk_sigma

    def forward(self, x):
        """
        pose 26 的新检测头
        :param x: 是一个list;
        """
        # 是一个list; [(b, 4 + nc, h, w), ...]
        det_list = self.detect(self, x)

        # 关键点的计算
        # 这三个变量都是 nn.ModuleList
        if self.end2end:
            pose_head = self.one2one["pose_head"]
            kpts_head = self.one2one["kpts_head"]
            # kpts_sigma_head = self.one2one["kpts_sigma_head"]
        else:
            pose_head = self.one2many["pose_head"]
            kpts_head = self.one2many["kpts_head"]
            # kpts_sigma_head = self.one2many["kpts_sigma_head"]

        # 三个特征层的输出
        kpt_list = []  # [(b, )]
        # 计算关键点
        for i in range(self.nl):
            # (b, c, h, w) -> (b, kpt_num * (kpt_dim + 2), h, w)
            feature = pose_head[i](x[i])
            # (b, kpt_num * (kpt_dim + 2), h, w) -> (b, kpt_num * kpt_dim, h, w)
            kpt = kpts_head[i](feature)
            # 获取其 shape
            b, c, h, w = kpt.shape
            # 转换关键点维度, 对分数求sigmoid
            # (b, kpt_num*3, h, w) -> (b, kpt_num*3, h*w)
            kpt = kpt.view(b, c, h * w)
            # (b, kpt_num*k_dim, h*w) -> (b, h*w, kpt_num*3)
            kpt = kpt.transpose(2, 1)
            # (b, h*w, kpt_num*3) -> (b, h*w, kpt_num, 3)
            kpt = kpt.view(b, h * w, *self.kpt_shape)

            # (b, h*w, kpt_num, 2), (b, h*w, kpt_num, 1)
            # kpt_xy, kpt_v = torch.split(kpt, [2, 1], dim=2)
            kpt_xy, kpt_v = kpt.split([2, 1], dim=-1)
            # 取sigmoid
            kpt_v = torch.sigmoid(kpt_v)

            # (b, h*w, kpt_num, 2), (b, h*w, kpt_num, 1) -> (b, h*w, kpt_num, 3)
            kpt = torch.cat([kpt_xy, kpt_v], dim=-1)
            # (b, h*w, kpt_num, 3) -> (b, h*w, kpt_num*3)
            kpt = kpt.view(b, h * w, c)
            # (b, h*w, kpt_num*3) -> (b, kpt_num*3, h*w)
            kpt = kpt.transpose(2, 1).contiguous()

            # (b, kpt_num*3, h*w) -> (b, kpt_num*3, h, w)
            kpt = kpt.view(b, c, h, w)

            kpt_list.append(kpt)

        # 拼接检测结果和关键点
        # det: (b, nc, h, w); kpt: (b, kpt_num * 3, h, w)
        outputs = list()
        for det, kpt in zip(det_list, kpt_list):
            out = torch.cat([det, kpt], dim=1)
            outputs.append(out)
        # [(b, 4 + nc + kpt_num * 3, h, w), ...]
        return outputs


# todo:添加新功能,直接定位到对应位置
# 需要添加重写后的seg或者OBB的head类


### =============================================================================================
# 转换函数
def yolov5_transfer_model(weight_path=None):
    """
    转换函数
    函数名称在量化时通过--model传进去
    @param weight_path:
    @return:
    """
    if weight_path is None:
        raise ValueError("weight_path is None")
    else:
        model = YOLOV5(
            weights=weight_path,
            device=torch.device("cpu"),
            dnn=False,
        )
    # 获取最后一个模块的名称
    name, _ = list(model.model.model.named_children())[-1]
    # 获取模型中最后一个模块的信息
    detect = getattr(model.model.model, name)
    # 新模块初始化
    new_det_head = YOLOv5DetectNew(detect)
    # 模型中替换新模块
    setattr(model.model.model, name, new_det_head)

    ## 返回之中必须包含字典中的模型 和 对应输入shape的Tensor
    # NOTE:要更改模型的输入大小, 需要在这里更改
    return model.model


def ultralytics_transfer_model(weight_path=None):
    """
    转换函数
    函数名称在量化时通过--model传进去
    @param weight_path:
    @return:
    """
    if weight_path is not None:
        yolo = YOLO(weight_path)
        model: DetectionModel = yolo.model
    else:
        raise ValueError("weight_path should not be None")

    # 获取最后一个模块，的名称
    name, sub_model = list(model.model.named_children())[-1]

    # todo:添加新功能,直接定位到对应位置
    if isinstance(sub_model, (OBB, Segment, OBB26, Segment26)):
        # !后续想支持的话在这里添加新的代码来转换模型
        raise ValueError(f"{name} is {type(sub_model)}; not support.")

    # 如果存在子类和父类先写子类后写父类
    elif isinstance(sub_model, Pose26):
        # 获取检测头的完整属性
        head_old = getattr(model.model, name)
        # 初始化一个模块，用来替换Pose头
        head_new = YOLO26PoseNew(head_old)
        print("Pose 使用 YOLO26PoseNew 替换")

    elif isinstance(sub_model, Pose):
        # 获取检测头的完整属性
        head_old = getattr(model.model, name)
        # 初始化一个模块，用来替换Pose头
        head_new = YOLOv8PoseNew(head_old)
        print("Pose 使用 YOLOv8PoseNew 替换")

    elif isinstance(sub_model, Detect):
        # 获取检测头的完整属性
        head_old = getattr(model.model, name)
        # 初始化一个模块，用来替换Detect头
        head_new = DetectNew(head_old)
        print("Detect 使用 DetectNew 替换")

    else:
        raise ValueError(f"{name} is {type(sub_model)}; not support.")

    # 模型中替换新模块
    setattr(model.model, name, head_new)

    ## 返回之中必须包含字典中的模型 和 对应输入shape的Tensor
    # NOTE:要更改模型的输入大小, 需要在这里更改
    return model


class TransDtype(nn.Module):
    def __init__(self, input_dtype=torch.float32):
        super().__init__()
        self.input_dtype = input_dtype

    def forward(self, x: torch.Tensor):
        return x.to(self.input_dtype)


class ModelWrapper(nn.Module):
    def __init__(
        self,
        model,
        input_channels=3,
        add_bn=False,
        mean=(0, 0, 0),
        std=(255, 255, 255),
    ):
        super().__init__()

        self.model = model

        if add_bn:
            # NOTE: 初始化一个BN，用来归一化 img / 255.
            new_bn = torch.nn.BatchNorm2d(input_channels)
            new_bn.weight = nn.Parameter(torch.ones((input_channels,)))
            new_bn.bias = nn.Parameter(torch.zeros((input_channels,)))
            new_bn.running_mean = torch.zeros((input_channels,)) + torch.FloatTensor(
                mean
            )
            new_bn.running_var = (
                torch.ones((input_channels,))
                * torch.FloatTensor(std)
                * torch.FloatTensor(std)
            )
            new_bn.eval()
            trans_dtype = TransDtype(input_dtype=torch.float32)
            self.model = nn.Sequential(trans_dtype, new_bn, model)

        # 添加转换数据类型
        self.model.eval()

    @torch.no_grad()
    def forward(self, x):
        return self.model(x)


def transfer_func_name(
    model_name: str,
    use_anchor: bool,
    input_bchw: list,
    add_bn: bool,
    input_dtype: torch.dtype,
    weight_path: str,
    save_path: str,
    opset_version=14,
):
    """
    description:
    param model_name {str} : 模型名称 yolov5/yolov8/yolov11 选择不同的转换策略
    param use_anchor {bool} : 是否使用anchors
    param input_bchw {list} : 输入的shape
    param add_bn {bool} : 是否添加BN
    param input_dtype {torch.dtype} : 输入数据的类型
    param weight_path {str} : 模型的pt权重路径
    param save_path {str} : 保存onnx的路径
    param opset_version {} : onnx的opset版本
    return {} 成功返回True, 失败返回False
    """

    if "yolov5" in model_name.lower():
        # 如果需要anchors
        if use_anchor:
            model = yolov5_transfer_model(weight_path)
        else:
            # 没有anchors的时候就是
            model = ultralytics_transfer_model(weight_path)

    elif "yolov8" in model_name.lower():
        model = ultralytics_transfer_model(weight_path)
    elif "yolov11" in model_name.lower() or "yolo11" in model_name.lower():
        model = ultralytics_transfer_model(weight_path)
    elif "yolo26" in model_name.lower():
        model = ultralytics_transfer_model(weight_path)
    elif "yolo12" in model_name.lower() or "yolov12" in model_name.lower():
        model = ultralytics_transfer_model(weight_path)

    # todo:添加新功能,直接定位到对应位置
    # 添加新模型
    else:
        raise ValueError("model_name is not supported")

    model = ModelWrapper(
        model,
        input_channels=input_bchw[1],
        add_bn=add_bn,
    )

    try:
        # 随机数输入
        if input_dtype == torch.float32:
            random_tensor = torch.randn(input_bchw, device="cpu", dtype=input_dtype)
        elif input_dtype == torch.uint8:
            random_tensor = torch.randint(
                0, 256, input_bchw, device="cpu", dtype=input_dtype
            )
        else:
            raise ValueError(f"input_dtype {input_dtype} is not supported")

        # 转换为onnx
        model = model.eval().to(device="cpu")
        torch.onnx.export(
            model,  # 要转换的模型
            (random_tensor,),  # 示例输入, 用于定义模型输入的形状和数据类型
            save_path,  # 保存onnx路径
            export_params=True,  # 是否导出模型的参数 (权重)。如果设置为 False, 则不会包含模型的参数
            opset_version=opset_version,  # ONNX 的操作集版本。
            do_constant_folding=True,  # 是否在导出时执行常量折叠优化
            input_names=[
                "input"
            ],  # 模型输入的名称列表。这些名称将在 ONNX 图中使用。如果不提供, 将使用默认名称。
            output_names=["out0", "out1", "out2"],  # 模型输出的名称列表
            verbose=False,
        )

        # 将保存的onnx模型加载进来
        onnx_model = onnx.load(save_path)

        # 获取模型的graph
        graph = onnx_model.graph

        # 打印输入信息
        print("Model Inputs:")
        for input_node in graph.input:
            input_bchw = [
                dim.dim_value for dim in input_node.type.tensor_type.shape.dim
            ]
            print(f"Name: {input_node.name}, Shape: {input_bchw}")

        # 打印输出信息
        print("\nModel Outputs:")
        for output in graph.output:
            output_shape = [dim.dim_value for dim in output.type.tensor_type.shape.dim]
            print(f"Name: {output.name}, Shape: {output_shape}")

        # 使用 onnxsim 进行模型简化
        model_simp, check = onnxsim.simplify(onnx_model)
        # 确认简化后的模型是有效的
        assert check, "Simplified ONNX model could not be validated"
        # 保存简化后的模型
        print(f"os.remove {save_path}")
        os.remove(save_path)
        print(f"onnxsim save {save_path}")
        onnx.save(model_simp, save_path)

        return True

    except Exception as e:
        print(f"Exception: {e}")
        print(traceback.format_exc())
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="将YOLO转换为ONNX",
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
        "--model_name",
        type=str,
        required=True,
        help="模型名称 yolov5/yolov8/yolov11 选择不同的转换策略;",
    )
    parser.add_argument(
        "--use_anchor",
        action="store_true",
        help="是否使用anchors, 针对带anchors的YOLOv5要开启该选项, 如果是yolov8中的yolov5则不用开启;",
    )
    parser.add_argument(
        "--input_bchw",
        type=int,
        nargs=4,
        default=[1, 3, 384, 640],
        help=(
            "输入的bchw; "
            "\n常见的是: "
            "\n180P: [1, 3, 192, 320]; "
            "\n240P: [1, 3, 256, 448]; "
            "\n360P: [1, 3, 384, 640]; "
            "\n720P: [1, 3, 736, 1280]; "
            "\n1080P: [1, 3, 1088, 1920]; "
        ),
    )
    parser.add_argument(
        "--add_bn",
        action="store_true",
        help="是否在模型开始添加BN层来计算归一化: img / 255.0;",
    )
    parser.add_argument(
        "--input_dtype",
        type=str,
        default="float32",
        choices=["float32", "uint8"],
        help="输入数据的数据类型;",
    )
    parser.add_argument(
        "--weight_path",
        type=str,
        required=True,
        help="模型的pt权重路径;",
    )
    parser.add_argument(
        "--save_path",
        type=str,
        required=True,
        help=(
            "保存onnx的路径; "
            "\n推荐命名示例: yolov5s_v0.0_W640_H384.onnx; "
            "\n如果是 anchor-free yolov5n-v8_v0.0_W640_H384.onnx; "
            "yolov8s_v0.0_W640_H384.onnx; "
        ),
    )
    parser.add_argument(
        "--opset_version",
        type=int,
        default=14,
        help="onnx的opset版本;",
    )
    args = parser.parse_args()

    # 使用tabulate 打印输入参数
    print(
        tabulate(
            list(args.__dict__.items()),
            headers=["参数", "值"],
            tablefmt="simple",
        )
    )

    # 参数获取
    model_name = args.model_name
    use_anchor = args.use_anchor
    input_bchw = args.input_bchw
    add_bn = args.add_bn
    weight_path = os.path.abspath(args.weight_path)
    save_path = args.save_path
    opset_version = args.opset_version

    if args.input_dtype == "uint8":
        input_dtype = torch.uint8
    elif args.input_dtype == "float32":
        input_dtype = torch.float32
    else:
        raise ValueError(f"不支持的输入数据类型: {args.input_dtype}")

    # 检查路径
    assert os.path.exists(weight_path), f"{weight_path} 不存在"
    # 创建目录, 防止出现问题
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    # 转换模型
    if transfer_func_name(
        model_name=model_name,
        use_anchor=use_anchor,
        input_bchw=input_bchw,
        add_bn=add_bn,
        input_dtype=input_dtype,
        weight_path=weight_path,
        save_path=save_path,
        opset_version=opset_version,
    ):
        print("转换成功")
        print(f"onnx模型保存在 {save_path}")
    else:
        print("转换失败")
