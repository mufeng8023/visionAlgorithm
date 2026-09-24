# scripts 目录

<!-- vscode-markdown-toc -->
- [scripts 目录](#scripts-目录)
  - [概述](#概述)
  - [脚本说明](#脚本说明)
    - [pt2onnx.py](#pt2onnxpy)
    - [run_onnx_img.py](#run_onnx_imgpy)
  - [使用流程](#使用流程)
  - [扩展指南](#扩展指南)

<!-- vscode-markdown-toc -->

## 概述

`scripts/` 目录包含 Python 辅助脚本, 用于将 PyTorch 训练的 YOLO 系列模型 (YOLOv5 / YOLOv8 / YOLO11 / YOLO26) 转换为 ONNX 格式, 并基于 ONNX Runtime 对图片进行推理验证。这些脚本是 C++ 主工程的前置工具链, 帮助开发者快速完成模型导出与精度验证。

## 脚本说明

### pt2onnx.py

**功能**: 将 PyTorch 训练的 YOLO 系列模型 (`.pt`) 转换为 ONNX 格式, 并利用 `onnxsim` 进行模型简化。

**支持的模型**:

| 模型系列                 | 检测头类型        | 说明                         |
| ------------------------ | ----------------- | ---------------------------- |
| YOLOv5 (anchor-base)     | `YOLOv5DetectNew` | 带 anchors 的传统 YOLOv5     |
| YOLOv5 (anchor-free)     | `DetectNew`       | 类似 YOLOv8 的无 anchor 模式 |
| YOLOv8 / YOLO11 / YOLO12 | `DetectNew`       | 标准检测头                   |
| YOLOv8 / YOLO11 Pose     | `YOLOv8PoseNew`   | 姿态估计 (关键点检测)        |
| YOLO26 Pose              | `YOLO26PoseNew`   | 新一代姿态估计头             |

**核心特性**:

- **检测头重写**: 将原始 YOLO 的复杂检测头替换为简化的前向逻辑, 使 ONNX 输出为纯特征图格式 `(b, no, h, w)`, 便于 C++ 端后处理
- **可选 BN 归一化层**: 支持在模型前端插入 `BatchNorm2d` 层, 将输入归一化 `img / 255.0` 固化到模型中, 减少预处理负担
- **onnxsim 简化**: 导出后自动使用 `onnxsim` 进行常量折叠与图优化
- **输入尺寸灵活**: 支持通过 `--input_bchw` 指定任意输入分辨率

**命令行参数**:

| 参数              | 类型   | 默认值        | 说明                                                   |
| ----------------- | ------ | ------------- | ------------------------------------------------------ |
| `--model_name`    | str    | 必填          | 模型名称, 如 `yolov5` / `yolov8` / `yolo11` / `yolo26` |
| `--use_anchor`    | flag   | False         | 是否使用 anchors (仅 YOLOv5 anchor-base 需开启)        |
| `--input_bchw`    | int[4] | [1,3,384,640] | 输入 shape, 常见: 180P / 240P / 360P / 720P / 1080P    |
| `--add_bn`        | flag   | False         | 是否在模型前端添加 BN 归一化层                         |
| `--input_dtype`   | str    | float32       | 输入数据类型, 可选 `float32` / `uint8`                 |
| `--weight_path`   | str    | 必填          | PyTorch 权重路径 (`.pt`)                               |
| `--save_path`     | str    | 必填          | ONNX 保存路径                                          |
| `--opset_version` | int    | 14            | ONNX opset 版本                                        |

**使用示例**:

```bash
# YOLOv8 检测模型转换
python scripts/pt2onnx.py \
    --model_name=yolov8 \
    --input_bchw=1 3 384 640 \
    --add_bn \
    --weight_path=path/to/yolov8n.pt \
    --save_path=onnx/yolov8n-bn.onnx

# YOLOv5 anchor-base 模型转换
python scripts/pt2onnx.py \
    --model_name=yolov5 \
    --use_anchor \
    --input_bchw=1 3 384 640 \
    --weight_path=path/to/yolov5s.pt \
    --save_path=onnx/yolov5s-bn.onnx
```

---

### run_onnx_img.py

**功能**: 基于 ONNX Runtime 加载 ONNX 模型, 对指定目录下的图片进行推理, 并保存可视化结果 (检测框 / 关键点) 与标注文件。

**核心特性**:

- **多模型支持**: 兼容 YOLOv5 (anchor-base / anchor-free)、YOLOv8、YOLO11、YOLO26 的检测与姿态估计模型
- **后处理管线**: 内置完整的后处理流程, 包括:
  - 图像预处理 (等比例缩放 + 填充)
  - 特征图解码 (`YOLOv5PostProcess` / `YOLOv8PostProcess` / `YOLO26PostProcess`)
  - NMS 非极大值抑制 (支持普通 NMS 与旋转框 NMS)
  - 坐标映射回原图
- **结果可视化**: 自动绘制检测框、类别标签、置信度分数以及姿态关键点骨架
- **标注导出**: 支持保存归一化的 YOLO 格式 `.txt` 标注文件
- **GPU 加速**: 支持 CUDA 加速推理

**后处理类说明**:

| 类名                | 适用模型                               | 说明                        |
| ------------------- | -------------------------------------- | --------------------------- |
| `YOLOv5PostProcess` | YOLOv5 (anchor-base)                   | 基于 anchors 的坐标解码     |
| `YOLOv8PostProcess` | YOLOv8 / YOLO11 / YOLOv5 (anchor-free) | anchor-free 坐标解码        |
| `YOLO26PostProcess` | YOLO26                                 | 新一代 anchor-free 坐标解码 |

**命令行参数**:

| 参数               | 类型  | 默认值 | 说明                                                         |
| ------------------ | ----- | ------ | ------------------------------------------------------------ |
| `--config`         | str   | 必填   | 模型配置文件 (JSON), 包含 names / anchors / kpt_shape 等信息 |
| `--onnx_path`      | str   | 必填   | ONNX 模型路径                                                |
| `--conf_thres`     | float | 0.25   | 置信度阈值                                                   |
| `--iou_thres`      | float | 0.45   | NMS IoU 阈值                                                 |
| `--max_det`        | int   | 300    | 每张图片最大检测目标数                                       |
| `--data_path`      | str   | 必填   | 待推理图片目录                                               |
| `--save_root`      | str   | 必填   | 结果保存根目录                                               |
| `--input_type`     | str   | 必填   | 输入数据类型, `uint8` 或 `float32`                           |
| `--device`         | int   | -1     | GPU 设备编号 (-1 为 CPU)                                     |
| `--is_save_no_det` | flag  | False  | 是否保存无检测目标的图片                                     |
| `--is_save_txt`    | flag  | False  | 是否保存 YOLO 格式标注文件                                   |

**使用示例**:

```bash
# 使用 CPU 推理
python scripts/run_onnx_img.py \
    --config=config/yolov8nDetFace-bn.ini \
    --onnx_path=onnx/yolov8nDetFace-bn.onnx \
    --data_path=test_img \
    --save_root=results \
    --input_type=uint8

# 使用 GPU 推理
python scripts/run_onnx_img.py \
    --config=config/yolov8nDetPose-bn.ini \
    --onnx_path=onnx/yolov8nPose-bn.onnx \
    --data_path=test_img \
    --save_root=results \
    --input_type=uint8 \
    --device=0
```

## 使用流程

典型的模型部署流程如下:

1. **模型训练**: 使用 PyTorch 训练 YOLO 系列模型, 得到 `.pt` 权重文件
2. **模型导出**: 使用 `pt2onnx.py` 将 `.pt` 转换为 ONNX 格式
3. **精度验证**: 使用 `run_onnx_img.py` 对测试图片进行推理, 验证导出精度
4. **C++ 部署**: 将 ONNX 模型放入 `onnx/` 目录, 编写对应的 INI 配置文件, 使用 C++ 主工程进行部署

## 扩展指南

如需添加新的模型或任务 (如 OBB / Segment), 可在脚本中搜索 `todo:添加新功能` 定位到需要修改的位置:

- **pt2onnx.py**: 添加新的检测头类, 并在 `ultralytics_transfer_model` 中添加对应的 `isinstance` 判断分支
- **run_onnx_img.py**: 添加新的后处理类, 并在 `run` 函数中添加对应的模型名称判断分支, 同时在 `YOLOv8PostProcess` 或 `YOLO26PostProcess` 中添加对应的后处理逻辑
