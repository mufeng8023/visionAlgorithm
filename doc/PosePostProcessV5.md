# PosePostProcessV5 YOLOv5 姿态后处理

<!-- vscode-markdown-toc -->
- [PosePostProcessV5 YOLOv5 姿态后处理](#posepostprocessv5-yolov5-姿态后处理)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [`process_one` (单层特征图处理)](#process_one-单层特征图处理)
    - [`run` (入口)](#run-入口)
  - [关键点解码](#关键点解码)

<!-- vscode-markdown-toc -->

## 概述

`PosePostProcessV5` 是 `BasePostProcess` 的派生类, 专门处理 YOLOv5 系列 anchor-base 姿态估计模型的输出解码。在检测框解码的基础上, 额外解析关键点 (keypoint) 信息。

> **注意**: 导出 ONNX 模型时, 置信度分数、类别分数和关键点坐标已经过 sigmoid 激活, 因此后处理中直接使用原始值, 无需额外调用 sigmoid。sigmoid 可通过 NPU 等硬件加速完成。

## 类定义

**文件**: `lib/detector/PosePostProcessV5.hpp`  
**命名空间**: `yolo`

```cpp
class PosePostProcessV5 : public BasePostProcess {
private:
    // ... 检测相关参数 (同 DetPostProcessV5)
    uint32 kpt_count;             // 关键点个数
    uint32 kpt_dim;               // 关键点维度 (2: x,y; 3: x,y,v)
    // ...
};
```

## 核心流程

### `process_one` (单层特征图处理)

在 YOLOv5 检测解码的基础上, 额外解析关键点信息:

1. 执行检测框解码 (同 DetPostProcessV5)
2. 从特征图中提取每个关键点的 x, y 坐标
3. 如果 `kpt_dim == 3`, 额外提取可见性分数 v
4. 关键点坐标解码到特征图尺度

### `run` (入口)

```cpp
void run(const std::vector<NetOutput>& outputs,
         std::vector<ObjectBuffer>& results) override;
```

1. 遍历所有输出层, 调用 `process_one` 解析每层特征图
2. 对所有检测结果执行 NMS

## 关键点解码

关键点信息在输出通道中的排列顺序为:

```
[bbox(4) + conf(1) + nc + kpt0_x, kpt0_y, kpt0_v, kpt1_x, ...]
```

解码公式 (网络输出已过 sigmoid, 直接使用原始值):

```
kpt_x = (kpt_x_raw * anchor_w + grid_x) * stride
kpt_y = (kpt_y_raw * anchor_h + grid_y) * stride
kpt_v = kpt_v_raw  // 仅 kpt_dim == 3 时
```
