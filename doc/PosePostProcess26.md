# PosePostProcess26 YOLOv26 姿态后处理

<!-- vscode-markdown-toc -->
- [PosePostProcess26 YOLOv26 姿态后处理](#posepostprocess26-yolov26-姿态后处理)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [`process_one` (单层特征图处理)](#process_one-单层特征图处理)
    - [`run` (入口)](#run-入口)
  - [关键点解码](#关键点解码)

<!-- vscode-markdown-toc -->

## 概述

`PosePostProcess26` 是 `BasePostProcess` 的派生类, 专门处理 YOLOv26 系列 anchor-free 端到端姿态估计模型的输出解码。在检测框解码的基础上, 额外解析关键点 (keypoint) 信息。YOLOv26 是端到端模型, 导出时已内置 NMS, 后处理仅需按分数排序并截取前 `max_det` 个结果。

> **注意**: 导出 ONNX 模型时, 类别分数和关键点坐标已经过 sigmoid 激活, 因此后处理中直接使用原始值, 无需额外调用 sigmoid。sigmoid 可通过 NPU 等硬件加速完成。

## 类定义

**文件**: `lib/detector/PosePostProcess26.hpp`  
**命名空间**: `yolo`

```cpp
class PosePostProcess26 : public BasePostProcess {
private:
    // ... 检测相关参数 (同 DetPostProcess26)
    uint32 kpt_count;             // 关键点个数
    uint32 kpt_dim;               // 关键点维度 (2: x,y; 3: x,y,v)
    // ...
};
```

## 核心流程

### `process_one` (单层特征图处理)

在 YOLOv26 检测解码的基础上, 额外解析关键点信息:

1. 执行检测框解码 (同 DetPostProcess26)
2. 从特征图中提取每个关键点的 x, y 坐标
3. 如果 `kpt_dim == 3`, 额外提取可见性分数 v
4. 关键点坐标解码到特征图尺度
5. 如果一个位置有多个类别, 复制边界框和关键点信息并分别设置不同类别

### `run` (入口)

```cpp
void run(const std::vector<NetOutput>& outputs,
         std::vector<ObjectBuffer>& results) override;
```

1. 遍历所有输出层, 调用 `process_one` 解析每层特征图
2. 调用 `non_max_suppression` 并设置 `end2end = true`, 执行排序截取

## 关键点解码

关键点信息在输出通道中的排列顺序为:

```
[bbox(4) + nc + kpt0_x, kpt0_y, kpt0_v, kpt1_x, ...]
```

解码公式 (网络输出已过 sigmoid, 直接使用原始值):

```
kpt_x = (kpt_x_raw + grid_x + 0.5) * stride
kpt_y = (kpt_y_raw + grid_y + 0.5) * stride
kpt_v = kpt_v_raw  // 仅 kpt_dim == 3 时
```

> **与 YOLOv8-Pose 的区别**: YOLOv26 的关键点解码公式不同, 没有乘以 2.0 的缩放因子, 且偏移量计算方式为 `(raw + grid + 0.5)` 而非 `(raw * 2.0 + grid)`。
