# DetPostProcess26 YOLOv26 检测后处理

<!-- vscode-markdown-toc -->
- [DetPostProcess26 YOLOv26 检测后处理](#detpostprocess26-yolov26-检测后处理)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [`process_one` (单层特征图处理)](#process_one-单层特征图处理)
    - [`run` (入口)](#run-入口)
  - [边界框解码公式](#边界框解码公式)

<!-- vscode-markdown-toc -->

## 概述

`DetPostProcess26` 是 `BasePostProcess` 的派生类, 专门处理 YOLOv26 系列 anchor-free 端到端检测模型的输出解码。与 YOLOv8 不同, YOLOv26 是端到端模型, 导出时已内置 NMS, 后处理仅需按分数排序并截取前 `max_det` 个结果。

> **注意**: 导出 ONNX 模型时, 类别分数已经过 sigmoid 激活, 因此后处理中直接使用原始值, 无需额外调用 sigmoid。sigmoid 可通过 NPU 等硬件加速完成。

## 类定义

**文件**: `lib/detector/DetPostProcess26.hpp`  
**命名空间**: `yolo`

```cpp
class DetPostProcess26 : public BasePostProcess {
private:
    uint32 batch_size;
    uint32 nc;                    // 类别数量
    std::vector<float32> scale_outputs; // 反量化系数
    bool has_conf = false;        // YOLOv26 无独立置信度通道
    std::vector<float32> conf_thrs;     // 各类别置信度阈值
    float32 min_conf;
    float32 iou_thrs;
    uint32 max_det;
    bool agnostic;
    uint32 na = 1;                // anchor-free, na 固定为 1
    uint32 no;
    uint32 nl;
    std::vector<uint32> strides;
};
```

## 核心流程

### `process_one` (单层特征图处理)

遍历特征图的每个网格位置:

1. 遍历所有类别, 根据 `agnostic` 模式处理:
   - `agnostic = true`: 每个位置仅保留最大分数类别
   - `agnostic = false`: 保留所有超过阈值的类别
2. 解码边界框坐标 (x1, y1, x2, y2 -> x_center, y_center, width, height)
3. 存储结果到 `ObjectBuffer`
4. 如果一个位置有多个类别, 复制边界框信息并分别设置不同类别

### `run` (入口)

```cpp
void run(const std::vector<NetOutput>& outputs,
         std::vector<ObjectBuffer>& results) override;
```

1. 遍历所有输出层, 调用 `process_one` 解析每层特征图
2. 调用 `non_max_suppression` 并设置 `end2end = true`, 执行排序截取

## 边界框解码公式

YOLOv26 使用 anchor-free 的解码方式 (网络输出已过 sigmoid, 直接使用原始值):

```
x_center = (grid_x + 0.5 + (dx2 - dx1) * 0.5) * stride
y_center = (grid_y + 0.5 + (dy2 - dy1) * 0.5) * stride
width = (dx2 + dx1) * stride
height = (dy2 + dy1) * stride
```
