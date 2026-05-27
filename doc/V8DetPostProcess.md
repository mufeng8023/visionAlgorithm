# V8DetPostProcess YOLOv8 检测后处理

<!-- vscode-markdown-toc -->
- [V8DetPostProcess YOLOv8 检测后处理](#v8detpostprocess-yolov8-检测后处理)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [`process_one` (单层特征图处理)](#process_one-单层特征图处理)
    - [`run` (入口)](#run-入口)
  - [边界框解码公式](#边界框解码公式)

<!-- vscode-markdown-toc -->

## 概述

`V8DetPostProcess` 是 `BasePostProcess` 的派生类, 专门处理 YOLOv8 系列 anchor-free 检测模型的输出解码。与 YOLOv5 不同, YOLOv8 直接预测 (x1, y1, x2, y2) 形式的边界框偏移量, 且没有独立的置信度通道。

## 类定义

**文件**: `lib/detector/V8DetPostProcess.hpp`  
**命名空间**: `yolo`

```cpp
class V8DetPostProcess : public BasePostProcess {
private:
    uint32 batch_size;
    uint32 nc;                    // 类别数量
    std::vector<float32> scale_outputs; // 反量化系数
    bool has_conf = false;        // YOLOv8 无独立置信度通道
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

1. 遍历所有类别, 找出最大分数和对应类别
2. 根据各类别阈值过滤低置信度检测
3. 解码边界框坐标 (x1, y1, x2, y2 -> x_center, y_center, width, height)
4. 存储结果到 `ObjectBuffer`

### `run` (入口)

```cpp
void run(const std::vector<NetOutput>& outputs,
         std::vector<ObjectBuffer>& results) override;
```

1. 遍历所有输出层, 调用 `process_one` 解析每层特征图
2. 对所有检测结果执行 NMS

## 边界框解码公式

YOLOv8 使用 anchor-free 的解码方式, 直接预测左上角和右下角相对于网格中心的偏移:

```
x_center = (grid_x + 0.5 + (dx2 - dx1) * 0.5) * stride
y_center = (grid_y + 0.5 + (dy2 - dy1) * 0.5) * stride
width = (dx2 + dx1) * stride
height = (dy2 + dy1) * stride
```
