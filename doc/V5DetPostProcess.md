# V5DetPostProcess YOLOv5 检测后处理

<!-- vscode-markdown-toc -->
- [V5DetPostProcess YOLOv5 检测后处理](#v5detpostprocess-yolov5-检测后处理)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [`process_one` (单层特征图处理)](#process_one-单层特征图处理)
    - [`run` (入口)](#run-入口)
  - [边界框解码公式](#边界框解码公式)

<!-- vscode-markdown-toc -->

## 概述

`V5DetPostProcess` 是 `BasePostProcess` 的派生类, 专门处理 YOLOv5 系列 anchor-base 检测模型的输出解码。支持可选的置信度通道 (如 YOLOv5-Face)。

## 类定义

**文件**: `lib/detector/V5DetPostProcess.hpp`  
**命名空间**: `yolo`

```cpp
class V5DetPostProcess : public BasePostProcess {
private:
    uint32 batch_size;
    uint32 nc;                    // 类别数量
    std::vector<float32> scale_outputs; // 反量化系数
    bool has_conf;                // 是否存在置信度通道
    std::vector<float32> conf_thrs;     // 各类别置信度阈值
    float32 min_conf;             // 最小置信度阈值
    float32 iou_thrs;             // NMS IoU 阈值
    uint32 max_det;               // 最大检测目标数
    bool agnostic;                // 是否类别不敏感 NMS
    uint32 na;                    // 每个位置 anchor 数
    uint32 no;                    // 每个位置输出信息数
    uint32 nl;                    // 输出层数
    std::vector<uint32> strides;  // 各层步长
    std::vector<std::vector<float32>> anchors; // anchor 信息
};
```

## 核心流程

### `process_one` (单层特征图处理)

遍历特征图的每个网格位置, 对每个 anchor 执行:

1. 获取 box 置信度 (如果有 conf 通道)
2. 遍历所有类别, 找出最大分数和对应类别
3. 根据各类别阈值过滤低置信度检测
4. 解码边界框坐标 (x_center, y_center, width, height)
5. 存储结果到 `ObjectBuffer`

### `run` (入口)

```cpp
void run(const std::vector<NetOutput>& outputs,
         std::vector<ObjectBuffer>& results) override;
```

1. 遍历所有输出层, 调用 `process_one` 解析每层特征图
2. 对所有检测结果执行 NMS

## 边界框解码公式

YOLOv5 使用 anchor-base 的解码方式:

```
x = (sigmoid(tx) * 2 - 0.5 + grid_x) * stride
y = (sigmoid(ty) * 2 - 0.5 + grid_y) * stride
w = (sigmoid(tw) * 2)^2 * anchor_w
h = (sigmoid(th) * 2)^2 * anchor_h
```
