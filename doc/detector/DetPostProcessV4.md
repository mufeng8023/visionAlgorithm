# DetPostProcessV4 YOLOv4 检测后处理

<!-- vscode-markdown-toc -->
- [DetPostProcessV4 YOLOv4 检测后处理](#detpostprocessv4-yolov4-检测后处理)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [`process_one` (单层特征图处理)](#process_one-单层特征图处理)
    - [`run` (入口)](#run-入口)
  - [边界框解码公式](#边界框解码公式)

<!-- vscode-markdown-toc -->

## 概述

`DetPostProcessV4` 是 `BasePostProcess` 的派生类, 专门处理 YOLOv4 系列 anchor-base 检测模型的输出解码。YOLOv4 与 YOLOv5 不同之处在于解码公式: YOLOv4 的 x/y 直接使用 sigmoid 后的 tx/ty 加网格坐标, w/h 使用 exp(tw) 乘以 anchor 宽高。

> **注意**: 导出 ONNX 模型时, 置信度分数和类别分数已经过 sigmoid 激活, w/h 已经过 exp 处理, 因此后处理中直接使用原始值。

## 类定义

**文件**: `lib/detector/DetPostProcessV4.hpp`  
**命名空间**: `yolo`

```cpp
class DetPostProcessV4 : public BasePostProcess {
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
    uint32 na;                    // 每个位置 anchor 数 (anchor-base 为 3)
    uint32 no;                    // 每个位置输出信息数
    uint32 nl;                    // 输出层数
    std::vector<uint32> strides;  // 各层步长
    std::vector<std::vector<float32>> anchors; // anchor 信息
    std::vector<uint32> net_out_h;  // 各输出层高度
    std::vector<uint32> net_out_w;  // 各输出层宽度
    std::vector<uint32> output_len; // 各输出层数据元素总数
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

YOLOv4 使用 anchor-base 的解码方式 (tx/ty/conf/cls 已 sigmoid, tw/th 已 exp):

```text
cx = (tx + grid_x) * stride
cy = (ty + grid_y) * stride
w  = exp(tw) * anchor_w
h  = exp(th) * anchor_h
score = conf * max(class_scores)
```

**与 YOLOv5 的关键区别对比**:

| 版本   | 中心点解码                           | 宽高解码                     |
| ------ | ------------------------------------ | ---------------------------- |
| YOLOv4 | cx = (sig(tx) + grid_x) * stride     | w = exp(tw) * anchor_w       |
| YOLOv5 | cx = (sig(tx)*2-0.5 + grid_x)*stride | w = (sig(tw)*2)^2 * anchor_w |

YOLOv4 的 anchor 配置 (相对于模型输入 384x640):

| 特征层 | stride | anchor 尺寸                     |
| ------ | ------ | ------------------------------- |
| layer0 | 8      | (12,16), (19,36), (40,28)       |
| layer1 | 16     | (36,75), (76,55), (72,146)      |
| layer2 | 32     | (142,110), (192,243), (459,401) |
