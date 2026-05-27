# BasePostProcess 后处理基类

<!-- vscode-markdown-toc -->
- [BasePostProcess 后处理基类](#basepostprocess-后处理基类)
  - [概述](#概述)
  - [类定义](#类定义)
  - [NMS 非极大值抑制](#nms-非极大值抑制)
    - [`nms_ops` (单图 NMS)](#nms_ops-单图-nms)
    - [`non_max_suppression` (批量 NMS)](#non_max_suppression-批量-nms)
  - [派生类](#派生类)

<!-- vscode-markdown-toc -->

## 概述

`BasePostProcess` 是所有后处理实现的抽象基类, 定义了将网络输出特征图解析为检测结果的标准接口。同时在该文件中提供了 NMS (非极大值抑制) 的实现。

## 类定义

**文件**: `lib/detector/BasePostProcess.hpp`  
**命名空间**: `yolo`

```cpp
class BasePostProcess {
public:
    BasePostProcess() = default;
    virtual ~BasePostProcess() = default;

    // 后处理入口
    virtual void run(const std::vector<NetOutput>& outputs,
                     std::vector<ObjectBuffer>& results) = 0;

    virtual std::string to_string() const;
};
```

## NMS 非极大值抑制

### `nms_ops` (单图 NMS)

```cpp
static inline void nms_ops(ObjectBuffer& output,
                           float32 iou_thr = 0.45,
                           bool agnostic = false);
```

对单张图片的检测结果执行 NMS:

1. 按置信度降序排列所有检测框
2. 遍历每个框, 计算与后续框的 IoU
3. IoU 超过阈值的框标记为无效
4. 调用 `ObjectBuffer::compact()` 压缩缓冲区

### `non_max_suppression` (批量 NMS)

```cpp
void non_max_suppression(std::vector<ObjectBuffer>& outputs,
                         float32 iou_thr = 0.45,
                         bool agnostic = false);
```

对 batch 中每张图片的结果分别调用 `nms_ops`。

参数说明:

- `iou_thr`: IoU 阈值, 默认 0.45
- `agnostic`: 是否进行类别不敏感的 NMS
  - `false` (默认): 不同类别之间不会互相抑制
  - `true`: 所有类别统一进行 NMS

## 派生类

| 类名                                      | 说明                          |
| ----------------------------------------- | ----------------------------- |
| [V5DetPostProcess](V5DetPostProcess.md)   | YOLOv5 anchor-base 检测后处理 |
| [V8DetPostProcess](V8DetPostProcess.md)   | YOLOv8 anchor-free 检测后处理 |
| [V5PosePostProcess](V5PosePostProcess.md) | YOLOv5 姿态估计后处理         |
| [V8PosePostProcess](V8PosePostProcess.md) | YOLOv8 姿态估计后处理         |
