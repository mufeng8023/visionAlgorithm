# BasePostProcess 后处理基类

<!-- vscode-markdown-toc -->
- [BasePostProcess 后处理基类](#basepostprocess-后处理基类)
  - [概述](#概述)
  - [类定义](#类定义)
  - [NMS 非极大值抑制](#nms-非极大值抑制)
    - [`nms_ops` (单图 NMS)](#nms_ops-单图-nms)
    - [`end2end_post` (端到端后处理)](#end2end_post-端到端后处理)
    - [`non_max_suppression` (批量 NMS)](#non_max_suppression-批量-nms)
  - [派生类](#派生类)

<!-- vscode-markdown-toc -->

## 概述

`BasePostProcess` 是所有后处理实现的抽象基类, 定义了将网络输出特征图解析为检测结果的标准接口。同时在该文件中提供了 NMS (非极大值抑制) 和端到端后处理的实现。

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
                           float32 iou_thr,
                           bool agnostic,
                           std::vector<size_t>& indices_buffer);
```

对单张图片的检测结果执行 NMS:

1. 按置信度降序排列所有检测框
2. 遍历每个框, 计算与后续框的 IoU
3. IoU 超过阈值的框标记为无效
4. 调用 `ObjectBuffer::compact()` 压缩缓冲区

### `end2end_post` (端到端后处理)

```cpp
static inline void end2end_post(ObjectBuffer& output,
                                size_t max_det,
                                std::vector<size_t>& indices_buffer);
```

用于 YOLOv10 / YOLOv26 等端到端模型的输出后处理:

1. 确保所有目标均为有效状态
2. 按置信度降序排列
3. 保留前 `max_det` 个目标, 其余标记为无效
4. 调用 `ObjectBuffer::compact()` 压缩缓冲区

> 端到端模型在导出时已内置 NMS, 因此后处理仅需按分数排序并截取前 `max_det` 个结果。

### `non_max_suppression` (批量 NMS)

```cpp
void non_max_suppression(std::vector<ObjectBuffer>& outputs,
                         float32 iou_thr,
                         bool agnostic,
                         size_t max_det,
                         bool end2end = false);
```

对 batch 中每张图片的结果分别处理:

- `end2end = false` (默认): 调用 `nms_ops` 执行标准 NMS
- `end2end = true`: 调用 `end2end_post` 执行端到端后处理

参数说明:

| 参数       | 说明                                                      |
| ---------- | --------------------------------------------------------- |
| `iou_thr`  | IoU 阈值, 默认 0.45                                       |
| `agnostic` | 是否进行类别不敏感的 NMS                                  |
| `max_det`  | 每张图片最大检测目标数, 默认 300                          |
| `end2end`  | 是否启用端到端模式, `false` 执行 NMS, `true` 执行排序截取 |

## 派生类

| 类名                                      | 说明                              |
| ----------------------------------------- | --------------------------------- |
| [DetPostProcessV4](DetPostProcessV4.md)   | YOLOv4 anchor-base 检测后处理     |
| [DetPostProcessV5](DetPostProcessV5.md)   | YOLOv5 anchor-base 检测后处理     |
| [DetPostProcessV8](DetPostProcessV8.md)   | YOLOv8 anchor-free 检测后处理     |
| [DetPostProcess26](DetPostProcess26.md)   | YOLOv26 端到端检测后处理          |
| [PosePostProcessV5](PosePostProcessV5.md) | YOLOv5 姿态估计后处理             |
| [PosePostProcessV8](PosePostProcessV8.md) | YOLOv8 anchor-free 姿态估计后处理 |
| [PosePostProcess26](PosePostProcess26.md) | YOLOv26 端到端姿态估计后处理      |
