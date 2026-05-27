# YoloObject 检测结果数据结构

<!-- vscode-markdown-toc -->
- [YoloObject 检测结果数据结构](#yoloobject-检测结果数据结构)
  - [概述](#)
  - [结构体定义](#)

<!-- vscode-markdown-toc -->

## 概述

`YoloObject` 是检测结果的统一数据结构, 包含检测框、关键点、分割掩码等所有可能的信息。不同类型的任务使用不同的字段组合。

## 结构体定义

**文件**: `lib/detector/YoloObject.h`  
**命名空间**: `yolo`

### `Box` (边界框)

```cpp
typedef struct {
    uint32 cls_id;    // 类别 ID
    float32 score;    // 置信度分数
    float32 x1;       // 左上角 x 坐标
    float32 y1;       // 左上角 y 坐标
    float32 x2;       // 右下角 x 坐标
    float32 y2;       // 右下角 y 坐标
} Box;
```

### `KeyPoint` (关键点)

```cpp
typedef struct {
    float32 x;        // x 坐标
    float32 y;        // y 坐标
    float32 score;    // 置信度分数
} KeyPoint;
```

### `YoloObject` (检测结果)

```cpp
typedef struct {
    TaskType type = TaskType::detection;  // 任务类型
    Box box;                              // 检测框 (所有任务共有)
    std::vector<KeyPoint> kpts;           // 关键点 (Pose 任务)
    std::vector<uint8> mask;              // 分割掩码 (Seg 任务)
    float32 angle = 0.0f;                 // 旋转角度 (OBB 任务)
    uint32 mask_width = 0;                // 分割掩码宽度
    uint32 mask_height = 0;               // 分割掩码高度
} YoloObject;
```

各任务类型使用的字段:

| 任务类型     | 使用的字段                                 |
| ------------ | ------------------------------------------ |
| detection    | `box`                                      |
| pose         | `box`, `kpts`                              |
| segmentation | `box`, `mask`, `mask_width`, `mask_height` |
| obb          | `box`, `angle`                             |
