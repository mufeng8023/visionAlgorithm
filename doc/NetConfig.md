# NetConfig 模型配置

<!-- vscode-markdown-toc -->
- [NetConfig 模型配置](#netconfig-模型配置)
  - [概述](#概述)
  - [枚举定义](#枚举定义)
    - [`ModelType` (模型类型)](#modeltype-模型类型)
    - [`TaskType` (任务类型)](#tasktype-任务类型)
  - [配置结构体](#配置结构体)
    - [`DetectionNetConfig`](#detectionnetconfig)
  - [工具函数](#工具函数)

<!-- vscode-markdown-toc -->

## 概述

`NetConfig.hpp` 定义了项目中使用的所有枚举类型和模型配置数据结构。配置信息通过 INI 配置文件加载, 驱动整个推理流程的参数设置。

## 枚举定义

### `ModelType` (模型类型)

```cpp
enum class ModelType : uint8 {
    yolov3 = 0,
    yolov4,
    yolov5,   // anchor-base
    yolov6,
    yolov7,
    yolov5u,  // anchor-free (ultralytics)
    yolov8,
    yolov9,
    yolov10,
    yolo11,
    yolo12,
    yolo26,
    count
};
```

### `TaskType` (任务类型)

```cpp
enum class TaskType : uint8 {
    classification = 0,
    detection,
    segmentation,
    pose,
    obb,
    count
};
```

## 配置结构体

### `DetectionNetConfig`

```cpp
typedef struct {
    std::string model_name;                 // 模型名称
    ModelType model_type;                   // 模型类型
    TaskType task;                          // 任务类型
    bool has_conf;                          // 是否有置信度通道
    std::vector<std::string> names;         // 类别名称列表
    int32 nc;                               // 类别数量
    std::vector<float32> scale_outputs;     // 反量化系数
    std::vector<float32> conf_thrs;         // 各类别置信度阈值
    float32 min_conf;                       // 最小置信度阈值
    float32 iou_thrs;                       // NMS IoU 阈值
    uint32 max_det;                         // 最大检测目标数
    bool agnostic;                          // 是否类别不敏感 NMS
    uint32 batch_size;                      // Batch 大小
    uint32 kpt_count;                       // 关键点个数
    uint32 kpt_dim;                         // 关键点维度
    float32 kpt_conf_thr;                   // 关键点置信度阈值
    uint32 input_width;                     // 输入宽度
    uint32 input_height;                    // 输入高度
    uint32 input_channels;                  // 输入通道数
    std::vector<uint32> strides;            // 各输出层步长
    std::vector<std::vector<float32>> anchors; // anchor 信息
    uint32 na;                              // 每个位置 anchor 数
    uint32 no;                              // 每个位置输出信息数
    uint32 nl;                              // 输出层数
    std::vector<uint32> net_out_h;          // 各输出层高度
    std::vector<uint32> net_out_w;          // 各输出层宽度
} DetectionNetConfig;
```

## 工具函数

| 函数                                  | 说明                |
| ------------------------------------- | ------------------- |
| `model_type_to_string(ModelType)`     | 枚举转字符串 (O(1)) |
| `model_type_from_string(string_view)` | 字符串转枚举        |
| `task_type_to_string(TaskType)`       | 枚举转字符串 (O(1)) |
| `task_type_from_string(string_view)`  | 字符串转枚举        |
