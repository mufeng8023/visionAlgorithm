# TrackerConfig 跟踪器配置

<!-- vscode-markdown-toc -->
- [TrackerConfig 跟踪器配置](#trackerconfig-跟踪器配置)
  - [概述](#概述)
  - [枚举定义](#枚举定义)
    - [`TrackerType` (跟踪器类型)](#trackertype-跟踪器类型)
    - [`TrackerType` 字符串转换函数](#trackertype-字符串转换函数)
  - [配置结构体](#配置结构体)

<!-- vscode-markdown-toc -->

## 概述

`TrackerConfig.hpp` 定义了项目中跟踪器的枚举类型和配置数据结构。配置信息通过 INI 配置文件加载, 驱动整个跟踪流程的参数设置。

**文件**: `lib/tracker/TrackerConfig.hpp`
**命名空间**: `tracker`

## 枚举定义

### `TrackerType` (跟踪器类型)

```cpp
enum class TrackerType : uint8 {
    bytetrack = 0,  // ByteTrack (推荐, 无需 ReID 模型)
    deepsort,       // DeepSORT (需要 ReID 特征)
    sort,           // SORT (占位, 暂未实现)
    ocsort,         // OC_SORT (占位, 暂未实现)
    count
};
```

### `TrackerType` 字符串转换函数

| 函数                                           | 说明                    |
| ---------------------------------------------- | ----------------------- |
| `tracker_type_to_string(TrackerType)`          | 枚举转字符串            |
| `tracker_type_from_string(const std::string&)` | 字符串转枚举 (小写匹配) |

## 配置结构体

```cpp
typedef struct {
    TrackerType tracker_type = TrackerType::bytetrack;   // 跟踪器类型

    // 基础参数
    uint32 frame_rate = 30;               // 视频帧率
    uint32 track_buffer = 30;             // 轨迹保留帧缓冲数
    float32 track_thresh = 0.5f;          // 检测框置信度阈值
    float32 high_thresh = 0.6f;           // 高分检测框阈值
    float32 match_thresh = 0.8f;          // 关联匹配阈值
    int32 max_age = 30;                   // 轨迹最大丢失帧数
    int32 n_init = 3;                     // 新轨迹确认所需初始帧数
    float32 max_iou_distance = 0.7f;      // IoU 匹配阈值

    // 边界框扩展参数
    float32 expand_box_rate = 0.0f;       // 边界框扩展率

    // ByteTrack 专用参数
    float32 match_thresh_low = 0.5f;      // 第二次关联阈值 (低分检测)
    float32 match_thresh_unconfirmed = 0.7f; // 第三次关联阈值 (未确认轨迹)

    // DeepSORT 专用参数
    bool use_reid = false;                // 是否使用 ReID 特征
    uint32 reid_feature_dim = 0;          // ReID 特征维度
    float32 max_cosine_distance = 0.2f;   // 余弦距离阈值
    float32 lambda_cosine_weight = 0.98f; // 余弦距离融合权重
} TrackerConfig;
```

各参数说明:

| 参数                       | 默认值    | 说明                                                                              |
| -------------------------- | --------- | --------------------------------------------------------------------------------- |
| `tracker_type`             | bytetrack | 跟踪器类型, 支持 bytetrack / deepsort                                             |
| `frame_rate`               | 30        | 视频帧率, 用于时间相关计算                                                        |
| `track_buffer`             | 30        | 轨迹保留帧缓冲数                                                                  |
| `track_thresh`             | 0.5       | 检测框置信度阈值, 低于此值的检测框将被过滤                                        |
| `high_thresh`              | 0.6       | 高分检测框阈值, ByteTrack 用于高低分框分流                                        |
| `match_thresh`             | 0.8       | 关联匹配阈值, IoU 或余弦距离的匹配阈值                                            |
| `max_age`                  | 30        | 轨迹最大丢失帧数, 超过此值后轨迹被移除                                            |
| `n_init`                   | 3         | 新轨迹确认所需初始帧数, 连续 n_init 帧匹配成功才确认为正式轨迹                    |
| `max_iou_distance`         | 0.7       | IoU 匹配阈值, 高于此值的框被认为匹配                                              |
| `expand_box_rate`          | 0.0       | 边界框扩展率, 对小目标跟踪有帮助                                                  |
| `match_thresh_low`         | 0.5       | ByteTrack 第二次关联阈值 (低分检测 vs 未匹配 Tracked 轨迹)                        |
| `match_thresh_unconfirmed` | 0.7       | ByteTrack 第三次关联阈值 (未确认轨迹 vs 剩余高分检测)                             |
| `use_reid`                 | false     | DeepSORT 是否使用 ReID 特征                                                       |
| `reid_feature_dim`         | 0         | DeepSORT ReID 特征维度                                                            |
| `max_cosine_distance`      | 0.2       | DeepSORT 余弦距离阈值                                                             |
| `lambda_cosine_weight`     | 0.98      | DeepSORT 余弦距离融合权重, 公式: combined = (1-lambda)*马氏距离 + lambda*余弦距离 |
