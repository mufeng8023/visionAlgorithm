# TrackResult 跟踪结果结构体

<!-- vscode-markdown-toc -->
- [TrackResult 跟踪结果结构体](#trackresult-跟踪结果结构体)
  - [概述](#概述)
  - [结构体定义](#结构体定义)

<!-- vscode-markdown-toc -->

## 概述

`TrackResult` 是跟踪器输出结果的统一数据结构。跟踪结果通过 `std::vector<TrackResult>` 输出, 每条记录包含一个活跃轨迹的跟踪 ID 及其关联的原始检测框索引。

**文件**: `lib/tracker/TrackResult.hpp`
**命名空间**: `tracker`

## 结构体定义

```cpp
typedef struct {
    int32 track_id;      // 轨迹唯一 ID, 由跟踪器分配 (从 1 开始, 单调递增)
    int32 det_index;     // 检测框索引: >= 0 匹配到的检测框; -1 纯卡尔曼预测帧
    float32 score;       // 检测置信度
    int32 cls_id;        // 类别 ID
    float32 x1, y1;      // 边界框左上角 (原始检测框, 未扩展)
    float32 x2, y2;      // 边界框右下角 (原始检测框, 未扩展)
    float32 x, y, w, h;  // 边界框 center-wh 表示 (原始检测框, 未扩展)
} TrackResult;
```

各字段说明:

| 字段             | 说明                                                  |
| ---------------- | ----------------------------------------------------- |
| `track_id`       | 轨迹 ID, 单调递增, 保证每个轨迹具有唯一 ID            |
| `det_index`      | 本帧匹配到的检测框在输入数组中的索引; `-1` 表示无匹配 |
| `score`          | 当前轨迹的置信度分数                                  |
| `cls_id`         | 类别 ID                                               |
| `x1, y1, x2, y2` | 边界框的 xyxy 表示 (左上角 + 右下角, 原始框)          |
| `x, y, w, h`     | 边界框的 center-wh 表示 (中心点 + 宽高, 原始框)       |

> **注意**: TrackResult 中的坐标始终使用原始检测框 (ltwh), 而不是扩展框 (ltwh_expand)。外部用户应直接读取这些坐标, 无需额外转换。

构造辅助:

```cpp
// 从 BaseTrack 构建 TrackResult (自动从 ltwh 计算各坐标格式)
TrackResult make_track_result(const BaseTrack& track, int32 det_index);
