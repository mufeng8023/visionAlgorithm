# TrackHistory 轨迹历史记录

<!-- vscode-markdown-toc -->
- [TrackHistory 轨迹历史记录](#trackhistory-轨迹历史记录)
  - [概述](#概述)
  - [结构体定义](#结构体定义)
    - [`TrackHistoryFrame` (单帧记录)](#trackhistoryframe-单帧记录)
    - [`TrackHistory` (完整轨迹历史)](#trackhistory-完整轨迹历史)
  - [核心接口](#核心接口)
  - [应用场景](#应用场景)

<!-- vscode-markdown-toc -->

## 概述

`TrackHistory` 为上层应用 (客流量统计, 行为分析, 轨迹回溯等) 提供跨帧目标关联数据。`TrackerRuntime` 在每帧更新后将结果填充到此结构中, 应用层可直接读取实现各类高级分析。

**核心数据流**: `YoloObject → TrackerMiddleware 转换 + 跟踪 → TrackResult (单帧) → TrackHistory (多帧累积)`

**文件**: `lib/tracker/TrackHistory.hpp`
**命名空间**: `tracker`

## 结构体定义

### `TrackHistoryFrame` (单帧记录)

```cpp
struct TrackHistoryFrame {
    int32 frame_id = 0;        // 帧号
    int32 det_index = -1;      // 输入检测索引 (-1 表示纯预测)
    int32 cls_id = -1;         // 类别 ID
    float32 score = 0.0f;      // 置信度
    float32 ltwh[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // 原始检测框
};
```

### `TrackHistory` (完整轨迹历史)

```cpp
struct TrackHistory {
    int32 track_id = -1;                   // 轨迹唯一 ID
    int32 cls_id = -1;                     // 最新类别 ID
    bool is_active = false;                // 当前帧是否仍然活跃
    std::deque<TrackHistoryFrame> frames;  // 历史帧记录 (滚动窗口)
    yolo::YoloObject last_detection;       // 最近一次匹配的完整检测结果
};
```

## 核心接口

| 方法                                 | 说明                                 |
| ------------------------------------ | ------------------------------------ |
| `TrackHistory::add_frame(frame)`     | 追加一帧记录到 frames 队列, 自动滚窗 |
| `TrackHistory::get_latest_frame()`   | 获取最新的一帧记录                   |
| `TrackHistory::get_history_length()` | 获取当前历史帧数                     |
| `TrackHistory::clear()`              | 清空历史记录                         |

历史帧以 `deque` 形式存储, 超出 `max_history_frames` 时自动滚动删除最旧帧。

## 应用场景

- **客流量统计**: 判断轨迹是否穿过统计线 (用 frames 中的位置序列判断)
- **行为分析**: 对 frames 中的位置序列做速度/加速度/方向分析
- **轨迹回溯**: 可视化某段时间内目标的移动路径
- **轨迹可视化**: 绘制每个目标的历史运动路径
