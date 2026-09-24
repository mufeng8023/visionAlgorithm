# matching 匹配算法 (ByteTrack)

<!-- vscode-markdown-toc -->
- [matching 匹配算法 (ByteTrack)](#matching-匹配算法-bytetrack)
  - [概述](#概述)
  - [核心函数](#核心函数)
    - [`linear_assignment` (线性分配)](#linear_assignment-线性分配)
    - [`cal_iou_distance` (IoU 距离计算)](#cal_iou_distance-iou-距离计算)
    - [`joint_stracks` (合并轨迹)](#joint_stracks-合并轨迹)
    - [`sub_stracks` (轨迹差集)](#sub_stracks-轨迹差集)
    - [`remove_duplicate_stracks` (移除重复轨迹)](#remove_duplicate_stracks-移除重复轨迹)

<!-- vscode-markdown-toc -->

## 概述

`matching.hpp` 提供了 ByteTrack 的匹配算法实现, 包括 IoU 距离计算和匈牙利匹配 (LAPJV), 以及轨迹列表管理工具函数。

**文件**: `lib/tracker/bytetrack/matching.hpp`
**命名空间**: `tracker::bytetrack`

## 核心函数

### `linear_assignment` (线性分配)

```cpp
ByteMatchResult linear_assignment(
    const std::vector<std::vector<float32>>& cost_matrix,
    float32 cost_thresh,
    int32 max_iters = 1000);
```

使用 LAPJV 算法求解线性分配问题。返回匹配结果 (`ByteMatchResult`), 包含 `matches` (匹配对), `unmatched_tracks` (未匹配轨迹), `unmatched_detections` (未匹配检测)。

`ByteMatchResult` 结构体:

```cpp
struct ByteMatchResult {
    std::vector<std::pair<int32, int32>> matches;          // 匹配对 (轨迹索引, 检测索引)
    std::vector<int32> unmatched_tracks;                   // 未匹配的轨迹索引
    std::vector<int32> unmatched_detections;               // 未匹配的检测索引
};
```

### `cal_iou_distance` (IoU 距离计算)

```cpp
std::vector<std::vector<float32>> cal_iou_distance(
    std::vector<BytetrackTrack*>& atracks,
    std::vector<BytetrackTrack>& btracks);
```

计算轨迹列表和检测列表之间的 IoU 距离矩阵。`IoU 距离 = 1 - IoU`。使用扩展框 (`ltwh_expand`) 计算 IoU。

### `joint_stracks` (合并轨迹)

```cpp
std::vector<BytetrackTrack> joint_stracks(
    std::vector<BytetrackTrack>& track_list_a,
    std::vector<BytetrackTrack>& track_list_b);
```

合并两个轨迹列表, 自动去重。例如将 `lost_stracks` 合并回 `tracked_stracks`。

### `sub_stracks` (轨迹差集)

```cpp
std::vector<BytetrackTrack> sub_stracks(
    std::vector<BytetrackTrack>& track_list_a,
    std::vector<BytetrackTrack>& track_list_b);
```

从 `track_list_a` 中减去 `track_list_b` 中的轨迹 (基于 track_id)。例如从 `lost_stracks` 中移除已经找回的轨迹。

### `remove_duplicate_stracks` (移除重复轨迹)

```cpp
void remove_duplicate_stracks(
    std::vector<BytetrackTrack>& resa,
    std::vector<BytetrackTrack>& resb,
    std::vector<BytetrackTrack>& tracks_a,
    std::vector<BytetrackTrack>& tracks_b);
```

基于 IoU 重叠检测重复轨迹。如果两个轨迹的 IoU > 0.85, 认为跟踪的是同一个目标, 保留年龄更大的轨迹。

**为什么会有重复轨迹?** 如果某个目标被检测器连续检测到, 但中途短暂丢失后又重新出现, 可能会产生两个 `track_id` 不同的轨迹实际上跟踪的是同一个目标。
