# matching 匹配算法 (DeepSORT)

<!-- vscode-markdown-toc -->
- [matching 匹配算法 (DeepSORT)](#matching-匹配算法-deepsort)
  - [概述](#概述)
  - [匹配流程概览](#匹配流程概览)
  - [核心函数](#核心函数)
    - [`cal_cosine_distance` (余弦距离矩阵)](#cal_cosine_distance-余弦距离矩阵)
    - [`cal_gating_distance` (马氏距离矩阵)](#cal_gating_distance-马氏距离矩阵)
    - [`cal_combined_distance` (融合距离矩阵)](#cal_combined_distance-融合距离矩阵)
    - [`gate_cost_matrix` (门控过滤)](#gate_cost_matrix-门控过滤)
    - [`min_cost_matching` (最小成本匹配)](#min_cost_matching-最小成本匹配)
    - [`matching_cascade` (级联匹配)](#matching_cascade-级联匹配)
    - [`iou_match` (IoU 匹配)](#iou_match-iou-匹配)

<!-- vscode-markdown-toc -->

## 概述

`matching.hpp` 提供了 DeepSORT 的匹配算法实现, 包括余弦距离矩阵计算、马氏距离门控、级联匹配 (Cascade Matching) 和 IoU 匹配。参考 deepsort 原版 `linear_assignment.py` 实现。

**文件**: `lib/tracker/deepsort/matching.hpp`
**命名空间**: `tracker::deepsort`

## 匹配流程概览

DeepSORT 的数据关联分两步:

**Step 1 - 级联匹配 (Cascade Matching)**:

- 优先匹配"最近更新"的轨迹 (time_since_update 小的优先)
- 距离度量 = `(1 - lambda) * 马氏距离 + lambda * 余弦距离`
- 门控: 马氏距离超过卡方阈值 (9.4877) 的匹配被拒绝; 余弦距离超过 `max_cosine_distance` 的匹配被拒绝

**Step 2 - IoU 匹配 (IoU Matching)**:

- 级联匹配中未匹配的轨迹 x 未匹配的检测
- 仅对 `time_since_update == 1` 的轨迹计算 IoU
- 距离度量 = `1 - IoU` (使用扩展框)

**为什么用级联匹配而不是一次性关联?**

- 如果轨迹 A 刚更新过 (time_since_update=1), 卡尔曼预测很准
- 如果轨迹 B 已经丢失了 10 帧 (time_since_update=10), 预测很不准
- 级联匹配让 A 先挑检测框, B 只能挑 A 不要的框
- 这样避免了 B 的"大光环"覆盖掉 A 的精确匹配

## 核心函数

### `cal_cosine_distance` (余弦距离矩阵)

```cpp
vector2D<float32> cal_cosine_distance(
    const vector2D<float32>& features,
    const vector2D<float32>& track_features);
```

计算 ReID 外观特征的余弦距离矩阵。余弦距离 = 1 - 余弦相似度。当特征向量为空时返回零矩阵 (不使用余弦距离)。

### `cal_gating_distance` (马氏距离矩阵)

```cpp
vector2D<float32> cal_gating_distance(
    KFBox* kf,
    std::vector<DeepSortTrack*>& tracks,
    const std::vector<BoxObject>& detections,
    const std::vector<int32>& track_indices,
    const std::vector<int32>& detection_indices);
```

使用卡尔曼滤波器的 `gating_distance()` 方法计算马氏距离矩阵。马氏距离是"归一化后的欧氏距离", 考虑了各维度的方差和相关性。

### `cal_combined_distance` (融合距离矩阵)

```cpp
vector2D<float32> cal_combined_distance(
    const vector2D<float32>& mahala_dist,
    const vector2D<float32>& cosine_dist,
    float32 lambda = 0.98f);
```

计算融合距离矩阵 (马氏距离 + 余弦距离)。公式: `combined = (1 - lambda) * mahala + lambda * cosine`。当未配置 ReID 时, cosine_dist 是全零矩阵, 退化为马氏距离匹配。

### `gate_cost_matrix` (门控过滤)

```cpp
void gate_cost_matrix(
    KFBox* kf,
    vector2D<float32>& cost_matrix,
    std::vector<DeepSortTrack*>& tracks,
    const std::vector<BoxObject>& detections,
    const std::vector<int32>& track_indices,
    const std::vector<int32>& detection_indices,
    float32 gated_cost = INFTY_COST_DEEP);
```

应用门控过滤到成本矩阵。马氏距离超过卡方 95% 置信区间阈值 (4 自由度, 9.4877) 的匹配被标记为无穷大成本。门控阈值的含义: 如果马氏距离平方 > 9.4877, 该检测与轨迹属于同一个目标的概率 < 5%。

### `min_cost_matching` (最小成本匹配)

```cpp
MatchResult min_cost_matching(
    const vector2D<float32>& cost_matrix,
    float32 max_distance,
    const std::vector<int32>& track_indices,
    const std::vector<int32>& detection_indices);
```

使用匈牙利算法求解线性分配问题。检查每个匹配对的成本是否 <= max_distance, 超过阈值的视为未匹配。

### `matching_cascade` (级联匹配)

```cpp
MatchResult matching_cascade(
    KFBox* kf,
    NNMetric<float32>& metric,
    const TrackerConfig& config,
    std::vector<DeepSortTrack*>& tracks,
    const std::vector<BoxObject>& detections,
    const std::vector<int32>& track_indices,
    const std::vector<int32>& detection_indices);
```

DeepSORT 核心的级联匹配算法。按 `time_since_update` 升序排序轨迹, 年龄小的优先匹配。每层使用融合距离 (马氏 + 余弦) + 门控过滤 + 匈牙利算法求解。

### `iou_match` (IoU 匹配)

```cpp
MatchResult iou_match(
    const std::vector<DeepSortTrack*>& tracks,
    const std::vector<BoxObject>& detections,
    const std::vector<int32>& track_indices,
    const std::vector<int32>& detection_indices);
```

级联匹配后的二次匹配。仅对 `time_since_update == 1` 的轨迹计算 IoU。距离度量 = 1 - IoU。
