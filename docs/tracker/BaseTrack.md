# BaseTrack 轨迹基类

<!-- vscode-markdown-toc -->
- [BaseTrack 轨迹基类](#basetrack-轨迹基类)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心成员](#核心成员)
    - [边界框存储 (ltwh / ltwh\_expand)](#边界框存储-ltwh--ltwh_expand)
    - [卡尔曼状态 (mean / covariance)](#卡尔曼状态-mean--covariance)
  - [核心方法](#核心方法)
    - [`activate` (激活轨迹)](#activate-激活轨迹)
    - [`re_activate` (重新激活轨迹)](#re_activate-重新激活轨迹)
    - [`predict` (卡尔曼预测)](#predict-卡尔曼预测)
    - [`update` (卡尔曼更新)](#update-卡尔曼更新)
    - [`update_ltwh_from_mean` (从卡尔曼状态更新扩展框)](#update_ltwh_from_mean-从卡尔曼状态更新扩展框)
    - [`get_xyah` (获取 xyah 表示)](#get_xyah-获取-xyah-表示)
    - [`get_xyxy` (获取 xyxy 表示)](#get_xyxy-获取-xyxy-表示)
  - [边界框扩展机制](#边界框扩展机制)

<!-- vscode-markdown-toc -->

## 概述

`BaseTrack` 是所有跟踪算法轨迹的公共基类, 定义了轨迹的基本数据结构和核心方法。所有跟踪器 (ByteTrack / DeepSORT) 的轨迹类均继承自此类。

**核心设计思想**: 边界框存储采用 `ltwh` (原始框) + `ltwh_expand` (扩展框) 的双轨策略, 确保卡尔曼操作在统一的扩展空间中进行, 同时保留原始检测框供外部使用。

**文件**: `lib/tracker/BaseTrack.hpp`
**命名空间**: `tracker`

## 类定义

```cpp
class BaseTrack {
public:
    // 轨迹标识
    int32 track_id = -1;         // 轨迹唯一 ID (由跟踪器分配, 从 1 开始)
    int32 frame_id = 0;          // 当前帧 ID
    int32 start_frame = 0;       // 轨迹开始的帧号
    int32 age = 1;               // 轨迹已存在的总帧数

    // 检测信息
    float32 score = 0.0f;        // 检测置信度分数
    int32 cls_id = 0;            // 类别 ID
    float32 cls_score = 0.0f;    // 类别分数

    // 边界框 (双轨存储)
    float32 ltwh[4] = {0.0f, 0.0f, 0.0f, 0.0f};         // 原始检测框
    float32 ltwh_expand[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 扩展后的检测框

    // 卡尔曼滤波器状态
    BOX_MEAN mean;               // 状态均值 (8 维向量)
    BOX_COVA covariance;         // 状态协方差 (8x8 矩阵)

protected:
    float32 _expand_box_rate = 0.0f;  // 边界框扩展率
    bool _is_activated = false;       // 是否完成卡尔曼初始化

public:
    BaseTrack(int32 track_id, int32 frame_id, float32 expand_box_rate = 0.0f);
    virtual ~BaseTrack() = default;

    // 纯虚函数
    virtual void mark_lost() = 0;
    virtual void mark_removed() = 0;

    // 核心方法
    void activate(KFBox& kalman_filter, int32 frame_id, int32 track_id);
    void re_activate(KFBox& kalman_filter, const BaseTrack& new_track, bool new_det = false);
    void predict(KFBox& kalman_filter);
    void update(KFBox& kalman_filter, const BaseTrack& new_track);

    // 坐标转换
    std::array<float32, 4> get_xyah() const;
    std::array<float32, 4> get_xyxy() const;
    float32 get_x1() const;
    float32 get_y1() const;
    float32 get_x2() const;
    float32 get_y2() const;

    // 状态查询
    bool is_activated() const;
    float32 get_expand_box_rate() const;
    void set_expand_box_rate(float32 rate);
};
```

## 核心成员

### 边界框存储 (ltwh / ltwh_expand)

| 成员             | 格式    | 说明                                       |
| ---------------- | ------- | ------------------------------------------ |
| `ltwh[4]`        | L,T,W,H | 原始检测框, 由检测器直接输出, 永不修改     |
| `ltwh_expand[4]` | L,T,W,H | 扩展后的检测框, 所有卡尔曼操作在此空间进行 |

### 卡尔曼状态 (mean / covariance)

- `mean`: 8 维状态均值 `[cx, cy, a, h, v_cx, v_cy, v_a, v_h]` (扩展空间)
- `covariance`: 8x8 状态协方差矩阵

> **重要**: mean 中的位置分量 (cx, cy, a, h) 存储的是扩展空间中的状态。ltwh_expand 与 mean 在同一个扩展空间中, 从 mean 转换到 ltwh_expand 时直接写入, 无需再次扩展或收缩。

## 核心方法

### `activate` (激活轨迹)

```cpp
void activate(KFBox& kalman_filter, int32 frame_id, int32 track_id);
```

使用第一帧检测结果初始化轨迹的卡尔曼状态:

1. 调用 `kalman_filter.initiate(measurement)` 获取初始均值和协方差
2. 设置 `_is_activated = true`

### `re_activate` (重新激活轨迹)

```cpp
void re_activate(KFBox& kalman_filter, const BaseTrack& new_track, bool new_det = false);
```

将丢失的轨迹与新的检测重新关联 (例如 ByteTrack 中 Lost 状态轨迹重新匹配成功时调用)。`new_det = true` 时重置 `start_frame`。

### `predict` (卡尔曼预测)

```cpp
void predict(KFBox& kalman_filter);
```

每帧调用, 利用卡尔曼滤波器预测轨迹在当前帧的新位置。内部调用 `kalman_filter.predict(mean, covariance)`, 然后通过 `update_ltwh_from_mean()` 更新 ltwh_expand。

### `update` (卡尔曼更新)

```cpp
void update(KFBox& kalman_filter, const BaseTrack& new_track);
```

检测框匹配成功后, 使用新的检测值修正卡尔曼状态:

1. 获取新检测的 xyah 测量值
2. 调用 `kalman_filter.update(mean, covariance, measurement)`
3. 更新 score / cls_id / cls_score 等信息
4. 调用 `update_ltwh_from_mean()` 写回 ltwh_expand

### `update_ltwh_from_mean` (从卡尔曼状态更新扩展框)

```cpp
void update_ltwh_from_mean();
```

从卡尔曼 mean 更新 ltwh_expand (扩展框):

```
cx = mean[0], cy = mean[1], a = mean[2], h = mean[3]
w = a * h
ltwh_expand = [cx - w/2, cy - h/2, w, h]
```

注意: 原始 ltwh 在此过程中不会被修改。

### `get_xyah` (获取 xyah 表示)

```cpp
std::array<float32, 4> get_xyah() const;
```

将 ltwh_expand 转换为 `[cx, cy, a, h]` 格式, 用于卡尔曼滤波器操作。

### `get_xyxy` (获取 xyxy 表示)

```cpp
std::array<float32, 4> get_xyxy() const;
```

将 ltwh_expand 转换为 `[x1, y1, x2, y2]` 格式。

## 边界框扩展机制

小目标 (如远处的人, 几十像素大小) 在检测器中边界框定位精度有限。通过将边界框向外扩展一定比例, 可以给卡尔曼滤波器更宽松的搜索空间, 提高小目标跟踪的鲁棒性。

| 扩展率取值 | 效果                            |
| ---------- | ------------------------------- |
| `0.0`      | 不扩展 (默认)                   |
| `0.1`      | 各方向扩展 10% (宽高各增加 10%) |
| `0.2`      | 各方向扩展 20%                  |

扩展在 `BoxObject` 创建时执行, 结果存储在 `ltwh_expand` 中。跟踪过程中全程使用 `ltwh_expand`, 不再需要 expand 或 shrink 操作。
