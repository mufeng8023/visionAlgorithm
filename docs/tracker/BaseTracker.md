# BaseTracker 跟踪器抽象基类

<!-- vscode-markdown-toc -->
- [BaseTracker 跟踪器抽象基类](#basetracker-跟踪器抽象基类)
  - [概述](#概述)
  - [类定义](#类定义)
  - [公共成员](#公共成员)
  - [核心接口](#核心接口)
    - [`update` (纯虚函数)](#update-纯虚函数)
    - [`reset`](#reset)
    - [`next_track_id`](#next_track_id)
  - [各跟踪器差异速览](#各跟踪器差异速览)

<!-- vscode-markdown-toc -->

## 概述

`BaseTracker` 是所有跟踪器实现的抽象基类, 定义了跟踪器的公共接口和公共成员。DeepSORT / ByteTrack / SORT / OC_SORT 等所有跟踪器均继承此类。

**文件**: `lib/tracker/BaseTracker.hpp`
**命名空间**: `tracker`

## 类定义

```cpp
class BaseTracker {
public:
    int32 _frame_id = 0;          // 当前帧 ID (每调用一次 update 自增)
    int32 _next_id = 1;           // 下一个轨迹 ID (每次创建新轨迹时自增)
    KFBox _kalman_filter;         // 卡尔曼滤波器实例 (所有轨迹共享)
    TrackerConfig _config;        // 跟踪器配置 (从 ini 文件读取)

public:
    explicit BaseTracker(const TrackerConfig& config);
    virtual ~BaseTracker() = default;

    virtual void update(const std::vector<BoxObject>& detections,
                        std::vector<TrackResult>& results,
                        int32 frame_id = -1) = 0;

    virtual void reset();
    inline int32 next_track_id();
    inline int32 frame_id() const;
    inline KFBox& kalman_filter();
    inline const TrackerConfig& config() const;
};
```

## 公共成员

| 成员             | 说明                                                            |
| ---------------- | --------------------------------------------------------------- |
| `_frame_id`      | 当前帧 ID, 每次调用 update 自增 1, 初始值为 0                   |
| `_next_id`       | 下一个轨迹 ID, 每次创建新轨迹时自增, 从 1 开始                  |
| `_kalman_filter` | KFBox 卡尔曼滤波器实例, 所有轨迹共享同一个对象                  |
| `_config`        | 跟踪器配置, 包含 track_thresh, high_thresh, match_thresh 等参数 |

> **注意**: DeepSORT 是"所有轨迹共享同一个 KFBox 对象"(指针传递); ByteTrack 是"每个轨迹持有自己的 KFBox 结果副本"。虽然用法不同, 但 KFBox 对象本身是无状态的, 两种方式都正确。

## 核心接口

### `update` (纯虚函数)

```cpp
virtual void update(const std::vector<BoxObject>& detections,
                    std::vector<TrackResult>& results,
                    int32 frame_id = -1) = 0;
```

跟踪器主入口, 每帧调用一次。输入当前帧的检测结果, 更新内部轨迹状态。跟踪结果通过 `results` 输出。

`det_index` 含义:

- `>= 0`: 本帧匹配到了 `detections[det_index]` 这个检测框
- `-1`: 本帧无匹配 (DeepSORT 已确认轨迹的纯卡尔曼预测帧)

### `reset`

```cpp
virtual void reset();
```

重置跟踪器状态, 清空所有轨迹, 重置帧计数器和轨迹 ID 计数器。通常在视频序列切换或跟踪异常恢复时调用。

### `next_track_id`

```cpp
inline int32 next_track_id();
```

分配并返回下一个轨迹 ID, 每次调用自增 `_next_id`, 保证全局唯一。

## 各跟踪器差异速览

| Tracker   | 匹配策略                                                       |
| --------- | -------------------------------------------------------------- |
| SORT      | 单次 IoU 关联 (卡尔曼预测 + 匈牙利匹配)                        |
| DeepSORT  | 级联匹配 (马氏距离 + 余弦距离) + 二次 IoU 关联                 |
| ByteTrack | 两次 IoU 关联 (高分检测优先, 低分检测兜底, 未确认轨迹单独处理) |
| OC_SORT   | ByteTrack 基础上增加观测置信度修正                             |
