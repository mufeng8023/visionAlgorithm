# BytetrackTrack ByteTrack 轨迹

<!-- vscode-markdown-toc -->
- [BytetrackTrack ByteTrack 轨迹](#bytetracktrack-bytetrack-轨迹)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心方法](#核心方法)
  - [与原版 ByteTrack 的关键差异](#与原版-bytetrack-的关键差异)

<!-- vscode-markdown-toc -->

## 概述

`BytetrackTrack` 是 ByteTrack 算法专用的轨迹类, 继承自 `BaseTrack<ByteTrackState>`。每个轨迹持有自己的 `KFBox` 卡尔曼滤波器副本, 实现轨迹级别的状态管理。

**文件**: `lib/tracker/bytetrack/BytetrackTrack.hpp`
**命名空间**: `tracker::bytetrack`

## 类定义

```cpp
class BytetrackTrack : public BaseTrack<ByteTrackState> {
private:
    KFBox _kalman_filter;  // 每个轨迹持有自己的 KFBox 副本

public:
    // 构造函数
    BytetrackTrack(const std::vector<float32>& ltwh,
                   int32 frame_id, int32 track_id,
                   int32 cls_id = -1, float32 score = 0.0f,
                   float32 expand_box_rate = 0.0f);  // 从 ltwh 创建
    BytetrackTrack(const BoxObject& box,
                   int32 frame_id, int32 track_id,
                   float32 expand_box_rate = 0.0f);  // 从 BoxObject 创建
    BytetrackTrack();  // 默认构造 (用于容器)

    // 核心方法
    void update_ltwh() override;
    void activate(KFBox& kalman_filter, int32 frame_id, int32 track_id);
    void re_activate(KFBox& kalman_filter, const BaseTrack& new_track, bool new_det = false);
    void predict(KFBox& kalman_filter);
    void update(KFBox& kalman_filter, const BaseTrack& new_track);
    void mark_lost() override;
    void mark_removed() override;
};
```

## 核心方法

| 方法             | 说明                                                   |
| ---------------- | ------------------------------------------------------ |
| `update_ltwh()`  | 从卡尔曼 mean 更新 ltwh_expand; New 状态保持原始框不变 |
| `activate()`     | 激活轨迹, 使用扩展后的 xyah 初始化卡尔曼滤波器         |
| `re_activate()`  | 重新激活丢失的轨迹                                     |
| `predict()`      | 卡尔曼预测, 更新 ltwh_expand                           |
| `update()`       | 使用新检测值修正卡尔曼状态                             |
| `mark_lost()`    | 标记轨迹为 Lost 状态                                   |
| `mark_removed()` | 标记轨迹为 Removed 状态                                |

## 与原版 ByteTrack 的关键差异

| 对比项     | 原版 STrack                                 | 本实现                                   |
| ---------- | ------------------------------------------- | ---------------------------------------- |
| 边界框存储 | 三种框: raw_tlwh / _tlwh(扩展) / tlwh(输出) | 两种框: ltwh (原始) + ltwh_expand (扩展) |
| 坐标转换   | 独立函数 `tlwh_to_xyah`                     | 基类 `get_xyah()` 统一完成               |
| 重新激活   | 使用未扩展框                                | 统一使用 ltwh_expand                     |
