# DeepSortTrack DeepSORT 轨迹

<!-- vscode-markdown-toc -->
- [DeepSortTrack DeepSORT 轨迹](#deepsorttrack-deepsort-轨迹)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心成员](#核心成员)
  - [与 ByteTrack 轨迹的关键差异](#与-bytetrack-轨迹的关键差异)

<!-- vscode-markdown-toc -->

## 概述

`DeepSortTrack` 是 DeepSORT 算法专用的轨迹类, 继承自 `BaseTrack<DeepSortState>`。与 ByteTrack 不同, DeepSORT 轨迹不持有卡尔曼滤波器对象, 而是通过指针引用共享的 `KFBox` 实例。同时具有 `hits`, `age`, `time_since_update` 等统计字段用于级联匹配。

**文件**: `lib/tracker/deepsort/DeepSortTrack.hpp`
**命名空间**: `tracker::deepsort`

## 类定义

```cpp
class DeepSortTrack : public BaseTrack<DeepSortState> {
public:
    int32 hits = 0;                         // 匹配成功次数
    int32 age = 0;                          // 轨迹存在帧数
    int32 time_since_update = 0;            // 自上次更新以来的帧数
    std::vector<float32> features;          // ReID 外观特征向量

protected:
    int32 _n_init = 3;                      // 轨迹确认所需最少匹配帧数
    int32 _max_age = 30;                    // 轨迹最大丢失帧数

public:
    DeepSortTrack(const BOX_MEAN& mean, const BOX_COVA& covariance,
                  int32 track_id, int32 n_init, int32 max_age,
                  const std::vector<float32>& feature = {});  // 从卡尔曼状态创建

    void update(KFBox& kalman_filter, const DeepSortTrack& new_track);
    void predict(KFBox& kalman_filter);
    void mark_lost() override;
    void mark_removed() override;
    bool is_tentative() const;
    bool is_confirmed() const;
    bool is_deleted() const;
};
```

## 核心成员

| 成员                | 说明                                                  |
| ------------------- | ----------------------------------------------------- |
| `hits`              | 匹配成功次数, 用于判断是否从 Tentative 转为 Confirmed |
| `age`               | 轨迹存在帧数, 每 predict() 一次 +1                    |
| `time_since_update` | 自上次更新以来的帧数, 用于级联匹配排序                |
| `features`          | 当前帧从匹配检测框中提取的 ReID 外观特征向量          |
| `_n_init`           | 轨迹确认所需的最少匹配帧数                            |
| `_max_age`          | 轨迹最大丢失帧数, 超过此值标记为 Deleted              |

## 与 ByteTrack 轨迹的关键差异

| 对比项         | BytetrackTrack                          | DeepSortTrack                           |
| -------------- | --------------------------------------- | --------------------------------------- |
| 卡尔曼持有方式 | 每个轨迹持有 KFBox 副本                 | 共享全局 KFBox 对象 (指针传递)          |
| 创建方式       | 先创建空轨迹, activate() 时初始化卡尔曼 | 先卡尔曼 initiate(), 再创建带状态的轨迹 |
| 统计字段       | 仅有 `track_len`                        | 有 `hits`, `age`, `time_since_update`   |
| 外观特征       | 无                                      | 支持 ReID 特征 (`features`)             |
| 状态流         | New -> Tracked -> Lost -> Removed       | Tentative -> Confirmed -> Deleted       |
