# DeepSortTracker DeepSORT 跟踪器

<!-- vscode-markdown-toc -->
- [DeepSortTracker DeepSORT 跟踪器](#deepsorttracker-deepsort-跟踪器)
  - [概述](#概述)
  - [核心思想](#核心思想)
  - [类定义](#类定义)
  - [核心流程](#核心流程)

<!-- vscode-markdown-toc -->

## 概述

`DeepSORTTracker` 是 DeepSORT 跟踪算法的实现, 继承自 `BaseTracker`。DeepSORT 在 SORT 的基础上增加了外观特征 (ReID) 的级联匹配策略, 显著提升了遮挡场景下的身份保持能力。

**文件**: `lib/tracker/deepsort/DeepSortTracker.hpp`
**命名空间**: `tracker::deepsort`

## 核心思想

DeepSORT 的级联匹配 (Cascade Matching) 设计:

1. **级联匹配**: 优先匹配最近更新过的轨迹 (time_since_update 小的优先)
   - 使用马氏距离 (位置) + 余弦距离 (外观) 的加权融合距离
   - 马氏距离用于门控过滤 (排除距离过远的匹配)
   - 余弦距离用于外观特征的相似度匹配
   - 融合公式: `combined = (1 - lambda) * 马氏距离 + lambda * 余弦距离`
2. **IoU 二次匹配**: 级联匹配未匹配的轨迹再尝试 IoU 关联
3. **新轨迹初始化**: 未匹配的高分检测创建新轨迹

**与 ByteTrack 的核心区别**:

| 对比项      | ByteTrack        | DeepSORT                  |
| ----------- | ---------------- | ------------------------- |
| 匹配策略    | 三轮 IoU 关联    | 级联匹配 + IoU 二次匹配   |
| 特征        | 仅位置 (IoU)     | 位置 (马氏) + 外观 (余弦) |
| ReID 模型   | 不需要           | 需要                      |
| 轨迹持有 KF | 每个轨迹持有副本 | 共享全局 KFBox 对象       |

## 类定义

```cpp
class DeepSORTTracker : public BaseTracker {
public:
    std::vector<DeepSortTrack> tracks;              // 所有轨迹
    NNMetric metric;                                // 外观特征度量学习器

public:
    explicit DeepSORTTracker(const TrackerConfig& config);
    void update(const std::vector<BoxObject>& detections,
                std::vector<TrackResult>& results, int32 frame_id = -1) override;
    void reset() override;
};
```

## 核心流程

DeepSORTTracker 的 `update()` 方法每帧执行以下流程:

1. **帧 ID 自增, 卡尔曼预测**: 对所有已确认轨迹执行 `predict()`
2. **轨迹分流**: 分离为 Confirmed 和 Tentative 两组
3. **级联匹配**: Confirmed 轨迹按 `time_since_update` 排序, 年龄小的优先
   - 融合马氏距离 + 余弦距离
   - 使用匈牙利算法求解最优匹配
4. **IoU 二次匹配**: 级联匹配未匹配的轨迹再与剩余检测 IoU 关联
5. **Tentative 轨迹匹配**: 未确认轨迹与剩余检测做 IoU 匹配
6. **初始化新轨迹**: 未匹配检测创建 Tentative 状态的新轨迹
7. **更新状态**: 标记失活轨迹 (超时删除), 更新轨迹状态
8. **更新特征库**: 将匹配成功的 ReID 特征存入 NNMetric 特征库
9. **输出**: 返回 Confirmed 状态的轨迹结果
