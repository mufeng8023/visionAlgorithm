# ByteTracker ByteTrack 跟踪器

<!-- vscode-markdown-toc -->
- [ByteTracker ByteTrack 跟踪器](#bytetracker-bytetrack-跟踪器)
  - [概述](#概述)
  - [核心思想](#核心思想)
  - [类定义](#类定义)
  - [核心流程](#核心流程)

<!-- vscode-markdown-toc -->

## 概述

`ByteTracker` 是 ByteTrack 跟踪算法的实现, 继承自 `BaseTracker`。ByteTrack 的核心创新在于利用低分检测框 (被遮挡时的检测) 进行二次关联, 显著提升遮挡场景下的跟踪鲁棒性。

**文件**: `lib/tracker/bytetrack/ByteTracker.hpp`
**命名空间**: `tracker::bytetrack`

## 核心思想

ByteTrack 的"Byte"来源于"Byte" (字节), 意思是"不放过任何一个检测框, 就像不放过任何一个字节"。

传统跟踪器 (如 SORT) 只使用高置信度检测框进行关联, 低置信度检测框直接丢弃。ByteTrack 的创新在于: **低分检测框也有价值**, 它通过两次关联来利用所有检测框。

**三轮 IoU 匹配设计**:

| 关联 | 名称       | 左集合                | 右集合       | 阈值                              |
| ---- | ---------- | --------------------- | ------------ | --------------------------------- |
| 一   | 高分匹配   | 轨迹池 (Tracked+Lost) | 高分检测     | `match_thresh`                    |
| 二   | 低分匹配   | 未匹配 Tracked 轨迹   | 低分检测     | `match_thresh_low` (宽松)         |
| 三   | 未确认匹配 | 未确认轨迹 (New)      | 剩余高分检测 | `match_thresh_unconfirmed` (严格) |

**设计假设**:

- 高分检测框通常是正确的检测 (目标清晰可见)
- 低分检测框通常是部分遮挡或模糊的检测 (目标被遮挡)
- 如果轨迹被遮挡, 它很可能匹配不到高分检测, 但有可能匹配到一个低分检测

## 类定义

```cpp
class ByteTracker : public BaseTracker {
public:
    std::vector<BytetrackTrack> tracked_stracks;   // Tracked + New 状态轨迹
    std::vector<BytetrackTrack> lost_stracks;       // Lost 状态轨迹
    std::vector<BytetrackTrack> removed_stracks;    // Removed 状态轨迹
    std::vector<BytetrackTrack> output_stracks;     // 输出结果

public:
    explicit ByteTracker(const TrackerConfig& config);
    void update(const std::vector<BoxObject>& detections,
                std::vector<TrackResult>& results, int32 frame_id = -1) override;
};
```

## 核心流程

ByteTracker 的 `update()` 方法每帧执行以下流程:

1. **帧 ID 自增**
2. **检测框分流**: 按 `high_thresh` 将检测框分为高分和低分两组
3. **轨迹分离**: 将 `tracked_stracks` 分离为 `active_tracked` (Tracked) 和 `unconfirmed` (New)
4. **轨迹池预测**: 轨迹池 = `active_tracked + lost_stracks`, 执行卡尔曼预测
5. **关联一**: 轨迹池 x 高分检测, 使用 `match_thresh`
6. **关联二**: 未匹配 Tracked 轨迹 x 低分检测, 使用 `match_thresh_low`
7. **关联三**: 未确认轨迹 x 剩余高分检测, 使用 `match_thresh_unconfirmed`
8. **初始化新轨迹**: 仅使用剩余的高分检测
9. **状态更新**: 更新所有轨迹的状态 (Tracked / Lost / Removed)
10. **去重**: 移除 `tracked_stracks` 和 `lost_stracks` 中的重复轨迹
11. **输出**: 将 Tracked 状态的轨迹输出到 `results`
