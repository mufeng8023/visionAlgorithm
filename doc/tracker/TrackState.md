# TrackState 轨迹状态枚举

<!-- vscode-markdown-toc -->
- [TrackState 轨迹状态枚举](#trackstate-轨迹状态枚举)
  - [概述](#概述)
  - [枚举定义](#枚举定义)
    - [`ByteTrackState` (ByteTrack 轨迹状态)](#bytetrackstate-bytetrack-轨迹状态)
    - [`DeepSortState` (DeepSORT 轨迹状态)](#deepsortstate-deepsort-轨迹状态)
  - [状态转换](#状态转换)
    - [ByteTrack 状态机](#bytetrack-状态机)
    - [DeepSORT 状态机](#deepsort-状态机)

<!-- vscode-markdown-toc -->

## 概述

`TrackState.hpp` 定义了 ByteTrack 和 DeepSORT 的轨迹生命周期状态枚举及辅助函数。

**文件**: `lib/tracker/TrackState.hpp`
**命名空间**: `tracker`

## 枚举定义

### `ByteTrackState` (ByteTrack 轨迹状态)

```cpp
enum class ByteTrackState : uint8 {
    New = 0,      // 新创建, 等待确认 (未激活卡尔曼)
    Tracked,      // 已确认, 正在跟踪 (已激活卡尔曼)
    Lost,         // 丢失, 等待重新激活
    Removed       // 已移除, 将被清理
};
```

| 状态      | 说明                                                         |
| --------- | ------------------------------------------------------------ |
| `New`     | 刚创建; 连续 n_init 帧匹配成功 -> Tracked; 未匹配 -> Removed |
| `Tracked` | 已确认; 未匹配 -> Lost; 匹配成功保持                         |
| `Lost`    | 暂时丢失; 再次匹配成功 -> Tracked; 超过 max_age -> Removed   |
| `Removed` | 已移除; 不再参与任何匹配                                     |

### `DeepSortState` (DeepSORT 轨迹状态)

```cpp
enum class DeepSortState : uint8 {
    Tentative = 0,  // 暂定, 等待确认
    Confirmed,      // 已确认
    Deleted         // 已删除
};
```

| 状态        | 说明                                                       |
| ----------- | ---------------------------------------------------------- |
| `Tentative` | 暂定; 连续 n_init 帧匹配成功 -> Confirmed; 否则 -> Deleted |
| `Confirmed` | 已确认; 未匹配 -> 超过 max_age -> Deleted                  |
| `Deleted`   | 已删除; 不再参与任何匹配                                   |

## 状态转换

### ByteTrack 状态机

```
                 连续 n_init 帧匹配成功
  New ──────────────────────────────────────> Tracked
   │                                            │
   │ 未匹配                                     │ 未匹配
   v                                            v
  Removed <────────────────────── Lost ─────────┘
              超 max_age           │
                                  │ 重新匹配成功
                                  v
                                Tracked (重新激活)
```

### DeepSORT 状态机

```
                 连续 n_init 帧匹配成功
  Tentative ──────────────────────────────> Confirmed
      │                                        │
      │ 未匹配                                  │ 超 max_age
      v                                        v
   Deleted <────────────────────────────────── Deleted
