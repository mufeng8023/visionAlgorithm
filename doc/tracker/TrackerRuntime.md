# TrackerRuntime 跟踪运行时调度器

<!-- vscode-markdown-toc -->
- [TrackerRuntime 跟踪运行时调度器](#trackerruntime-跟踪运行时调度器)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [`init` (初始化)](#init-初始化)
    - [`update` (每帧更新)](#update-每帧更新)
  - [核心接口](#核心接口)
  - [det\_index 回溯机制](#det_index-回溯机制)

<!-- vscode-markdown-toc -->

## 概述

`TrackerRuntime` 是跟踪器的高级接口, 封装了底层跟踪算法 (ByteTrack / DeepSORT) 的统一运行时。上层应用无需关心具体的跟踪算法细节, 只需:

1. 调用 `init()` 加载 ini 配置文件, 自动初始化跟踪器
2. 每帧调用 `update()` 传入 `YoloObject` 检测结果
3. 通过返回的 `TrackResult` 列表获取每个活跃目标的轨迹 ID 和检测索引
4. 通过 `get_histories()` 获取所有轨迹的历史信息

**文件**: `lib/tracker/TrackerRuntime.hpp`
**命名空间**: `tracker`

## 类定义

```cpp
class TrackerRuntime {
public:
    TrackerConfig _config;                          // 跟踪器配置
    int32 _current_frame_id = 0;                    // 当前帧 ID
    std::map<int32, TrackHistory> _histories;        // 轨迹历史 map
    int32 _max_history_frames = 150;                 // 最大历史帧数

private:
    std::unique_ptr<BaseTracker> _tracker;           // 多态跟踪器指针
    bool _is_initialized = false;                    // 初始化状态

public:
    TrackerRuntime() = default;
    ~TrackerRuntime() = default;

    void init(const std::string& ini_path, int32 max_history_frames = 150);
    std::vector<TrackResult> update(
        const std::vector<yolo::YoloObject>& detections, int32 frame_id = -1);
    const std::map<int32, TrackHistory>& get_histories() const;
    size_t active_track_count() const;
    size_t total_track_count() const;
    void reset();
    const TrackerConfig& config() const;
};
```

## 核心流程

### `init` (初始化)

```cpp
void init(const std::string& ini_path, int32 max_history_frames = 150);
```

1. 从 ini 配置文件解析跟踪器参数 (`parser_ini_tracker_config`)
2. 根据 `tracker_type` 创建对应的跟踪器实例:
   - `bytetrack`: 创建 `ByteTracker`
   - `deepsort`: 创建 `DeepSORTTracker`
3. 通过基类指针 (多态) 统一管理

支持通过 `max_history_frames` 控制每条轨迹最多保留的历史帧数 (滚动窗口, 默认 150 帧约 5 秒 @30fps)。

### `update` (每帧更新)

```cpp
std::vector<TrackResult> update(
    const std::vector<yolo::YoloObject>& detections, int32 frame_id = -1);
```

每帧调用, 内部执行以下流程:

1. **帧 ID 管理**: `frame_id < 0` 时自动自增
2. **排序**: 按置信度从高到低对检测结果排序, 记录原始下标
3. **BoxObject 转换**: 将 `YoloObject` 转换为 `BoxObject` 送入跟踪器
4. **跟踪器更新**: 通过虚函数调用底层跟踪器的 `update()`
5. **det_index 回映射**: 将排序后的下标映射回原始 `detections[]` 下标
6. **历史更新**: 自动更新 `_histories` 轨迹历史记录

## 核心接口

| 方法                           | 说明                                       |
| ------------------------------ | ------------------------------------------ |
| `init(ini_path, max_hist)`     | 初始化, 加载配置, 创建跟踪器实例           |
| `update(detections, frame_id)` | 每帧更新, 返回 TrackResult 列表            |
| `get_histories()`              | 获取所有轨迹的历史记录 map (key=track_id)  |
| `active_track_count()`         | 获取当前帧活跃轨迹数量                     |
| `total_track_count()`          | 获取历史上出现过的轨迹总数 (自 reset 以来) |
| `reset()`                      | 重置跟踪器和所有历史记录                   |
| `config()`                     | 获取当前跟踪器配置 (只读)                  |

## det_index 回溯机制

`update()` 内部先对 `YoloObject` 按置信度排序, 再转换为 `BoxObject` 送入跟踪器。跟踪器内部在匹配阶段直接记录 `track_id → BoxObject` 下标映射。排序后的下标由 `sorted_indices` 回映射至原始 `detections[]` 下标。因此 `TrackResult.det_index` 直接对应调用方的 `detections[det_index]`。

```
原始检测 detections[]
  ↓ 排序 (按置信度)
排序后 box_objects[]
  ↓ 跟踪器匹配
结果: track_id ↔ box_object_index
  ↓ 回映射 sorted_indices[box_object_index]
结果: TrackResult.det_index = 原始 detections[] 下标
