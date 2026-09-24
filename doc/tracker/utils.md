# utils 工具函数

<!-- vscode-markdown-toc -->
- [utils 工具函数](#utils-工具函数)
  - [概述](#概述)
  - [函数列表](#函数列表)
    - [`parser_ini_tracker_config` (INI 配置解析)](#parser_ini_tracker_config-ini-配置解析)
    - [`tracker_type_to_string` / `tracker_type_from_string`](#tracker_type_to_string--tracker_type_from_string)

<!-- vscode-markdown-toc -->

## 概述

`utils.hpp` 提供了跟踪器相关的工具函数, 包括 INI 配置文件解析和枚举类型转换。

**文件**: `lib/tracker/utils.hpp`
**命名空间**: `tracker`

## 函数列表

### `parser_ini_tracker_config` (INI 配置解析)

```cpp
void parser_ini_tracker_config(const std::string& ini_path, TrackerConfig& config);
```

从 INI 配置文件加载并解析跟踪器配置, 填充 `TrackerConfig` 结构体。解析内容包括:

- 跟踪器类型 (`tracker_type`): bytetrack / deepsort
- 视频帧率 (`frame_rate`)
- 轨迹保留帧缓冲数 (`track_buffer`)
- 置信度阈值 (`track_thresh`, `high_thresh`)
- 匹配阈值 (`match_thresh`, `max_iou_distance`)
- 轨迹生命周期 (`max_age`, `n_init`)
- 边界框扩展率 (`expand_box_rate`)
- ByteTrack 专用参数 (`match_thresh_low`, `match_thresh_unconfirmed`)
- DeepSORT 专用参数 (`use_reid`, `reid_feature_dim`, `max_cosine_distance`, `lambda_cosine_weight`)

INI 配置文件格式示例 (参考 `config/tracker/bytetrack.ini`):

```ini
[track]
tracker_type = bytetrack
frame_rate = 30
track_buffer = 30
track_thresh = 0.5
high_thresh = 0.6
match_thresh = 0.8
max_age = 30
n_init = 3
max_iou_distance = 0.7
expand_box_rate = 0.0
match_thresh_low = 0.5
match_thresh_unconfirmed = 0.7
use_reid = false
reid_feature_dim = 0
max_cosine_distance = 0.2
lambda_cosine_weight = 0.98
```

### `tracker_type_to_string` / `tracker_type_from_string`

```cpp
std::string tracker_type_to_string(TrackerType type);
TrackerType tracker_type_from_string(const std::string& str);
```

枚举与字符串之间的相互转换, 用于配置解析。
