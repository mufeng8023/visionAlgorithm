# utils 工具函数

<!-- vscode-markdown-toc -->
- [utils 工具函数](#utils-工具函数)
  - [概述](#概述)
  - [函数列表](#函数列表)
    - [`vector_to_string` / `table_to_string`](#vector_to_string--table_to_string)
    - [`to_lower`](#to_lower)
    - [`parser_ini_det_net_config`](#parser_ini_det_net_config)
    - [`pre_process_resize_img`](#pre_process_resize_img)

<!-- vscode-markdown-toc -->

## 概述

`utils.hpp` 提供了项目中使用的各种工具函数, 包括 INI 配置文件解析、图像预处理、字符串转换等。

## 函数列表

### `vector_to_string` / `table_to_string`

```cpp
template <typename T>
std::string vector_to_string(const std::vector<T>& vec, const std::string& delimiter = ",");

template <typename T>
std::string table_to_string(const std::vector<std::vector<T>>& table);
```

将一维/二维 vector 转换为可读的字符串, 用于日志输出和调试。

### `to_lower`

```cpp
std::string to_lower(const std::string& str);
```

将字符串转换为小写。

### `parser_ini_det_net_config`

```cpp
void parser_ini_det_net_config(const std::string& ini_path, DetectionNetConfig& config);
```

从 INI 配置文件加载并解析模型配置, 填充 `DetectionNetConfig` 结构体。解析内容包括:

- 模型名称、类型、任务类型
- 类别名称列表和数量
- 置信度阈值、IoU 阈值、最大检测数
- 输入尺寸、步长、anchor 信息
- 关键点数量、维度
- 自动计算 `na`, `no`, `nl`, `net_out_h`, `net_out_w` 等派生参数

### `pre_process_resize_img`

```cpp
std::tuple<float32, int32, int32> pre_process_resize_img(
    const cv::Mat& image,
    const int32 target_height,
    const int32 target_width,
    cv::Mat& padded_img);
```

图像预处理函数, 执行等比例缩放 + 填充操作:

1. 计算缩放比例, 使图像在目标尺寸内完整显示
2. 等比例缩放图像
3. 在缩放后的图像四周填充黑边, 使其达到目标尺寸
4. 返回 `(ratio, dw, dh)` 元组, 用于后续坐标还原

坐标还原公式:

```
x = (x - dw) / ratio
y = (y - dh) / ratio
w = w / ratio
h = h / ratio
```
