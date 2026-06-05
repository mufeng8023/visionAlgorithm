# draw_result 结果可视化

<!-- vscode-markdown-toc -->
- [draw\_result 结果可视化](#draw_result-结果可视化)
  - [概述](#概述)
  - [函数列表](#函数列表)
    - [`get_color` (颜色生成)](#get_color-颜色生成)
    - [`draw_detection_result` (绘制检测框)](#draw_detection_result-绘制检测框)
    - [`draw_pose_result` (绘制姿态关键点)](#draw_pose_result-绘制姿态关键点)

<!-- vscode-markdown-toc -->

## 概述

`draw_result.hpp` 提供了检测结果的可视化绘制工具, 支持在图像上绘制检测框、类别标签、置信度分数以及姿态关键点骨架。颜色使用 HSL 色环均匀分布算法, 确保不同类别获得视觉上区分度高的颜色。

## 函数列表

### `get_color` (颜色生成)

```cpp
static std::tuple<uint8, uint8, uint8> get_color(int32 index, bool is_rgb);
```

使用黄金比例 (137.508°) 在色环上均匀分布颜色, 确保不同类别获得视觉上区分度高的颜色。

- `index`: 类别索引, 不同索引生成不同色相的颜色
- `is_rgb`: `true` 返回 (R, G, B) 顺序, `false` 返回 (B, G, R) 顺序 (OpenCV 默认)

### `draw_detection_result` (绘制检测框)

```cpp
void draw_detection_result(cv::Mat& image_bgr,
                           const std::vector<YoloObject>& det_results,
                           const std::vector<std::string>& names);
```

在图像上绘制每个检测到的目标的边界框和类别标签:

- 框线粗细和文字大小根据图像尺寸自适应调整
- 标签位置自动调整以避免超出图像边界
- 标签格式: `类别名:置信度` (例如 `person:0.92`)

### `draw_pose_result` (绘制姿态关键点)

```cpp
void draw_pose_result(cv::Mat& image_bgr,
                      const std::vector<YoloObject>& det_results,
                      const std::vector<std::string>& names,
                      const float32 kpt_thr = 0.5f);
```

在检测框绘制的基础上, 额外绘制姿态关键点和骨架连接线:

- 内部先调用 `draw_detection_result` 绘制检测框
- 关键点置信度低于 `kpt_thr` 时会被过滤不显示
- 内置 COCO 17 关键点骨架连接关系定义
- 关键点和骨架线段使用独立颜色, 避免与检测框颜色冲突
