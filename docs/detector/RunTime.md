# RunTime 运行时调度器

<!-- vscode-markdown-toc -->
- [RunTime 运行时调度器](#runtime-运行时调度器)
  - [概述](#概述)
  - [ModelBench 枚举](#modelbench-枚举)
  - [类定义](#类定义)
  - [构造函数](#构造函数)
  - [核心接口](#核心接口)
    - [推理入口 `operator()`](#推理入口-operator)
    - [结果可视化 `draw_result`](#结果可视化-draw_result)
  - [使用示例](#使用示例)

<!-- vscode-markdown-toc -->

## 概述

`RunTime` 是项目的核心调度类, 负责串联整个推理流程: 加载配置 -> 初始化模型 -> 初始化后处理 -> 图像预处理 -> 模型推理 -> 后处理 -> 坐标还原 -> 结果可视化。它封装了所有内部细节, 对外提供简洁的调用接口。

## ModelBench 枚举

```cpp
enum class ModelBench : uint8 {
    OpenCV = 0,
    count,
};
```

当前仅支持 `OpenCV` 推理后端, 后续可扩展其他框架。枚举转字符串使用 `model_bench_to_string()` (O(1) 性能, 基于 `std::array`), 字符串转枚举使用 `model_bench_from_string()` (O(n) 遍历)。

## 类定义

**文件**: `lib/detector/RunTime.hpp`  
**命名空间**: `yolo`

```cpp
class RunTime {
    // 私有成员
    DetectionNetConfig config;              // 模型配置
    std::shared_ptr<BaseNet> net;           // 网络推理实例 (多态)
    std::shared_ptr<BasePostProcess> postProcess; // 后处理实例 (多态)
    std::vector<cv::Mat> imgs_resized;      // 预处理后的图像缓存
    std::vector<std::tuple<float32, int32, int32>> resize_info; // 缩放信息 (ratio, dw, dh)
    std::vector<NetOutput> net_outputs;     // 网络输出缓存
    std::vector<ObjectBuffer> results;      // 后处理结果缓存
};
```

## 构造函数

```cpp
RunTime(const std::string& config_path,
        const ModelPathParams& param,
        const ModelBench& net_bench = ModelBench::OpenCV,
        int32 device = -1);
```

构造函数执行以下初始化流程:

1. 解析 INI 配置文件, 填充 `DetectionNetConfig` 结构体
2. 根据 `ModelBench` 枚举创建对应的网络推理实例 (当前仅支持 `OpenCV`)
3. 根据 `TaskType` 和 `ModelType` 创建对应的后处理实例:
   - `detection` + `yolov4` -> `DetPostProcessV4`
   - `detection` + `yolov5` -> `DetPostProcessV5`
   - `detection` + `yolov3u/yolov5u/yolov6u/yolov8/yolov9/yolo11/yolo12` -> `DetPostProcessV8`
   - `detection` + `yolov10/yolo26` -> `DetPostProcess26`
   - `pose` + `yolov5` -> `PosePostProcessV5`
   - `pose` + `yolov5u/yolov8/yolo11` -> `PosePostProcessV8`
   - `pose` + `yolo26` -> `PosePostProcess26`
4. 预分配图像预处理缓存、网络输出缓存、检测结果缓存

## 核心接口

### 推理入口 `operator()`

```cpp
void operator()(const std::vector<cv::Mat>& images_bgr,
                std::vector<std::vector<YoloObject>>& det_results);
```

执行完整的推理流水线:

1. **图像预处理**: 等比例缩放 + 填充至模型输入尺寸, 记录缩放信息 (ratio, dw, dh)
2. **模型推理**: 调用 `BaseNet::run()` 获取特征图
3. **后处理**: 调用 `BasePostProcess::run()` 解析检测结果
4. **坐标还原**: 将检测框和关键点坐标映射回原始图像尺寸
5. **结果输出**: 填充 `std::vector<std::vector<YoloObject>>`

### 结果可视化 `draw_result`

```cpp
void draw_result(std::vector<cv::Mat>& images_bgr,
                 const std::vector<std::vector<yolo::YoloObject>>& det_results);
```

根据任务类型自动选择绘制方式:

- `detection` 任务: 调用 `draw_detection_result` 绘制检测框
- `pose` 任务: 调用 `draw_pose_result` 绘制检测框 + 关键点骨架

## 使用示例

```cpp
#include "RunTime.hpp"

// 初始化运行时
yolo::RunTime run_time(
    "../config/model.ini",           // 模型配置文件
    { "../onnx/model.onnx" },        // 模型路径参数
    yolo::ModelBench::OpenCV,        // 推理框架
    0                                // GPU 设备编号
);

// 读取图片
cv::Mat image = cv::imread("test.jpg");
std::vector<cv::Mat> images = { image };

// 推理
std::vector<std::vector<yolo::YoloObject>> results;
run_time(images, results);

// 绘制结果
run_time.draw_result(images, results);

// 保存结果
cv::imwrite("result.jpg", images[0]);
```
