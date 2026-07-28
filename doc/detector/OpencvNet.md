# OpencvNet OpenCV DNN 推理实现

<!-- vscode-markdown-toc -->
- [OpencvNet OpenCV DNN 推理实现](#opencvnet-opencv-dnn-推理实现)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [1. 模型加载 `load_model`](#1-模型加载-load_model)
    - [2. 预处理 `preprocess`](#2-预处理-preprocess)
    - [3. 推理 `run`](#3-推理-run)
  - [设备选择策略](#设备选择策略)

<!-- vscode-markdown-toc -->

## 概述

`OpencvNet` 是 `BaseNet` 的派生类, 使用 OpenCV DNN 模块加载 ONNX 模型并执行推理。支持 CPU 和 CUDA GPU 两种推理模式。

## 类定义

**文件**: `lib/detector/OpencvNet.hpp`  
**命名空间**: `yolo`

```cpp
class OpencvNet : public BaseNet {
private:
    cv::dnn::Net net;           // OpenCV DNN 网络对象
    uint32 batch_size;          // Batch 大小
    uint32 nc;                  // 类别数量
    uint32 input_width;         // 输入宽度
    uint32 input_height;        // 输入高度
    uint32 input_channels;      // 输入通道数
    std::vector<uint32> strides;// 各输出层步长
    std::vector<std::vector<float32>> anchors; // anchor 信息
    uint32 na;                  // 每个位置 anchor 数
    uint32 no;                  // 每个位置输出信息数
    uint32 nl;                  // 输出层数
    std::vector<uint32> net_out_h;  // 各输出层高度
    std::vector<uint32> output_w;   // 各输出层宽度
    std::vector<uint32> output_len; // 各输出层数据元素总数
    cv::Mat inputBatch;         // 预处理后的输入 Blob
};
```

## 核心流程

### 1. 模型加载 `load_model`

从 ONNX 文件加载网络, 根据 `device` 参数自动选择 CPU 或 CUDA 后端。

### 2. 预处理 `preprocess`

```cpp
void preprocess(const std::vector<cv::Mat>& images_bgr);
```

使用 `cv::dnn::blobFromImages` 将输入图像转换为 NCHW 格式的 Blob:

- 缩放至模型输入尺寸
- BGR -> RGB 通道交换
- 输出数据类型为 `CV_8U` (uint8)

### 3. 推理 `run`

```cpp
bool run(const std::vector<cv::Mat>& images_bgr,
         std::vector<NetOutput>& outputs) override;
```

1. 调用 `preprocess` 预处理图像
2. 设置网络输入 `net.setInput(inputBatch)`
3. 前向推理 `net.forward()` 获取多输出层特征图
4. 校验输出形状, 将数据复制到 `NetOutput` 容器

## 设备选择策略

| device 参数 | 行为                                               |
| ----------- | -------------------------------------------------- |
| `-1`        | 强制使用 CPU (DNN_BACKEND_OPENCV + DNN_TARGET_CPU) |
| `>= 0`      | 检测 CUDA GPU 可用性, 可用则使用 GPU, 否则回退 CPU |
| 越界        | 返回 `false`, 加载失败                             |
