# BaseNet 网络推理基类

<!-- vscode-markdown-toc -->
- [BaseNet 网络推理基类](#basenet-网络推理基类)
  - [概述](#概述)
  - [ModelPathParams 结构体](#modelpathparams-结构体)
  - [类定义](#类定义)
  - [纯虚接口](#纯虚接口)
    - [`load_model`](#load_model)
    - [`run`](#run)
  - [派生类](#派生类)

<!-- vscode-markdown-toc -->

## 概述

`BaseNet` 是所有网络推理实现的抽象基类, 定义了模型加载和推理的标准接口。采用策略模式, 允许在不修改上层代码的情况下切换不同的推理后端。

## ModelPathParams 结构体

```cpp
typedef struct {
    std::string onnx_path = "";  // ONNX 模型文件路径
} ModelPathParams;
```

## 类定义

**文件**: `lib/detector/BaseNet.hpp`  
**命名空间**: `yolo`

```cpp
class BaseNet {
public:
    ~BaseNet() = default;  // 非虚析构函数, 禁止通过基类指针删除派生类对象

    // 加载模型
    virtual bool load_model(const ModelPathParams& param, int32 device = -1) = 0;

    // 推理入口
    virtual bool run(const std::vector<cv::Mat>& images_bgr,
                     std::vector<NetOutput>& outputs) = 0;

    // 输出类别信息
    virtual std::string to_string() const;
};
```

## 纯虚接口

### `load_model`

```cpp
virtual bool load_model(const ModelPathParams& param, int32 device = -1) = 0;
```

加载 ONNX 模型文件, 配置推理设备 (CPU / GPU)。`ModelPathParams` 结构体包含 `onnx_path` 字段。

### `run`

```cpp
virtual bool run(const std::vector<cv::Mat>& images_bgr,
                 std::vector<NetOutput>& outputs) = 0;
```

执行模型推理, 输入为已 resize 到模型输入尺寸的 batch 图像, 输出为多个特征图 (对应不同 stride 的输出层)。

## 派生类

| 类名                      | 说明                       |
| ------------------------- | -------------------------- |
| [OpencvNet](OpencvNet.md) | 基于 OpenCV DNN 的推理实现 |
