# NetOutput 网络输出容器

<!-- vscode-markdown-toc -->
- [NetOutput 网络输出容器](#netoutput-网络输出容器)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心接口](#核心接口)

<!-- vscode-markdown-toc -->

## 概述

`NetOutput` 是网络输出特征图的存储容器, 内部使用 `std::vector<float32>` 管理连续内存。支持移动语义, 禁用拷贝构造以避免大内存复制。

## 类定义

**文件**: `lib/detector/NetOutput.hpp`  
**命名空间**: `yolo`

```cpp
class NetOutput {
private:
    uint32 batch_size;
    uint32 channel;
    uint32 height;
    uint32 width;
    std::vector<float32> buffer;  // 特征图数据 (NCHW 布局)
};
```

## 核心接口

| 方法                         | 说明                             |
| ---------------------------- | -------------------------------- |
| `NetOutput(b, c, h, w)`      | 构造函数, 预分配内存并初始化为 0 |
| `NetOutput()`                | 默认构造函数, 创建空对象         |
| `set_data(float32*, uint32)` | 从外部指针复制数据到缓冲区       |
| `operator[](uint32)`         | 索引访问 (可读写)                |
| `at(uint32)`                 | 安全索引访问 (返回副本)          |
| `data()`                     | 获取底层指针, 对接 C 风格接口    |
| `get_batch_size()`           | 获取 batch 大小                  |
| `get_channel()`              | 获取通道数                       |
| `get_height()`               | 获取高度                         |
| `get_width()`                | 获取宽度                         |
| `get_buffer_size()`          | 获取缓冲区元素总数               |
| `to_string()`                | 打印数据信息                     |
