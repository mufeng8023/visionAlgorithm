# NNMetric 外观特征度量学习

<!-- vscode-markdown-toc -->
- [NNMetric 外观特征度量学习](#nnmetric-外观特征度量学习)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心方法](#核心方法)

<!-- vscode-markdown-toc -->

## 概述

`NNMetric` 是 DeepSORT 的外观特征度量学习器, 用于管理轨迹的 ReID 特征库并计算最近邻距离。支持样本特征库的增量维护 (k 近邻), 通过余弦距离衡量外观相似度。

**文件**: `lib/tracker/deepsort/NNMetric.hpp`
**命名空间**: `tracker::deepsort`

## 类定义

```cpp
template <typename T>
class NNMetric {
public:
    std::vector<std::vector<T>> samples;    // 特征样本库
    int32 max_features = 100;               // 每条轨迹最大特征数

public:
    NNMetric(const TrackerConfig& config);
    void partial_fit(const std::vector<std::vector<T>>& features);  // 增量更新特征库
    std::vector<std::vector<T>> distance(const std::vector<std::vector<T>>& features);  // 计算距离矩阵
};
```

## 核心方法

| 方法          | 说明                                                                           |
| ------------- | ------------------------------------------------------------------------------ |
| `partial_fit` | 增量更新特征库: 将当前帧匹配成功的 ReID 特征存入样本库, 超出最大数量时滚动删除 |
| `distance`    | 计算所有轨迹特征与所有检测特征之间的余弦距离矩阵                               |

**余弦距离公式**:

```
cos_sim = sum(a_k * b_k) / (sqrt(sum(a_k^2)) * sqrt(sum(b_k^2)))
cos_distance = 1 - cos_sim
```

**特征库管理**:

- 每条轨迹维护一个特征样本队列 (最多 `max_features` 个)
- 每帧匹配成功后, 将当前检测的 ReID 特征存入样本库
- 超出 `max_features` 时删除最旧样本
