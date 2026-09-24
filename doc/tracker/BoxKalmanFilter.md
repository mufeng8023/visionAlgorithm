# BoxKalmanFilter Box 跟踪卡尔曼滤波器

<!-- vscode-markdown-toc -->
- [BoxKalmanFilter Box 跟踪卡尔曼滤波器](#boxkalmanfilter-box-跟踪卡尔曼滤波器)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
    - [`initiate` (初始化)](#initiate-初始化)
    - [`predict` (预测)](#predict-预测)
    - [`project` (投影)](#project-投影)
  - [噪声模型与参数](#噪声模型与参数)

<!-- vscode-markdown-toc -->

## 概述

`KFBox` 是 `BaseKalmanFilter<8, 4>` 的派生类, 实现了 Bounding Box 目标跟踪的恒定速度卡尔曼滤波器。适用于 SORT / ByteTrack / DeepSORT / OC_SORT 等所有基于检测框的跟踪器。

状态空间 8 维: `[cx, cy, a, h, v_cx, v_cy, v_a, v_h]`

- cx, cy: 目标框中心坐标 (像素)
- a: 宽高比 (width / height)
- h: 目标框高度 (像素)
- v_*: 对应的速度分量 (像素/帧)

观测空间 4 维: `[cx, cy, a, h]`

`KFBox` 对象本身是无状态的 (轨迹状态存储在外部的 mean / covariance 中), 一个 KFBox 实例可以同时服务所有轨迹。

**文件**: `lib/tracker/BoxKalmanFilter.hpp`
**命名空间**: `tracker`

## 类定义

```cpp
class KFBox : public KFBase<8, 4> {
private:
    Eigen::Matrix<float32, 8, 8, Eigen::RowMajor> _motion_mat;  // 状态转移矩阵 F
    Eigen::Matrix<float32, 4, 8, Eigen::RowMajor> _update_mat;  // 观测矩阵 H
    float32 _std_weight_position;    // 位置过程噪声标准差权重 (1/20 = 0.05)
    float32 _std_weight_velocity;    // 速度过程噪声标准差权重 (1/160 = 0.00625)

protected:
    const MatH& get_update_matrix() const override;

public:
    KFBox();
    DataS initiate(const VecM& measurement) override;
    void predict(VecS& mean, MatS& covariance) override;
    DataM project(const VecS& mean, const MatS& covariance) override;
};
```

## 核心流程

### `initiate` (初始化)

```cpp
DataS initiate(const VecM& measurement) override;
```

当目标第一次被检测到时, 使用检测值初始化卡尔曼状态:

1. 均值 mean: `[cx, cy, a, h, 0, 0, 0, 0]` (位置来自检测值, 速度初始化为 0)
2. 协方差: 基于 `_std_weight_position` 和 `_std_weight_velocity` 构建对角矩阵
   - 位置分量标准差 = `_std_weight_position * measurement[3]` (与 h 成正比)
   - 速度分量标准差 = `_std_weight_velocity * measurement[3]` (与 h 成正比)

### `predict` (预测)

```cpp
void predict(VecS& mean, MatS& covariance) override;
```

每帧获取检测结果前调用, 利用恒定速度模型将状态向前推进一帧:

1. 状态均值: `mean_new = F * mean_old` (位置 = 旧位置 + 速度)
2. 状态协方差: `covariance_new = F * covariance_old * F^T + Q` (Q 为过程噪声)
   - Q 为对角矩阵, 元素基于 `_std_weight_position` / `_std_weight_velocity` 和当前高度 h 构建

### `project` (投影)

```cpp
DataM project(const VecS& mean, const MatS& covariance) override;
```

将状态空间的均值与协方差投影到观测空间:

1. 预测观测均值: `z_pred = H * mean`
2. 新息协方差: `S = H * covariance * H^T + R` (R 为测量噪声)
   - R 为对角矩阵, 元素基于 `_std_weight_position` 和目标高度 h 构建

## 噪声模型与参数

过程噪声和测量噪声均与目标高度 h 成正比 (自适应噪声模型):

| 参数                   | 默认值          | 说明                                               |
| ---------------------- | --------------- | -------------------------------------------------- |
| `_std_weight_position` | 1/20 = 0.05     | 位置噪声权重; 大目标运动时像素变化大, 小目标变化小 |
| `_std_weight_velocity` | 1/160 = 0.00625 | 速度噪声权重; 远小于位置噪声, 假设速度平滑变化     |

噪声计算公式:

```
过程噪声 Q 的对角元素:
  position_std = _std_weight_position * h
  velocity_std = _std_weight_velocity * h

测量噪声 R 的对角元素:
  meas_std = _std_weight_position * h
