# BaseKalmanFilter 卡尔曼滤波器泛型基类

<!-- vscode-markdown-toc -->
- [BaseKalmanFilter 卡尔曼滤波器泛型基类](#basekalmanfilter-卡尔曼滤波器泛型基类)
  - [概述](#概述)
  - [模板参数](#模板参数)
  - [内部类型别名](#内部类型别名)
  - [类定义](#类定义)
  - [纯虚接口](#纯虚接口)
    - [`initiate`](#initiate)
    - [`predict`](#predict)
    - [`project`](#project)
  - [通用默认实现](#通用默认实现)
    - [`update` (卡尔曼更新)](#update-卡尔曼更新)
    - [`gating_distance` (马氏距离计算)](#gating_distance-马氏距离计算)
  - [派生类](#派生类)

<!-- vscode-markdown-toc -->

## 概述

`BaseKalmanFilter` 是所有卡尔曼滤波器实现的泛型抽象基类。通过 C++ 模板参数在编译期决定状态空间与观测空间的维度, 完全消除动态内存分配, 使用栈上的固定尺寸 Eigen 矩阵。

设计采用模板方法模式 (Template Method):

- **纯虚接口** (子类必须实现): `initiate`, `predict`, `project` — 差异化部分 (噪声建模)
- **通用默认实现** (基类已提供): `update`, `gating_distance` — 公共数学核心 (子类可按需覆盖)

基类的 `update()` 和 `gating_distance()` 通过内部的 `get_update_matrix()` 纯虚函数获取子类的观测矩阵 H, 实现算法的复用。

## 模板参数

| 参数         | 说明                                        |
| ------------ | ------------------------------------------- |
| `StateDim`   | 状态空间维数 (Box 跟踪为 8, 关键点平滑为 4) |
| `MeasureDim` | 观测空间维数 (Box 跟踪为 4, 关键点平滑为 2) |

## 内部类型别名

| 别名    | 尺寸                      | 说明                              |
| ------- | ------------------------- | --------------------------------- |
| `VecS`  | `1 x StateDim`            | 状态行向量 (如 `[位置, 速度]`)    |
| `MatS`  | `StateDim x StateDim`     | 状态方阵 (协方差, 运动矩阵 F)     |
| `VecM`  | `1 x MeasureDim`          | 观测行向量 (如 `[cx, cy, a, h]`)  |
| `MatM`  | `MeasureDim x MeasureDim` | 观测方阵 (新息协方差 S)           |
| `MatH`  | `MeasureDim x StateDim`   | 观测矩阵 H (状态空间 -> 观测空间) |
| `DataS` | `std::pair<VecS, MatS>`   | 状态域数据对 (均值 + 协方差)      |
| `DataM` | `std::pair<VecM, MatM>`   | 观测域数据对 (均值 + 协方差)      |

## 类定义

**文件**: `lib/tracker/BaseKalmanFilter.hpp`
**命名空间**: `tracker`

```cpp
template <int StateDim, int MeasureDim>
class BaseKalmanFilter {
public:
    virtual ~BaseKalmanFilter() = default;

    // 纯虚接口
    virtual DataS initiate(const VecM& measurement) = 0;
    virtual void predict(VecS& mean, MatS& covariance) = 0;
    virtual DataM project(const VecS& mean, const MatS& covariance) = 0;

    // 通用默认实现
    virtual DataS update(const VecS& mean, const MatS& covariance, const VecM& measurement);
    virtual Eigen::Matrix<float32, 1, Eigen::Dynamic>
        gating_distance(const VecS& mean, const MatS& covariance,
                        const std::vector<VecM>& measurements);

protected:
    virtual const MatH& get_update_matrix() const = 0;
};
```

## 纯虚接口

### `initiate`

```cpp
virtual DataS initiate(const VecM& measurement) = 0;
```

初始化新轨迹。当目标第一次被检测到时调用, 子类根据自身噪声模型构建合理的初始均值和协方差。返回 `{初始状态均值, 初始状态协方差}`。

### `predict`

```cpp
virtual void predict(VecS& mean, MatS& covariance) = 0;
```

预测步骤。每帧在获取检测结果前调用, 利用运动模型将状态向前推进一帧。`mean` 和 `covariance` 均为输入/输出参数。

### `project`

```cpp
virtual DataM project(const VecS& mean, const MatS& covariance) = 0;
```

投影步骤 (内部使用)。将状态空间的均值和协方差映射到观测空间。被基类的 `update()` 和 `gating_distance()` 内部调用。返回 `{预测的观测均值 VecM, 新息协方差 MatM (含测量噪声 R)}`。

## 通用默认实现

### `update` (卡尔曼更新)

```cpp
virtual DataS update(const VecS& mean, const MatS& covariance, const VecM& measurement);
```

通用卡尔曼更新步骤 (标准 Kalman 公式 3~7)。检测框与轨迹匹配成功后调用, 用实际检测值修正预测值, 得到后验估计。

算法流程:

1. 调用 `project()` 获取预测测量值 `z_pred` 和新息协方差 `S`
2. 计算新息: `y = z - z_pred`
3. 通过 `get_update_matrix()` 获取 H, 计算卡尔曼增益 K
4. 使用 Cholesky 分解求解 `S * K^T = B`, 比直接求逆矩阵更稳定高效
5. 后验均值: `new_mean = mean + K * y`
6. 后验协方差: `new_covariance = covariance - K * S * K^T`

### `gating_distance` (马氏距离计算)

```cpp
virtual Eigen::Matrix<float32, 1, Eigen::Dynamic>
    gating_distance(const VecS& mean, const MatS& covariance,
                    const std::vector<VecM>& measurements);
```

通用马氏距离计算。衡量预测位置与实际检测位置之间的归一化距离。马氏距离考虑了各维度的方差和相关性, 比欧氏距离更合理。常用于 DeepSORT 的门控过滤: 距离超过阈值则不允许匹配。

算法流程:

1. 调用 `project()` 获取预测测量值和新息协方差 `S`
2. 构建差值矩阵 `diff` (N x MeasureDim)
3. 通过 Cholesky 分解 `S = L * L^T`, 计算归一化残差
4. 返回每个检测值的马氏距离平方 (1 x N)

## 派生类

| 类名                                                         | 状态/观测维度 | 说明                               |
| ------------------------------------------------------------ | ------------- | ---------------------------------- |
| [KFBox (BoxKalmanFilter)](BoxKalmanFilter.md)                | 8/4           | Box 跟踪, 恒定速度模型, 噪声自适应 |
| [KFKeypoint (KeypointKalmanFilter)](KeypointKalmanFilter.md) | 4/2           | 关键点平滑, 固定/自适应双模式      |
