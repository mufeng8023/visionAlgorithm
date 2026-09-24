# KeypointKalmanFilter 关键点平滑卡尔曼滤波器

<!-- vscode-markdown-toc -->
- [KeypointKalmanFilter 关键点平滑卡尔曼滤波器](#keypointkalmanfilter-关键点平滑卡尔曼滤波器)
  - [概述](#概述)
  - [类定义](#类定义)
  - [核心流程](#核心流程)
  - [噪声模式对比](#噪声模式对比)

<!-- vscode-markdown-toc -->

## 概述

`KFKeypoint` 是 `BaseKalmanFilter<4, 2>` 的派生类, 实现了关键点 XY 坐标平滑的恒定速度卡尔曼滤波器。适用于姿态估计关键点平滑 (人体骨骼点, 手部关键点, 面部关键点等) 及任何仅有 XY 二维坐标输入的平滑需求。

**与 KFBox 的设计对比**:

| 对比项   | KFBox (8维/4维)                        | KFKeypoint (4维/2维)        |
| -------- | -------------------------------------- | --------------------------- |
| 状态     | `[cx, cy, a, h, v_cx, v_cy, v_a, v_h]` | `[x, y, v_x, v_y]`          |
| 观测     | `[cx, cy, a, h]`                       | `[x, y]`                    |
| 噪声模型 | 与目标高度 h 自适应缩放                | 固定模式 + 自适应模式双模式 |

**文件**: `lib/tracker/KeypointKalmanFilter.hpp`
**命名空间**: `tracker`

## 类定义

```cpp
class KFKeypoint : public KFBase<4, 2> {
private:
    // 固定噪声模式参数
    float32 _p_noise;    // 过程噪声标准差 (固定模式)
    float32 _m_noise;    // 测量噪声标准差 (固定模式)

    // 自适应模式参数 (与 KFBox 对称)
    float32 _std_weight_position;  // 位置噪声权重, 默认 1/20 = 0.05
    float32 _std_weight_velocity;  // 速度噪声权重, 默认 1/160 = 0.00625

    Eigen::Matrix<float32, 4, 4, Eigen::RowMajor> _motion_mat;  // F
    Eigen::Matrix<float32, 2, 4, Eigen::RowMajor> _update_mat;  // H

protected:
    const MatH& get_update_matrix() const override;

public:
    KFKeypoint(float32 p_noise = 0.05f, float32 m_noise = 0.5f);

    // 模式 A: 固定噪声 (无 bbox_height)
    DataS initiate(const VecM& measurement) override;
    void predict(VecS& mean, MatS& covariance) override;
    DataM project(const VecS& mean, const MatS& covariance) override;

    // 模式 B: 高度自适应 (带 bbox_height)
    DataS initiate(const VecM& measurement, float32 bbox_height);
    void predict(VecS& mean, MatS& covariance, float32 bbox_height);
    DataM project(const VecS& mean, const MatS& covariance, float32 bbox_height);
    DataS update(const VecS& mean, const MatS& covariance,
                 const VecM& measurement, float32 bbox_height);
    Eigen::Matrix<float32, 1, Eigen::Dynamic>
        gating_distance(const VecS& mean, const MatS& covariance,
                        const std::vector<VecM>& measurements, float32 bbox_height);
};
```

## 核心流程

`KFKeypoint` 的核心流程与 `KFBox` 类似, 但提供两套噪声模型:

- **模式 A (固定模式)**: 使用构造时传入的 `p_noise` / `m_noise` 绝对像素值, 适合无高度上下文的场景
- **模式 B (自适应模式)**: 传入关联目标的 `bbox_height`, 噪声随高度动态缩放, 与 KFBox 设计对称

## 噪声模式对比

| 模式   | 参数                                           | 适用场景                           |
| ------ | ---------------------------------------------- | ---------------------------------- |
| 固定   | `p_noise`, `m_noise`                           | 无高度上下文, 或关键点独立于检测框 |
| 自适应 | `_std_weight_position`, `_std_weight_velocity` | 有关联检测框, 噪声随目标尺寸缩放   |

**调参指南**:

- 模式 A `p_noise` (过程噪声标准差):
  - 值大 -> 更相信新的检测结果, 轨迹灵敏但可能抖动
  - 值小 -> 更相信运动预测, 轨迹平滑但对快速运动反应慢
  - 建议: 正常人体关键点取 0.01~0.05, 快速运动可取到 0.1
- 模式 A `m_noise` (测量噪声标准差):
  - 值大 -> 更信任滤波器预测, 输出更平滑 (抗检测器抖动)
  - 值小 -> 更信任检测器输出, 对真实运动跟随更紧
  - 建议: 检测结果置信度低时可适当增大, 取 0.1~1.0
