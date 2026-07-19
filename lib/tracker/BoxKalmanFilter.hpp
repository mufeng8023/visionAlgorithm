/***
 * @Author       : gxs
 * @Date         : 2026-07-19 15:00:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-19 15:00:00
 * @FilePath     : /visionAlgorithm/lib/tracker/BoxKalmanFilter.hpp
 * @Description  : Bounding Box 目标跟踪卡尔曼滤波器 (Box Tracker Kalman Filter)
 *                 继承自 KFBase<8, 4>, 实现恒定速度模型;
 *                 状态空间 8 维: [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
 *                 观测空间 4 维: [cx, cy, a, h]
 *                 适用于 SORT / ByteTrack / DeepSORT / OC_SORT 等所有基于检测框的跟踪器;
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */

#ifndef __BOX_KALMAN_FILTER__H__
#define __BOX_KALMAN_FILTER__H__

#include <cmath>  // std::sqrt 等数学函数
#include <vector>

#include <Eigen/Core>   // Eigen 核心矩阵类型
#include <Eigen/Dense>  // Eigen 稠密矩阵运算

#include "tracker/BaseKalmanFilter.hpp"  // 泛型基类

namespace tracker
{

// =====================================================================
// KFBox: Bounding Box 目标跟踪的卡尔曼滤波器
//
// 继承关系: KFBox -> BaseKalmanFilter<8, 4>
//   StateDim  = 8: [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
//   MeasureDim = 4: [cx, cy, a, h]
//
// 适用于 SORT / ByteTrack / DeepSORT / OC_SORT 等所有基于检测框的跟踪器;
//   update / gating_distance 复用基类通用实现, 无需在子类重写;
//
// 注意: 本类对象是无状态的 (轨迹状态存储在外部的 mean / covariance 中);
//       一个 KFBox 实例可以同时服务所有轨迹;
// =====================================================================
class KFBox : public KFBase<8, 4>
{
   public:
    // =================================================================
    // 将基类类型别名引入当前作用域, 避免书写冗长的 KFBase<8,4>::VecS
    // =================================================================
    using VecS = KFBase<8, 4>::VecS;    // 状态行向量 (1 x 8)
    using MatS = KFBase<8, 4>::MatS;    // 状态方阵 (8 x 8)
    using VecM = KFBase<8, 4>::VecM;    // 观测行向量 (1 x 4)
    using MatM = KFBase<8, 4>::MatM;    // 观测方阵 (4 x 4)
    using MatH = KFBase<8, 4>::MatH;    // 观测矩阵 H (4 x 8)
    using DataS = KFBase<8, 4>::DataS;  // 状态域数据对 {VecS, MatS}
    using DataM = KFBase<8, 4>::DataM;  // 观测域数据对 {VecM, MatM}

   private:
    // ---- 状态转移矩阵 F (_motion_mat) (8 x 8) ----
    //
    // 描述恒定速度运动模型: 新位置 = 旧位置 + 速度 * dt (dt = 1 帧)
    //
    //   [cx']   [1 0 0 0 | 1 0 0 0]   [cx  ]
    //   [cy']   [0 1 0 0 | 0 1 0 0]   [cy  ]
    //   [a' ] = [0 0 1 0 | 0 0 1 0] * [a   ]
    //   [h' ]   [0 0 0 1 | 0 0 0 1]   [h   ]
    //   [v_cx]  [0 0 0 0 | 1 0 0 0]   [v_cx]
    //   [v_cy]  [0 0 0 0 | 0 1 0 0]   [v_cy]
    //   [v_a ]  [0 0 0 0 | 0 0 1 0]   [v_a ]
    //   [v_h ]  [0 0 0 0 | 0 0 0 1]   [v_h ]
    Eigen::Matrix<float32, 8, 8, Eigen::RowMajor> _motion_mat;

    // ---- 观测矩阵 H (_update_mat) (4 x 8) ----
    //
    // 从 8 维状态中提取前 4 维位置, 忽略后 4 维速度:
    //
    //   [cx_meas]   [1 0 0 0 | 0 0 0 0]   [cx  ]
    //   [cy_meas] = [0 1 0 0 | 0 0 0 0] * [cy  ]
    //   [a_meas ]   [0 0 1 0 | 0 0 0 0]   [a   ]
    //   [h_meas ]   [0 0 0 1 | 0 0 0 0]   [h   ]
    //                                      [v_cx]
    //                                      ...
    Eigen::Matrix<float32, 4, 8, Eigen::RowMajor> _update_mat;

    // ---- 过程噪声标准差权重 ----
    //
    // _std_weight_position = 1/20 = 0.05
    //   位置过程噪声; 与目标高度 h 成正比;
    //   大目标运动时像素变化大, 小目标像素变化小, 按 h 缩放使噪声自适应;
    //
    // _std_weight_velocity = 1/160 = 0.00625
    //   速度过程噪声; 远小于位置噪声, 因为我们假设速度平滑变化;
    float32 _std_weight_position;
    float32 _std_weight_velocity;

   protected:
    /***
     * @description: 返回观测矩阵 H; 供基类 update() 和 gating_distance() 使用;
     * @return const MatH& : _update_mat 的常量引用 (4 x 8);
     */
    const MatH& get_update_matrix() const override { return this->_update_mat; }

   public:
    // =================================================================
    // 卡方分布 95% 置信区间表 (Chi-Squared Distribution 95th Percentile)
    //
    // 用于 DeepSORT 门控过滤: 当马氏距离平方 > chi2inv95[dim] 时拒绝匹配;
    // chi2inv95[4] = 9.4877 是我们 4 维测量空间最常用的阈值;
    // =================================================================
    static constexpr float64 chi2inv95[10] = {
        0.0,     // 0 自由度 (占位, 不使用)
        3.8415,  // 1 自由度
        5.9915,  // 2 自由度
        7.8147,  // 3 自由度
        9.4877,  // 4 自由度 <- 本类使用的阈值 (测量维度 MeasureDim = 4)
        11.070,  // 5 自由度
        12.592,  // 6 自由度
        14.067,  // 7 自由度
        15.507,  // 8 自由度
        16.919   // 9 自由度
    };

    /***
     * @description: 构造函数; 初始化状态转移矩阵 F, 观测矩阵 H 及噪声权重;
     *               这些矩阵在整个跟踪过程中保持不变, 一次初始化即可;
     */
    KFBox()
    {
        // 初始化状态转移矩阵 F: 先设为 8x8 单位阵, 再写入右上角速度耦合项
        // 右上角 4x4 子块 (_motion_mat(i, i+4) = 1.0) 表示位置 += 速度 * dt (dt=1)
        this->_motion_mat = Eigen::MatrixXf::Identity(8, 8);
        for (int32 i = 0; i < 4; i++)
        {
            this->_motion_mat(i, i + 4) = 1.0f;  // dt = 1.0 帧
        }

        // 初始化观测矩阵 H: 4x8 单位矩阵的前 4x4 块天然就是所需的 H
        // H = [I4 | 0] 提取前 4 维位置, 忽略后 4 维速度
        this->_update_mat = Eigen::MatrixXf::Identity(4, 8);

        // 噪声权重: 来自 ByteTrack / DeepSORT 论文的黄金参数
        this->_std_weight_position = 1.0f / 20.0f;   // = 0.05
        this->_std_weight_velocity = 1.0f / 160.0f;  // = 0.00625
    }

    ~KFBox() override = default;

    /***
     * @description: 初始化新轨迹; 当目标第一次被检测到时调用;
     *               初始速度设为 0 但给予大协方差, 让滤波器在后续帧快速收敛到真实速度;
     *               宽高比 a 的协方差极小 (1e-2), 因为第一帧的宽高比通常是准确的;
     * @param measurement 检测器输出的第一帧检测框 VecM [cx, cy, a, h] (1 x 4);
     * @return DataS {初始状态均值 (1 x 8), 初始状态协方差 (8 x 8)};
     */
    DataS initiate(const VecM& measurement) override
    {
        // 构建初始状态均值: 前 4 维为检测框位置, 后 4 维速度初始化为 0
        VecS mean;
        mean.block<1, 4>(0, 0) = measurement;  // [cx, cy, a, h] 来自检测器
        mean.block<1, 4>(0, 4).setZero();      // [v_cx, v_cy, v_a, v_h] = 0

        // 获取目标高度, 用于自适应缩放噪声 (大目标不确定度更大)
        float32 height = measurement(3);

        // 位置初始标准差: 倍数 2 给予适中的位置不确定度
        float32 std_pos[4] = {
            2.0f * this->_std_weight_position * height,  // cx 不确定度
            2.0f * this->_std_weight_position * height,  // cy 不确定度
            1e-2f,                                       // a  不确定度极小 (第一帧宽高比通常准确)
            2.0f * this->_std_weight_position * height   // h  不确定度
        };

        // 速度初始标准差: 倍数 10 给予很大的速度不确定度 (完全不知道初始速度)
        float32 std_vel[4] = {
            10.0f * this->_std_weight_velocity * height,  // v_cx 不确定度极大
            10.0f * this->_std_weight_velocity * height,  // v_cy 不确定度极大
            1e-5f,                                        // v_a  几乎为 0 (宽高比变化率基本为 0)
            10.0f * this->_std_weight_velocity * height   // v_h  不确定度极大
        };

        // 将 8 个标准差组装为行向量, 平方后得到方差, 再构建对角协方差矩阵
        VecS std_vec;
        std_vec.block<1, 4>(0, 0) = Eigen::Map<VecM>(std_pos);  // 位置标准差
        std_vec.block<1, 4>(0, 4) = Eigen::Map<VecM>(std_vel);  // 速度标准差

        // 协方差 = 各标准差平方构成的对角阵 (各维度独立, 无相关性)
        VecS var_vec = std_vec.array().square();  // 逐元素平方 -> 方差
        MatS covariance = var_vec.asDiagonal();   // (1 x 8) 转为 (8 x 8) 对角阵

        return std::make_pair(mean, covariance);
    }

    /***
     * @description: 预测步骤; 每帧在检测结果到来前对每条轨迹调用一次;
     *               公式 (1): mean' = F * mean    (恒定速度假设推进位置);
     *               公式 (2): P' = F * P * F^T + Q (协方差传播并叠加过程噪声);
     * @param mean       [in/out] 进入时为上一帧状态均值, 返回时为预测的新均值;
     * @param covariance [in/out] 进入时为上一帧协方差矩阵, 返回时为预测的新协方差;
     */
    void predict(VecS& mean, MatS& covariance) override
    {
        // 当前高度 (用于自适应缩放过程噪声)
        float32 height = mean(3);

        // 过程噪声标准差: 描述模型和现实之间的偏差 (目标可能加减速或转弯)
        float32 std_pos[4] = {
            this->_std_weight_position * height,  // cx 过程噪声
            this->_std_weight_position * height,  // cy 过程噪声
            1e-2f,                                // a  宽高比过程噪声 (几乎不变)
            this->_std_weight_position * height   // h  过程噪声
        };
        float32 std_vel[4] = {
            this->_std_weight_velocity * height,  // v_cx 过程噪声
            this->_std_weight_velocity * height,  // v_cy 过程噪声
            1e-5f,                                // v_a  宽高比变化率噪声 (极小)
            this->_std_weight_velocity * height   // v_h  过程噪声
        };

        // 组合为 8 维过程噪声向量并构建过程噪声协方差 Q (对角阵)
        VecS tmp;
        tmp.block<1, 4>(0, 0) = Eigen::Map<VecM>(std_pos);
        tmp.block<1, 4>(0, 4) = Eigen::Map<VecM>(std_vel);
        MatS motion_cov = tmp.array().square().matrix().asDiagonal();  // Q = diag(std^2)

        // 公式 (1): mean' = F * mean (对行向量做转置运算再转回)
        mean = (this->_motion_mat * mean.transpose()).transpose();

        // 公式 (2): P' = F * P * F^T + Q
        covariance = this->_motion_mat * covariance * this->_motion_mat.transpose() + motion_cov;
    }

    /***
     * @description: 投影步骤 (内部使用); 将 8 维状态空间投影到 4 维观测空间;
     *               被基类的 update() 和 gating_distance() 内部调用;
     *               公式: z_pred = H * mean, S = H * P * H^T + R;
     * @param mean       8 维状态均值 VecS;
     * @param covariance 8 x 8 状态协方差 MatS;
     * @return DataM {预测的观测均值 VecM [cx, cy, a, h], 新息协方差 MatM 4x4};
     */
    DataM project(const VecS& mean, const MatS& covariance) override
    {
        // 当前高度, 用于自适应缩放测量噪声 R
        float32 height = mean(3);

        // 测量噪声标准差 R: 描述检测器输出的边界框有多不准确
        // 注意宽高比 a 的噪声 (1e-1) 远大于过程噪声 (1e-2), 因为检测器对宽高比估计不稳定
        float32 std_meas[4] = {
            this->_std_weight_position * height,  // cx 检测误差
            this->_std_weight_position * height,  // cy 检测误差
            1e-1f,                                // a  宽高比检测误差 (较大, 检测器不太准)
            this->_std_weight_position * height   // h  检测误差
        };

        // z_pred = H * mean (提取前 4 维位置)
        VecM projected_mean = (this->_update_mat * mean.transpose()).transpose();

        // S = H * P * H^T + R
        MatM projected_cov = this->_update_mat * covariance * this->_update_mat.transpose();
        MatM R_diag = Eigen::Map<VecM>(std_meas).array().square().matrix().asDiagonal();
        projected_cov += R_diag;

        return std::make_pair(projected_mean, projected_cov);
    }
};

// =====================================================================
// 类型别名: BOX_* 前缀, 供 Box 跟踪相关代码直接使用;
// 命名规则与关键点跟踪 KP_* 系列对称, 便于在同一命名空间中区分;
// =====================================================================

// Box 跟踪状态均值向量 (1 x 8): [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
using BOX_MEAN = KFBox::VecS;

// Box 跟踪状态协方差矩阵 (8 x 8)
using BOX_COVA = KFBox::MatS;

// Box 跟踪观测均值向量 (1 x 4): [cx, cy, a, h]
using BOX_HMEAN = KFBox::VecM;

// Box 跟踪观测协方差矩阵 (4 x 4)
using BOX_HCOVA = KFBox::MatM;

// Box 跟踪状态域数据对: {BOX_MEAN, BOX_COVA}
using BOX_DATA = KFBox::DataS;

// Box 跟踪观测域数据对: {BOX_HMEAN, BOX_HCOVA}
using BOX_HDATA = KFBox::DataM;

}  // namespace tracker

#endif  // !__BOX_KALMAN_FILTER__H__
