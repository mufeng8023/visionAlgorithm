/***
 * @Author       : gxs
 * @Date         : 2026-07-19 15:00:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-19 15:00:00
 * @FilePath     : /visionAlgorithm/lib/tracker/BaseKalmanFilter.hpp
 * @Description  : 卡尔曼滤波器通用泛型基类 (Generic Kalman Filter Base Class)
 *                 通过 C++ 模板参数在编译期决定状态空间与观测空间的维度,
 *                 完全消除动态内存分配, 使用栈上的固定尺寸 Eigen 矩阵;
 *                 提供两类方法:
 *                   - 纯虚接口 (initiate / predict / project): 由子类各自实现;
 *                   - 通用默认实现 (update / gating_distance): 子类可按需覆盖;
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */

#ifndef __BASE_KALMAN_FILTER__H__
#define __BASE_KALMAN_FILTER__H__

#include <utility>  // std::pair, std::make_pair
#include <vector>

#include <Eigen/Cholesky>  // LLT Cholesky 分解, 用于稳定求解线性方程组
#include <Eigen/Core>      // Eigen 核心矩阵类型
#include <Eigen/Dense>     // Eigen 稠密矩阵运算

#include "types.hpp"  // float32, float64, int32 等基础类型

namespace tracker
{

// =====================================================================
// 卡尔曼滤波器泛型基类
//
// @template StateDim  : 状态空间维数 (Box 跟踪为 8, 关键点平滑为 4)
// @template MeasureDim: 观测空间维数 (Box 跟踪为 4, 关键点平滑为 2)
//
// 设计意图:
//   将各类跟踪场景公共的数学核心 (update / gating_distance) 收敛到基类,
//   把差异化的噪声建模 (initiate / predict / project) 下放到子类;
//   子类只需通过 get_update_matrix() 暴露自己的观测矩阵 H 即可;
// =====================================================================
template <int StateDim, int MeasureDim>
class BaseKalmanFilter
{
   public:
    // =================================================================
    // 内部类型别名 (Class-Internal Type Aliases)
    // 将 Eigen 冗长模板类型命名为短别名, 便于子类和外部使用
    // =================================================================

    // 状态行向量 (1 x StateDim): 存储目标的完整状态, 例如 [位置, 速度]
    using VecS = Eigen::Matrix<float32, 1, StateDim, Eigen::RowMajor>;

    // 状态方阵 (StateDim x StateDim): 协方差矩阵, 运动矩阵 F 等状态域方阵
    using MatS = Eigen::Matrix<float32, StateDim, StateDim, Eigen::RowMajor>;

    // 观测行向量 (1 x MeasureDim): 存储检测器输出的测量值, 例如 [cx, cy, a, h]
    using VecM = Eigen::Matrix<float32, 1, MeasureDim, Eigen::RowMajor>;

    // 观测方阵 (MeasureDim x MeasureDim): 观测协方差, 新息协方差 S 等测量域方阵
    using MatM = Eigen::Matrix<float32, MeasureDim, MeasureDim, Eigen::RowMajor>;

    // 观测矩阵 H (MeasureDim x StateDim): 将状态空间线性映射到观测空间
    // 例如 Box 跟踪中 H 提取前 4 维位置, 忽略后 4 维速度
    using MatH = Eigen::Matrix<float32, MeasureDim, StateDim, Eigen::RowMajor>;

    // 状态域数据对: {状态均值 VecS, 状态协方差 MatS}; 常见于接口返回值
    using DataS = std::pair<VecS, MatS>;

    // 观测域数据对: {观测均值 VecM, 观测协方差 MatM}; 常见于接口返回值
    using DataM = std::pair<VecM, MatM>;

   public:
    virtual ~BaseKalmanFilter() = default;

    // =================================================================
    // 纯虚接口: 子类必须实现的差异化部分
    // =================================================================

    /***
     * @description: 初始化新轨迹; 当目标第一次被检测到时调用;
     *               子类根据自身噪声模型构建合理的初始均值和协方差;
     * @param measurement 第一帧检测值 (VecM);
     * @return DataS {初始状态均值, 初始状态协方差};
     */
    virtual DataS initiate(const VecM& measurement) = 0;

    /***
     * @description: 预测步骤; 每帧在获取检测结果前调用;
     *               利用运动模型将状态向前推进一帧;
     * @param mean       [in/out] 进入时为旧状态均值, 返回时为预测的新均值;
     * @param covariance [in/out] 进入时为旧协方差矩阵, 返回时为预测的新协方差;
     */
    virtual void predict(VecS& mean, MatS& covariance) = 0;

    /***
     * @description: 投影步骤 (内部使用); 将状态空间的均值和协方差映射到观测空间;
     *               被基类的 update() 和 gating_distance() 内部调用;
     * @param mean       状态均值 VecS;
     * @param covariance 状态协方差 MatS;
     * @return DataM {预测的观测均值 VecM, 新息协方差 MatM (含测量噪声 R)};
     */
    virtual DataM project(const VecS& mean, const MatS& covariance) = 0;

    // =================================================================
    // 通用默认实现: 基类已提供的数学核心 (子类可按需覆盖)
    // =================================================================

    /***
     * @description: 通用卡尔曼更新步骤 (公式 3~7); 检测框与轨迹匹配成功后调用;
     *               用实际检测值修正预测值, 得到后验估计;
     *               内部调用 project() 获取 z_pred 和 S, 再通过 get_update_matrix()
     *               获取 H 矩阵, 计算卡尔曼增益 K 并更新均值与协方差;
     * @param mean        预测的状态均值 (来自 predict());
     * @param covariance  预测的状态协方差 (来自 predict());
     * @param measurement 实际检测到的测量值 VecM;
     * @return DataS {修正后的状态均值, 修正后的状态协方差};
     */
    virtual DataS update(const VecS& mean, const MatS& covariance, const VecM& measurement)
    {
        // 投影到观测空间, 获取预测测量值 z_pred 和新息协方差 S
        DataM pa = this->project(mean, covariance);
        VecM projected_mean = pa.first;
        MatM projected_cov = pa.second;

        // 计算新息 y = z - z_pred (实际观测值与预测值的差, 差越大说明预测越不准)
        VecM innovation = measurement - projected_mean;

        // 计算卡尔曼增益 K; 理论公式: K = P * H^T * S^{-1}
        // 等价于求解线性方程: S * K^T = H * P^T
        // B = (P * H^T)^T = H * P (利用协方差 P 的对称性)
        // 尺寸: (MeasureDim x StateDim) = ((StateDim x StateDim) @ (StateDim x MeasureDim))^T
        Eigen::Matrix<float32, MeasureDim, StateDim> B =
            (covariance * this->get_update_matrix().transpose()).transpose();

        // 用 Cholesky 分解求解 S * K^T = B, 比直接求逆矩阵更稳定高效
        // 解得 K^T, 再转置得到 K (StateDim x MeasureDim)
        Eigen::Matrix<float32, StateDim, MeasureDim> kalman_gain = projected_cov.llt().solve(B).transpose();

        // 后验均值更新: mean = mean' + K * y
        VecS new_mean = mean + innovation * kalman_gain.transpose();

        // 后验协方差更新: P = P' - K * S * K^T (引入测量后不确定度减小)
        MatS new_covariance = covariance - kalman_gain * projected_cov * kalman_gain.transpose();

        return std::make_pair(new_mean, new_covariance);
    }

    /***
     * @description: 通用马氏距离计算; 衡量预测位置与实际检测位置之间的归一化距离;
     *               马氏距离考虑了各维度的方差和相关性, 比欧氏距离更合理;
     *               常用于 DeepSORT 的门控过滤: 距离超过阈值则不允许匹配;
     * @param mean          预测的状态均值;
     * @param covariance    预测的状态协方差;
     * @param measurements  多个检测值的集合 std::vector<VecM>;
     * @return              每个检测值的马氏距离平方 (1 x N 矩阵);
     */
    virtual Eigen::Matrix<float32, 1, Eigen::Dynamic> gating_distance(const VecS& mean, const MatS& covariance,
                                                                      const std::vector<VecM>& measurements)
    {
        // 投影到观测空间, 获取预测测量值和新息协方差 S
        DataM pa = this->project(mean, covariance);
        VecM projected_mean = pa.first;
        MatM projected_cov = pa.second;

        // 构建差值矩阵 diff (N x MeasureDim), 每行对应一个检测值与预测值的差
        Eigen::Matrix<float32, Eigen::Dynamic, MeasureDim, Eigen::RowMajor> diff(
            static_cast<int32>(measurements.size()), MeasureDim);
        for (size_t i = 0; i < measurements.size(); i++)
        {
            diff.row(static_cast<int32>(i)) = measurements[i] - projected_mean;
        }

        // 用 Cholesky 分解计算马氏距离平方: d_i^2 = diff_i * S^{-1} * diff_i^T
        // S = L * L^T, 求解 X * L = diff (OnTheRight 三角回代, X = diff * L^{-1})
        // 因为 S 对称: S^{-1} = L^{-T} * L^{-1} = (L^{-1})^T * L^{-1}
        // 所以 d^2 = ||X_row||^2 = X_row * X_row^T = diff * L^{-1} * L^{-T} * diff^T = diff * S^{-1} * diff^T
        Eigen::Matrix<float32, MeasureDim, MeasureDim, Eigen::RowMajor> L_mat = projected_cov.llt().matrixL();

        // z 为 MeasureDim x N 矩阵, 每列对应一个检测值的归一化残差
        Eigen::Matrix<float32, Eigen::Dynamic, Eigen::Dynamic> z =
            L_mat.template triangularView<Eigen::Lower>().template solve<Eigen::OnTheRight>(diff).transpose();

        // 按列求平方和, 得到每个检测值的马氏距离平方 (1 x N)
        return z.array().square().colwise().sum();
    }

   protected:
    // =================================================================
    // 供基类通用算法 (update / gating_distance) 获取子类具体的观测矩阵 H
    // 接口设计: 子类通过覆盖此函数返回自身持有的 _update_mat 成员的引用
    // =================================================================
    virtual const MatH& get_update_matrix() const = 0;
};

// =====================================================================
// 全局泛型短别名 (消除外部调用时多层嵌套的长模板名称)
// =====================================================================

// 基类本身的短别名
template <int S, int M>
using KFBase = BaseKalmanFilter<S, M>;

// 指定维度的状态域数据对类型 {VecS, MatS}
template <int S, int M>
using KFState = typename BaseKalmanFilter<S, M>::DataS;

// 指定维度的观测域数据对类型 {VecM, MatM}
template <int S, int M>
using KFMeasure = typename BaseKalmanFilter<S, M>::DataM;

}  // namespace tracker

#endif  // !__BASE_KALMAN_FILTER__H__
