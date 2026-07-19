/***
 * @Author       : gxs
 * @Date         : 2026-07-19 15:00:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-19 16:00:00
 * @FilePath     : /visionAlgorithm/lib/tracker/KeypointKalmanFilter.hpp
 * @Description  : 关键点 XY 坐标平滑卡尔曼滤波器 (Keypoint XY Kalman Filter)
 *                 继承自 KFBase<4, 2>, 实现恒定速度模型;
 *                 状态空间 4 维: [x, y, v_x, v_y]
 *                 观测空间 2 维: [x, y]
 *
 *  █ 与 KFBox 的设计对比
 *
 *   KFBox (8维/4维):
 *     状态: [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
 *     观测: [cx, cy, a, h]
 *     噪声: 与目标高度 h 自适应缩放 (大目标噪声大, 小目标噪声小)
 *
 *   KFKeypoint (4维/2维):
 *     状态: [x, y, v_x, v_y]
 *     观测: [x, y]
 *     噪声模式 A (固定模式): 使用绝对像素值 p_noise / m_noise, 场景无高度信息时使用;
 *     噪声模式 B (自适应模式): 传入关联目标的 bbox_height, 噪声随高度动态缩放,
 *                              大目标像素抖动大 -> 允许更大的不确定度,
 *                              小目标像素精度高 -> 约束更严格的不确定度;
 *
 *  █ 两种噪声模式的接口对比
 *
 *   模式 A (固定噪声, 无 bbox_height 参数):
 *     initiate(measurement)                        -- 固定协方差初始化
 *     predict(mean, cov)                           -- 固定 Q 过程噪声
 *     project(mean, cov)                           -- 固定 R 测量噪声
 *     update(mean, cov, meas)                      -- 基类通用实现 (内部调 project 固定版)
 *     gating_distance(mean, cov, meas_vec)         -- 基类通用实现 (内部调 project 固定版)
 *
 *   模式 B (高度自适应, 带 bbox_height 参数):
 *     initiate(measurement, bbox_height)           -- 噪声随 h 缩放
 *     predict(mean, cov, bbox_height)              -- Q 随 h 缩放
 *     project(mean, cov, bbox_height)              -- R 随 h 缩放
 *     update(mean, cov, meas, bbox_height)         -- 内部调用自适应 project
 *     gating_distance(mean, cov, meas_vec, h)      -- 内部调用自适应 project
 *
 *  █ 噪声参数调参指南
 *
 *   模式 A 参数 (绝对像素值):
 *     p_noise (过程噪声标准差):
 *       值大 -> 更相信新的检测结果, 轨迹灵敏但可能抖动;
 *       值小 -> 更相信运动预测, 轨迹平滑但对快速运动反应慢;
 *       建议: 正常人体关键点取 0.01~0.05, 快速运动可取到 0.1;
 *     m_noise (测量噪声标准差):
 *       值大 -> 更信任滤波器预测, 输出更平滑 (抗检测器抖动);
 *       值小 -> 更信任检测器输出, 对真实运动跟随更紧;
 *       建议: 检测结果置信度低时可适当增大, 取 0.1~1.0;
 *
 *   模式 B 参数 (与 KFBox 对称, 权重 x bbox_height):
 *     std_weight_pos (位置噪声权重, 默认 1/20 = 0.05):
 *       与 KFBox 的 _std_weight_position 含义一致;
 *       std = std_weight_pos * bbox_height (像素);
 *     std_weight_vel (速度噪声权重, 默认 1/160 = 0.00625):
 *       与 KFBox 的 _std_weight_velocity 含义一致;
 *       std = std_weight_vel * bbox_height (像素/帧);
 *
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */

#ifndef __KEYPOINT_KALMAN_FILTER__H__
#define __KEYPOINT_KALMAN_FILTER__H__

#include <vector>

#include <Eigen/Core>   // Eigen 核心矩阵类型
#include <Eigen/Dense>  // Eigen 稠密矩阵运算

#include "tracker/BaseKalmanFilter.hpp"  // 泛型基类

namespace tracker
{

// =====================================================================
// KFKeypoint: 关键点 XY 坐标平滑卡尔曼滤波器
//
// 继承关系: KFKeypoint -> BaseKalmanFilter<4, 2>
//   StateDim   = 4: [x, y, v_x, v_y]
//   MeasureDim = 2: [x, y]
//
// 适用场景:
//   - 姿态估计关键点平滑 (人体骨骼点, 手部关键点, 面部关键点等)
//   - 单点目标的位置跟踪与平滑
//   - 任何仅有 XY 二维坐标输入的平滑需求
//
// 设计特点:
//   - 固定噪声模式: 构造时传入 p_noise / m_noise, 适合无高度上下文的场景;
//   - 自适应噪声模式: 调用带 bbox_height 的重载接口, 噪声随目标尺寸动态缩放,
//     与 KFBox 的设计思路对称, 更好地区分大/中/小目标的不确定度;
//   - update / gating_distance 的两套版本均在本类中提供, 无需子类重写;
// =====================================================================
class KFKeypoint : public KFBase<4, 2>
{
   public:
    // =================================================================
    // 将基类类型别名引入当前作用域, 避免书写冗长的 KFBase<4,2>::VecS
    // =================================================================
    using VecS = KFBase<4, 2>::VecS;    // 状态行向量 (1 x 4): [x, y, v_x, v_y]
    using MatS = KFBase<4, 2>::MatS;    // 状态方阵 (4 x 4)
    using VecM = KFBase<4, 2>::VecM;    // 观测行向量 (1 x 2): [x, y]
    using MatM = KFBase<4, 2>::MatM;    // 观测方阵 (2 x 2)
    using MatH = KFBase<4, 2>::MatH;    // 观测矩阵 H (2 x 4)
    using DataS = KFBase<4, 2>::DataS;  // 状态域数据对 {VecS, MatS}
    using DataM = KFBase<4, 2>::DataM;  // 观测域数据对 {VecM, MatM}

   private:
    // ---- 状态转移矩阵 F (_motion_mat) (4 x 4) ----
    //
    // 恒定速度运动模型: x_new = x + v_x, y_new = y + v_y (dt = 1)
    //
    //   [x' ]   [1 0 | 1 0]   [x  ]
    //   [y' ] = [0 1 | 0 1] * [y  ]
    //   [v_x]   [0 0 | 1 0]   [v_x]
    //   [v_y]   [0 0 | 0 1]   [v_y]
    //          左上单位阵  右上 dt*I (dt=1)
    //          左下零阵    右下单位阵 (速度保持不变)
    Eigen::Matrix<float32, 4, 4, Eigen::RowMajor> _motion_mat;

    // ---- 观测矩阵 H (_update_mat) (2 x 4) ----
    //
    // 从 4 维状态中提取前 2 维坐标 [x, y], 忽略后 2 维速度 [v_x, v_y]:
    //
    //   [x_meas]   [1 0 | 0 0]   [x  ]
    //   [y_meas] = [0 1 | 0 0] * [y  ]
    //                             [v_x]
    //                             [v_y]
    Eigen::Matrix<float32, 2, 4, Eigen::RowMajor> _update_mat;

    // ---- 模式 A: 固定噪声参数 (绝对像素值) ----
    // 当调用无 bbox_height 参数的接口时生效;
    float32 _p_noise;  // 过程噪声标准差 (像素): 值越大轨迹越灵敏, 值越小越平滑;
    float32 _m_noise;  // 测量噪声标准差 (像素): 值越大越平滑 (抗检测抖动), 值越小越跟随检测;

    // ---- 模式 B: 自适应噪声权重 (噪声 = 权重 x bbox_height) ----
    // 与 KFBox 的 _std_weight_position / _std_weight_velocity 含义对称;
    // 当调用带 bbox_height 参数的接口时生效;
    float32 _std_weight_pos;  // 位置噪声权重 (默认 1/20 = 0.05, 与 KFBox 对称);
    float32 _std_weight_vel;  // 速度噪声权重 (默认 1/160 = 0.00625, 与 KFBox 对称);

   protected:
    /***
     * @description: 返回观测矩阵 H; 供基类 update() 和 gating_distance() 使用;
     * @return const MatH& : _update_mat 的常量引用 (2 x 4);
     */
    const MatH& get_update_matrix() const override { return this->_update_mat; }

   public:
    /***
     * @description: 构造函数; 初始化状态转移矩阵 F, 观测矩阵 H 及两套噪声参数;
     *               两套参数均带默认值, 可灵活选择使用固定模式或自适应模式;
     * @param p_noise        模式 A: 过程噪声标准差 (像素), 默认 0.03;
     * @param m_noise        模式 A: 测量噪声标准差 (像素), 默认 0.5;
     * @param std_weight_pos 模式 B: 位置噪声权重 (默认 0.05 = 1/20, 与 KFBox 对称);
     * @param std_weight_vel 模式 B: 速度噪声权重 (默认 0.00625 = 1/160, 与 KFBox 对称);
     */
    KFKeypoint(float32 p_noise = 0.03f,                //
               float32 m_noise = 0.5f,                 //
               float32 std_weight_pos = 1.0f / 20.0f,  //
               float32 std_weight_vel = 1.0f / 160.0f)
        : _p_noise(p_noise),                //
          _m_noise(m_noise),                //
          _std_weight_pos(std_weight_pos),  //
          _std_weight_vel(std_weight_vel)
    {
        // 初始化状态转移矩阵 F: 先设为 4x4 单位阵, 再写入右上角速度耦合项;
        // _motion_mat(0, 2) = 1 表示 x_new = x + v_x * 1 (dt=1);
        // _motion_mat(1, 3) = 1 表示 y_new = y + v_y * 1 (dt=1);
        this->_motion_mat = Eigen::Matrix<float32, 4, 4, Eigen::RowMajor>::Identity();
        this->_motion_mat(0, 2) = 1.0f;  // x += v_x * dt (dt=1)
        this->_motion_mat(1, 3) = 1.0f;  // y += v_y * dt (dt=1)

        // 初始化观测矩阵 H: 提取前 2 维坐标, 忽略后 2 维速度;
        // H = [I2 | 0(2x2)], 左半为 2x2 单位阵, 右半为零;
        this->_update_mat = Eigen::Matrix<float32, 2, 4, Eigen::RowMajor>::Zero();
        this->_update_mat(0, 0) = 1.0f;  // x_meas = x
        this->_update_mat(1, 1) = 1.0f;  // y_meas = y
    }

    ~KFKeypoint() override = default;

    // =================================================================
    // 模式 A: 固定噪声接口 (无 bbox_height 参数)
    // =================================================================

    /***
     * @description: [模式 A] 初始化新关键点轨迹; 当关键点第一次被检测到时调用;
     *               初始速度设为 0, 给予较大的速度不确定度以便快速收敛;
     *               坐标 (x, y) 的初始不确定度固定为 1.0 像素;
     * @param measurement 第一帧检测到的关键点坐标 VecM [x, y] (1 x 2);
     * @return DataS {初始状态均值 (1 x 4), 初始状态协方差 (4 x 4)};
     */
    DataS initiate(const VecM& measurement) override
    {
        // 构建初始状态均值: 前 2 维为检测坐标, 后 2 维速度初始化为 0;
        VecS mean;
        mean.block<1, 2>(0, 0) = measurement;  // [x, y] 来自检测器
        mean.block<1, 2>(0, 2).setZero();      // [v_x, v_y] = 0 (初始速度未知)

        // 构建初始协方差矩阵 (对角阵, 固定值);
        //   坐标分量 (x, y): 方差 = 1.0^2 = 1.0 (中等不确定度);
        //   速度分量 (v_x, v_y): 方差 = 10.0^2 = 100.0 (极大不确定度, 因为完全不知道初始速度);
        MatS covariance = MatS::Identity();
        covariance(2, 2) = 100.0f;  // v_x 方差极大, 快速收敛到真实速度
        covariance(3, 3) = 100.0f;  // v_y 方差极大, 快速收敛到真实速度

        return std::make_pair(mean, covariance);
    }

    /***
     * @description: [模式 A] 预测步骤; 每帧在检测结果到来前对每个关键点调用一次;
     *               过程噪声 Q 使用固定的 _p_noise 标准差 (各向同性);
     *               公式 (1): mean' = F * mean;
     *               公式 (2): P' = F * P * F^T + Q;
     * @param mean       [in/out] 上一帧状态均值 [x, y, v_x, v_y], 返回时为预测新均值;
     * @param covariance [in/out] 上一帧协方差矩阵, 返回时为预测的新协方差;
     */
    void predict(VecS& mean, MatS& covariance) override
    {
        // 过程噪声协方差 Q: 各状态分量独立, 标准差均为 _p_noise (各向同性);
        float32 p_var = this->_p_noise * this->_p_noise;  // 方差 = 标准差^2
        MatS Q = MatS::Identity() * p_var;                // Q = p_noise^2 * I4

        // 公式 (1): mean' = F * mean (对行向量做转置运算再转回);
        mean = (this->_motion_mat * mean.transpose()).transpose();

        // 公式 (2): P' = F * P * F^T + Q;
        covariance = this->_motion_mat * covariance * this->_motion_mat.transpose() + Q;
    }

    /***
     * @description: [模式 A] 投影步骤 (内部使用); 将 4 维状态空间投影到 2 维观测空间;
     *               被基类的 update() 和 gating_distance() 内部调用;
     *               测量噪声 R 使用固定的 _m_noise 标准差;
     *               公式: z_pred = H * mean, S = H * P * H^T + R;
     * @param mean       4 维状态均值 VecS [x, y, v_x, v_y];
     * @param covariance 4 x 4 状态协方差 MatS;
     * @return DataM {预测的观测均值 VecM [x, y], 新息协方差 MatM 2x2};
     */
    DataM project(const VecS& mean, const MatS& covariance) override
    {
        // 测量噪声协方差 R: 各坐标分量的检测误差相同, 且独立 (固定模式);
        float32 m_var = this->_m_noise * this->_m_noise;  // 方差 = 标准差^2
        MatM R = MatM::Identity() * m_var;                // R = m_noise^2 * I2

        // z_pred = H * mean (提取前 2 维坐标 [x, y]);
        VecM projected_mean = (this->_update_mat * mean.transpose()).transpose();

        // S = H * P * H^T + R (新息协方差 = 预测不确定度 + 测量不确定度);
        MatM projected_cov = this->_update_mat * covariance * this->_update_mat.transpose() + R;

        return std::make_pair(projected_mean, projected_cov);
    }

    // =================================================================
    // 模式 B: 高度自适应噪声接口 (带 bbox_height 参数)
    //
    // 核心思想 (与 KFBox 对称):
    //   关键点所属目标的 bbox_height 越大 (大目标):
    //     - 关键点的绝对像素抖动也越大 -> 允许更大的过程/测量噪声;
    //   bbox_height 越小 (小目标):
    //     - 像素级别的精度更高 -> 用更小的噪声约束, 避免过度平滑;
    //
    // 公式: std = _std_weight_pos * bbox_height (位置)
    //            _std_weight_vel * bbox_height (速度)
    // =================================================================

    /***
     * @description: [模式 B] 带高度自适应噪声的初始化接口;
     *               初始协方差随 bbox_height 缩放, 大目标给予更宽松的初始不确定度;
     *               设计与 KFBox::initiate() 对称:
     *                 位置: std = 2.0 * _std_weight_pos * h (倍数 2 给予适中的初始不确定度);
     *                 速度: std = 10.0 * _std_weight_vel * h (倍数 10 反映完全不知道初始速度);
     * @param measurement 第一帧检测到的关键点坐标 VecM [x, y] (1 x 2);
     * @param bbox_height 关联目标的 bounding box 高度 (像素), 用于自适应缩放噪声;
     * @return DataS {初始状态均值 (1 x 4), 初始状态协方差 (4 x 4)};
     */
    DataS initiate(const VecM& measurement, const float32 bbox_height)
    {
        // 构建初始状态均值: 前 2 维为检测坐标, 后 2 维速度初始化为 0;
        VecS mean;
        mean.block<1, 2>(0, 0) = measurement;  // [x, y] 来自检测器
        mean.block<1, 2>(0, 2).setZero();      // [v_x, v_y] = 0 (初始速度未知)

        // 位置初始标准差 (与目标高度成正比, 倍数 2 对应适中的初始不确定度);
        float32 std_x = 2.0f * this->_std_weight_pos * bbox_height;  // x 初始标准差
        float32 std_y = 2.0f * this->_std_weight_pos * bbox_height;  // y 初始标准差

        // 速度初始标准差 (倍数 10 反映完全不知道初始速度方向和大小);
        float32 std_vx = 10.0f * this->_std_weight_vel * bbox_height;  // v_x 初始标准差
        float32 std_vy = 10.0f * this->_std_weight_vel * bbox_height;  // v_y 初始标准差

        // 构建对角协方差矩阵 (方差 = 标准差^2);
        MatS covariance = MatS::Zero();
        covariance(0, 0) = std_x * std_x;    // x  方差
        covariance(1, 1) = std_y * std_y;    // y  方差
        covariance(2, 2) = std_vx * std_vx;  // v_x 方差 (极大)
        covariance(3, 3) = std_vy * std_vy;  // v_y 方差 (极大)

        return std::make_pair(mean, covariance);
    }

    /***
     * @description: [模式 B] 带高度自适应噪声的预测步骤;
     *               过程噪声 Q 随 bbox_height 动态缩放, 与 KFBox::predict() 对称;
     *               公式 (1): mean' = F * mean;
     *               公式 (2): P' = F * P * F^T + Q (Q 随 h 缩放);
     * @param mean        [in/out] 上一帧状态均值, 返回时为预测新均值;
     * @param covariance  [in/out] 上一帧协方差矩阵, 返回时为预测的新协方差;
     * @param bbox_height 关联目标的 bounding box 高度 (像素);
     */
    void predict(VecS& mean, MatS& covariance, const float32 bbox_height)
    {
        // 过程噪声标准差: 随目标高度缩放 (大目标运动时像素变化大);
        float32 std_pos = this->_std_weight_pos * bbox_height;  // 位置过程噪声标准差
        float32 std_vel = this->_std_weight_vel * bbox_height;  // 速度过程噪声标准差

        // 构建过程噪声协方差 Q (对角阵, 各维度独立);
        MatS Q = MatS::Zero();
        Q(0, 0) = std_pos * std_pos;  // x  过程噪声方差
        Q(1, 1) = std_pos * std_pos;  // y  过程噪声方差
        Q(2, 2) = std_vel * std_vel;  // v_x 过程噪声方差
        Q(3, 3) = std_vel * std_vel;  // v_y 过程噪声方差

        // 公式 (1): mean' = F * mean;
        mean = (this->_motion_mat * mean.transpose()).transpose();

        // 公式 (2): P' = F * P * F^T + Q;
        covariance = this->_motion_mat * covariance * this->_motion_mat.transpose() + Q;
    }

    /***
     * @description: [模式 B] 带高度自适应噪声的投影步骤;
     *               测量噪声 R 随 bbox_height 动态缩放, 与 KFBox::project() 对称;
     *               公式: z_pred = H * mean, S = H * P * H^T + R (R 随 h 缩放);
     * @param mean        4 维状态均值 VecS [x, y, v_x, v_y];
     * @param covariance  4 x 4 状态协方差 MatS;
     * @param bbox_height 关联目标的 bounding box 高度 (像素);
     * @return DataM {预测的观测均值 VecM [x, y], 新息协方差 MatM 2x2};
     */
    DataM project(const VecS& mean, const MatS& covariance, const float32 bbox_height)
    {
        // 测量噪声标准差: 与目标高度成正比 (大目标检测误差的绝对像素值也大);
        float32 std_meas = this->_std_weight_pos * bbox_height;  // 测量噪声标准差

        // 测量噪声协方差 R (对角阵, x / y 检测误差相同且独立);
        MatM R = MatM::Zero();
        R(0, 0) = std_meas * std_meas;  // x 检测误差方差
        R(1, 1) = std_meas * std_meas;  // y 检测误差方差

        // z_pred = H * mean (提取前 2 维坐标 [x, y]);
        VecM projected_mean = (this->_update_mat * mean.transpose()).transpose();

        // S = H * P * H^T + R (新息协方差 = 预测不确定度 + 测量不确定度);
        MatM projected_cov = this->_update_mat * covariance * this->_update_mat.transpose() + R;

        return std::make_pair(projected_mean, projected_cov);
    }

    /***
     * @description: [模式 B] 带高度自适应噪声的卡尔曼更新步骤;
     *               内部调用带 bbox_height 的 project() 重载版本获取自适应的新息协方差 S;
     *               数学流程与基类 update() 完全相同, 仅噪声来源不同;
     * @param mean        预测的状态均值 (来自带 h 的 predict());
     * @param covariance  预测的状态协方差 (来自带 h 的 predict());
     * @param measurement 实际检测到的关键点坐标 VecM [x, y];
     * @param bbox_height 关联目标的 bounding box 高度 (像素);
     * @return DataS {修正后的状态均值, 修正后的状态协方差};
     */
    DataS update(const VecS& mean, const MatS& covariance, const VecM& measurement, const float32 bbox_height)
    {
        // 使用带高度的 project() 获取自适应的投影结果 (z_pred, S);
        DataM pa = this->project(mean, covariance, bbox_height);
        VecM projected_mean = pa.first;
        MatM projected_cov = pa.second;

        // 新息 y = z - z_pred (实际观测值与预测值的差);
        VecM innovation = measurement - projected_mean;

        // 计算辅助矩阵 B = (P * H^T)^T = H * P, 尺寸 (MeasureDim x StateDim) = (2 x 4);
        Eigen::Matrix<float32, 2, 4, Eigen::RowMajor> B =
            (covariance * this->get_update_matrix().transpose()).transpose();

        // 用 Cholesky 分解求解 S * K^T = B, 解得卡尔曼增益 K 尺寸 (4 x 2);
        Eigen::Matrix<float32, 4, 2, Eigen::RowMajor> kalman_gain = projected_cov.llt().solve(B).transpose();

        // 后验均值: mean = mean' + K * y;
        VecS new_mean = mean + innovation * kalman_gain.transpose();

        // 后验协方差: P = P' - K * S * K^T (引入测量后不确定度减小);
        MatS new_covariance = covariance - kalman_gain * projected_cov * kalman_gain.transpose();

        return std::make_pair(new_mean, new_covariance);
    }

    /***
     * @description: [模式 B] 带高度自适应噪声的马氏距离计算;
     *               内部调用带 bbox_height 的 project() 重载版本获取自适应的新息协方差 S;
     *               数学流程与基类 gating_distance() 完全相同, 仅噪声来源不同;
     * @param mean         预测的状态均值;
     * @param covariance   预测的状态协方差;
     * @param measurements 多个检测值的集合 std::vector<VecM>;
     * @param bbox_height  关联目标的 bounding box 高度 (像素);
     * @return             每个检测值的马氏距离平方 (1 x N 矩阵);
     */
    Eigen::Matrix<float32, 1, Eigen::Dynamic> gating_distance(const VecS& mean, const MatS& covariance,
                                                              const std::vector<VecM>& measurements,
                                                              const float32 bbox_height)
    {
        // 使用带高度的 project() 获取自适应投影结果;
        DataM pa = this->project(mean, covariance, bbox_height);
        VecM projected_mean = pa.first;
        MatM projected_cov = pa.second;

        // 构建差值矩阵 diff (N x 2), 每行为一个检测值与预测值的差;
        Eigen::Matrix<float32, Eigen::Dynamic, 2, Eigen::RowMajor> diff(static_cast<int32>(measurements.size()), 2);
        for (size_t i = 0; i < measurements.size(); i++)
        {
            diff.row(static_cast<int32>(i)) = measurements[i] - projected_mean;
        }

        // Cholesky 分解计算马氏距离平方: d_i^2 = diff_i * S^{-1} * diff_i^T;
        // S = L * L^T, 解 X * L = diff (三角回代), d^2 = ||X_row||^2;
        Eigen::Matrix<float32, 2, 2, Eigen::RowMajor> L_mat = projected_cov.llt().matrixL();
        Eigen::Matrix<float32, Eigen::Dynamic, Eigen::Dynamic> z =
            L_mat.template triangularView<Eigen::Lower>().template solve<Eigen::OnTheRight>(diff).transpose();

        // 按列求平方和, 得到每个检测值的马氏距离平方 (1 x N);
        return z.array().square().colwise().sum();
    }
};

// =====================================================================
// 关键点滤波器专用类型别名 (KP_* 前缀, 区别于 Box 的 BOX_*)
// =====================================================================

// 关键点状态均值向量 (1 x 4): [x, y, v_x, v_y]
using KP_MEAN = KFKeypoint::VecS;

// 关键点状态协方差矩阵 (4 x 4)
using KP_COVA = KFKeypoint::MatS;

// 关键点观测均值向量 (1 x 2): [x, y]
using KP_HMEAN = KFKeypoint::VecM;

// 关键点观测协方差矩阵 (2 x 2)
using KP_HCOVA = KFKeypoint::MatM;

// 关键点状态域数据对: {KP_MEAN, KP_COVA}
using KP_DATA = KFKeypoint::DataS;

// 关键点观测域数据对: {KP_HMEAN, KP_HCOVA}
using KP_HDATA = KFKeypoint::DataM;

}  // namespace tracker

#endif  // !__KEYPOINT_KALMAN_FILTER__H__
