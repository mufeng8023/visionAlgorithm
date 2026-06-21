/***
 * @Author       : gxs
 * @Date         : 2026-06-21 17:47:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-21 17:47:00
 * @FilePath     : /visionAlgorithm/lib/tracker/KalmanFilter.hpp
 * @Description  : 卡尔曼滤波器实现 (Kalman Filter Implementation)
 *                 本文件实现了 SORT / DeepSORT / ByteTrack / OC_SORT 通用的
 *                 恒定速度 (Constant Velocity) 卡尔曼滤波器;
 *                 状态空间为 8 维, 测量空间为 4 维;
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */

/***
 *  NOTE: 卡尔曼滤波器
 *
 *  █ 这个类到底用来干什么的?
 *
 *  在目标跟踪中, 检测器 (YOLO) 每帧会输出一堆检测框 (bounding box)。
 *  但这些检测框可能有噪声: 比如目标被遮挡时检测框会抖动, 或者偶尔漏检。
 *
 *  卡尔曼滤波器的作用就是:
 *    "根据目标之前的位置, 预测它现在应该在哪里,
 *     然后用检测器实际检测到的位置来修正这个预测,
 *     最终得到一个比"纯检测"更稳定的跟踪结果。"
 *
 *  你可以把它想象成一个"带脑子的平滑器"。
 *
 *  █ 核心思想: 预测 + 更新 = 最优估计
 *
 *  每一帧, 卡尔曼滤波器做两件事:
 *
 *  第 1 步 - 预测 (Predict):
 *    根据上一帧的目标位置和速度, 猜测这一帧目标大概在哪里。
 *    就像你看到一个人朝东走, 你会预测他下一秒还在东边。
 *    这时我们得到一个"先验估计" (先猜的, 不一定准)。
 *
 *  第 2 步 - 更新 (Update):
 *    用检测器实际检测到的位置来修正刚才的预测。
 *    你预测他在东边 10 米, 但检测器说他其实在东边 8 米,
 *    于是你取个折中: 东边 9 米 (或者更信任检测器一些)。
 *    这时我们得到一个"后验估计" (修正后的, 更准)。
 *
 *  █ 状态向量 (8 维) —— 我们到底在估计什么?
 *
 *  我们用 8 个数字来描述一个目标的状态:
 *
 *    mean = [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
 *            ├──── 位置 (4个) ────┤├── 速度 (4个) ───┤
 *
 *  位置部分 (你直接关心的):
 *    mean[0] = cx : 目标框的中心点 x 坐标 (像素)
 *    mean[1] = cy : 目标框的中心点 y 坐标 (像素)
 *    mean[2] = a  : 宽高比 (aspect ratio = 宽度 / 高度)
 *                    为什么用宽高比不用宽度?
 *                    因为人转身时宽高比基本不变, 但宽度会变。
 *    mean[3] = h  : 目标框的高度 (像素)
 *
 *  速度部分 (你猜的):
 *    mean[4] = v_cx : 中心点 x 方向的速度 (像素/帧)
 *    mean[5] = v_cy : 中心点 y 方向的速度 (像素/帧)
 *    mean[6] = v_a  : 宽高比的变化率
 *    mean[7] = v_h  : 高度的变化率
 *
 *  速度是"隐变量": 检测器不直接告诉你速度, 卡尔曼滤波器自己学会的。
 *
 *  █ 测量向量 (4 维) —— 检测器给了我们什么?
 *
 *  检测器只告诉我们位置, 不告诉我们速度:
 *
 *    measurement = [cx, cy, a, h]
 *
 *  所以我们只能拿这 4 个值去修正预测。速度怎么来的?
 *  卡尔曼滤波器通过"速度 = (新位置 - 旧位置) / 时间"的原理自动推算。
 *
 *  █ 协方差矩阵 (8x8) —— 我们有多相信自己的估计?
 *
 *  均值 mean 是"猜测的位置", 协方差 covariance 是"这个猜测有多不靠谱"。
 *  协方差越大, 说明我们越不确定; 越小, 说明我们越确定。
 *  比如刚初始化时, 速度的协方差特别大, 因为我们完全不知道速度。
 *
 *  █ 五个核心公式 —— 魔法发生的地方
 *
 *  以下公式中带 ' 的是"预测的", 不带的是"修正后的":
 *
 *  预测 (Predict):
 *    (1) x'(k) = F * x(k-1)       ← 用上一帧的状态猜这一帧
 *    (2) P'(k) = F * P(k-1) * F^T + Q  ← 猜的过程中不确定度增加了
 *
 *  更新 (Update):
 *    (3) y(k)  = z(k) - H * x'(k)    ← 检测值和预测值差多少? (新息)
 *    (4) S(k)  = H * P'(k) * H^T + R ← 这个差值有多可信?
 *    (5) K(k)  = P'(k) * H^T * S^(-1) ← 应该信预测还是信检测? (卡尔曼增益)
 *    (6) x(k)  = x'(k) + K * y(k)    ← 用检测值修正预测
 *    (7) P(k)  = (I - K * H) * P'(k) ← 修正后不确定度变小了
 *
 *  别怕! 看不懂公式没关系, 每个函数里面都有逐行解释。
 *  简单来说就是:
 *    "预测位置 -> 看检测器实际位置 -> 如果检测器靠谱就多信它,
 *     如果检测器不靠谱就多信自己的预测 -> 得到最终位置"
 *
 * =====================================================================
 *  各跟踪器使用差异
 *
 *  无论你用 SORT / ByteTrack / DeepSORT / OC_SORT,
 *  卡尔曼滤波器的数学计算完全一样! 区别在于:
 *
 *  ┌────────────┬──────────────────────────────────────────┐
 *  │ Tracker    │ 怎么用                                   │
 *  ├────────────┼──────────────────────────────────────────┤
 *  │ SORT       │ 只用 predict + update, 然后用 IoU 匹配    │
 *  │ ByteTrack  │ 同 SORT, 只是匹配时用了"高低分"两次关联    │
 *  │ DeepSORT   │ 除了 predict + update, 还调用             │
 *  │            │ gating_distance() 用马氏距离过滤匹配       │
 *  │ OC_SORT    │ 在 ByteTrack 上加了观测置信度修正          │
 *  └────────────┴──────────────────────────────────────────┘
 *
 *  gating_distance() 是什么?
 *  它是 DeepSORT 里用的"门控": 计算马氏距离,
 *  如果检测框离预测位置太远 (超出统计门限), 就不让它匹配。
 *  相当于"先筛选一遍, 再把剩下的拿去匹配"。
 *
 *  ByteTrack 和 SORT 不用这个, 因为它们只用 IoU 匹配,
 *  IoU 小的自然就被排除了, 不需要额外的门控。
 *
 * =====================================================================
 *  关键参数 (调参侠必看)
 * =====================================================================
 *
 *  _std_weight_position = 1/20 ≈ 0.05
 *    控制"位置噪声"的大小。值越大, 滤波器越相信检测器(轨迹更灵活但更抖)。
 *    值越小, 滤波器越相信自己的预测(轨迹更平滑但反应慢)。
 *
 *  _std_weight_velocity = 1/160 ≈ 0.00625
 *    控制"速度噪声"的大小。值越大, 速度更新越快(但对噪声更敏感)。
 *    值越小, 速度越平滑(但对突然转向反应慢)。
 *
 *  这两个参数为什么和高度 h 有关?
 *    目标越大 (h 越大), 它运动的不确定性也越大。
 *    一个 200 像素高的人, 位置偏差 10 像素很正常;
 *    但一个 50 像素高的小目标, 偏差 10 像素就很离谱了。
 *    所以噪声 = weight * height, 自适应。
 *
 * =====================================================================
 */

#ifndef __KALMANFILTER__H__
#define __KALMANFILTER__H__

#include <cmath>   // 数学函数
#include <vector>  // std::vector 用于存储多个检测框

#include <Eigen/Cholesky>  // Eigen 的 LLT 分解 (Cholesky 分解)
                           // 用于求解线性方程组, 比直接求逆矩阵更稳定
#include <Eigen/Core>      // Eigen 核心库 (Matrix, Array 等)
#include <Eigen/Dense>     // Eigen 稠密矩阵库

#include "types.hpp"  // uint8, float32, float64 等基础类型定义

// =====================================================================
// 命名空间: tracker
// 注意! 这里用的是 tracker, 不是 detector 模块的 yolo 命名空间。
// 这样做的原因是跟踪模块和检测模块是独立的, 避免命名冲突。
// =====================================================================
namespace tracker
{

// =====================================================================
// 卡尔曼滤波器的 Eigen 类型别名
//
// 为什么要定义这些别名?
// 因为 Eigen 的模板参数很长 (比如 Matrix<float, 1, 8, RowMajor>),
// 每次写都很麻烦, 所以用 using 起个短名字。
//
// 这些名字和 DeepSORT 官方代码保持一致, 方便对照:
//
//   KAL_MEAN   = 1行8列  = 状态均值向量
//   KAL_COVA   = 8行8列  = 状态协方差矩阵
//   KAL_HMEAN  = 1行4列  = 观测均值向量 (H 是 observation 的意思)
//   KAL_HCOVA  = 4行4列  = 观测协方差矩阵
//   KAL_DATA   = 状态 {均值, 协方差} 的对
//   KAL_HDATA  = 观测 {均值, 协方差} 的对 (投影到观测空间后的)
// =====================================================================

// KAL_MEAN: 状态均值向量, 1 行 8 列, 行主序 (RowMajor) 存储
//   行主序: 同一行的数据在内存中是连续的, 访问效率高
//   下标说明:
//     [0]=cx (中心x),  [1]=cy (中心y),  [2]=a (宽高比),  [3]=h (高度)
//     [4]=v_cx (x速度), [5]=v_cy (y速度), [6]=v_a (宽高比变化率), [7]=v_h (高度变化率)
using KAL_MEAN = Eigen::Matrix<float32, 1, 8, Eigen::RowMajor>;

// KAL_COVA: 状态协方差矩阵, 8 行 8 列
//   对角线上的值表示每个状态分量的不确定度 (方差)
//   非对角线上的值表示两个分量之间的相关性
//   例如 cov[0][4] > 0 表示 cx 和 v_cx 正相关 (cx 大时 v_cx 也倾向于大)
using KAL_COVA = Eigen::Matrix<float32, 8, 8, Eigen::RowMajor>;

// KAL_HMEAN: 观测均值向量, 1 行 4 列 [cx, cy, a, h]
//   注意! 观测值只有位置, 没有速度
using KAL_HMEAN = Eigen::Matrix<float32, 1, 4, Eigen::RowMajor>;

// KAL_HCOVA: 观测协方差矩阵, 4 行 4 列
//   表示测量空间中各分量的不确定度
using KAL_HCOVA = Eigen::Matrix<float32, 4, 4, Eigen::RowMajor>;

// KAL_DATA: 用 std::pair 把均值和协方差打包在一起
//   first  = KAL_MEAN (状态均值)
//   second = KAL_COVA (状态协方差)
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;

// KAL_HDATA: 观测空间下的 {均值, 协方差} 对
//   first  = KAL_HMEAN (观测均值)
//   second = KAL_HCOVA (观测协方差)
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

// =====================================================================
// 卡尔曼滤波器类
//
// 这个类实现了恒定速度 (Constant Velocity) 模型的卡尔曼滤波器。
// 所谓"恒定速度", 就是我们假设目标的速度不会突然变化,
// 如果目标突然转向或急停, 滤波器会通过"更新步骤"来适应。
//
// 用法:
//   1. 创建 KalmanFilter 对象 (每个视频序列一个, 不是每个目标一个!)
//   2. 当新目标出现时, 调用 initiate() 创建初始状态
//   3. 每帧先对所有轨迹调用 predict() 预测新位置
//   4. 匹配成功后, 对匹配上的轨迹调用 update() 修正位置
//
// 注意: KalmanFilter 对象本身是无状态的! 状态存储在每条轨迹的
//       mean 和 covariance 中。所以一个 KalmanFilter 可以服务所有轨迹。
//       就像一把尺子可以量所有东西, 不需要每个东西配一把尺子。
// =====================================================================
class KalmanFilter
{
   private:
    // ---- 状态转移矩阵 F (8x8) ----
    //
    // 这个矩阵描述了"目标在 1 帧内会怎么运动"。
    // 它把"旧状态"映射到"新状态":
    //
    //   新状态 = F * 旧状态
    //
    // 具体来说, F 是这样的:
    //
    //   [ 位置分量 ]   [ I(4x4) | dt*I(4x4) ]   [ 旧位置 ]
    //   [ 速度分量 ] = [ 0(4x4) | I(4x4)    ] * [ 旧速度 ]
    //
    // 翻译成人话:
    //   新位置 = 旧位置 + 旧速度 * dt      (位置因速度而改变)
    //   新速度 = 旧速度                    (速度保持不变, 恒定速度假设)
    //
    // 其中 dt = 1 (帧间隔为 1 帧)。
    // 这就是"恒定速度模型"的含义。
    // =================================================================
    Eigen::Matrix<float32, 8, 8, Eigen::RowMajor> _motion_mat;

    // ---- 观测矩阵 H (4x8) ----
    //
    // 这个矩阵描述了"如何从状态中提取出我们能观测到的部分"。
    // 我们的状态是 8 维 [位置, 速度], 但我们只能观测到 4 维 [位置]。
    //
    // H 的功能就是: 从 8 维状态中取出前 4 维 (位置), 忽略后 4 维 (速度)。
    //
    //   [cx_观测]   [1 0 0 0 0 0 0 0]   [cx]
    //   [cy_观测] = [0 1 0 0 0 0 0 0] * [cy]
    //   [a_观测 ]   [0 0 1 0 0 0 0 0]   [a ]
    //   [h_观测 ]   [0 0 0 1 0 0 0 0]   [h ]
    //                                    [v_cx]
    //                                    [v_cy]
    //                                    [v_a ]
    //                                    [v_h ]
    //
    // 为什么叫 "update_mat"? 因为它在更新步骤 (update) 中使用。
    // =================================================================
    Eigen::Matrix<float32, 4, 8, Eigen::RowMajor> _update_mat;

    // ---- 过程噪声标准差权重 ----
    //
    // 这两个参数控制了"模型有多不准确"。
    // 即使我们假设目标恒定速度运动, 实际目标可能会加速/减速/转弯。
    // 这种"模型和现实之间的差距"就是过程噪声。
    //
    // _std_weight_position = 1/20 ≈ 0.05
    //   位置过程噪声: 我们认为每帧的位置不确定度增加 ≈ 5% * 高度。
    //   比如一个 100 像素高的目标, 每帧位置不确定度增加 5 像素。
    //
    // _std_weight_velocity = 1/160 ≈ 0.00625
    //   速度过程噪声: 我们认为速度的变化 ≈ 0.6% * 高度。
    //   速度噪声比位置噪声小得多, 因为我们假设目标是平滑运动的。
    //
    // 为什么用高度 h 来缩放?
    //   大目标 (近处) 运动时像素变化大, 小目标 (远处) 像素变化小。
    //   用高度缩放后, 大小目标的噪声水平就一致了。
    // =================================================================
    float32 _std_weight_position;
    float32 _std_weight_velocity;

   public:
    // =================================================================
    // 卡方分布 95% 置信区间表
    //
    // 这是什么? 有什么用?
    //
    // 统计学中, 卡方分布用来判断"一个观测值是否异常"。
    // 比如自由度为 4 时, 卡方值 9.4877 表示:
    //   "如果马氏距离平方 < 9.4877, 则有 95% 的把握认为它是正常匹配"
    //   "如果马氏距离平方 > 9.4877, 则只有 5% 的可能它是正常匹配"
    //
    // 在 DeepSORT 中, 我们用它来判断"这个检测框是不是该分配给这个轨迹"。
    //
    // 例如: 轨迹预测目标在 (100, 100), 检测框在 (500, 500)。
    //       马氏距离平方可能很大 > 9.4877, 于是判定"距离太远, 不匹配"。
    //
    // 下标说明:
    //   chi2inv95[4] = 9.4877: 自由度为 4 (我们的测量是 4 维)
    // =================================================================
    static constexpr float64 chi2inv95[10] = {
        0.0,     // 0 自由度 (占位, 不使用)
        3.8415,  // 1 自由度
        5.9915,  // 2 自由度
        7.8147,  // 3 自由度
        9.4877,  // 4 自由度 ← 我们用的就是这个! 测量维度是 4
        11.070,  // 5 自由度
        12.592,  // 6 自由度
        14.067,  // 7 自由度
        15.507,  // 8 自由度
        16.919   // 9 自由度
    };

    /***
     * @description: 构造函数
     * 做三件事:
     *  1. 初始化状态转移矩阵 F (_motion_mat)
     *  2. 初始化观测矩阵 H (_update_mat)
     *  3. 设置噪声权重
     * 这些都是在对象创建时一次性完成的, 矩阵的内容不会改变。
     * @return
     */
    KalmanFilter()
    {
        // ---------------------------------------------------------
        // 第 1 件事: 初始化状态转移矩阵 F (_motion_mat)
        // ---------------------------------------------------------
        //
        // 我们想要的效果:
        //   新位置 = 旧位置 + 速度 * 1 (dt = 1)
        //   新速度 = 旧速度
        //
        // 对应的矩阵操作:
        //
        //   [cx']   [1 0 0 0 | 1 0 0 0] [cx ]
        //   [cy']   [0 1 0 0 | 0 1 0 0] [cy ]
        //   [a' ] = [0 0 1 0 | 0 0 1 0] [a  ]
        //   [h' ]   [0 0 0 1 | 0 0 0 1] [h  ]
        //   [v_cx]  [0 0 0 0 | 1 0 0 0] [v_cx]
        //   [v_cy]  [0 0 0 0 | 0 1 0 0] [v_cy]
        //   [v_a ]  [0 0 0 0 | 0 0 1 0] [v_a ]
        //   [v_h ]  [0 0 0 0 | 0 0 0 1] [v_h ]
        //          ├───── 左上 I ───┤├── 右上 dt*I ──┤
        //          ├───── 左下 0 ───┤├── 右下 I    ──┤
        //
        // 左上角 4x4 单位矩阵: 位置本身保持不变 (1*cx = cx)
        // 右上角 4x4 单位矩阵: 位置 += 速度 * 1 (1*v_cx 加到 cx')
        // 左下角 4x4 零矩阵: 速度不受位置影响
        // 右下角 4x4 单位矩阵: 速度保持不变 (1*v_cx = v_cx)

        // 先把 F 设成 8x8 单位矩阵 (对角线为 1, 其余为 0)
        this->_motion_mat = Eigen::MatrixXf::Identity(8, 8);

        // 然后设置右上角 4x4 子块为 dt=1
        // _motion_mat(行, 列) = 值
        // 这里 i 从 0 到 3, 对应位置分量的 4 个维度
        // _motion_mat(i, i+4) 就是第 i 行第 i+4 列
        // 例如 i=0: _motion_mat(0, 4) = 1, 表示 cx 受 v_cx 影响
        for (int32 i = 0; i < 4; i++)
        {
            this->_motion_mat(i, i + 4) = 1.0f;  // dt = 1.0
        }

        // ---------------------------------------------------------
        // 第 2 件事: 初始化观测矩阵 H (_update_mat)
        // ---------------------------------------------------------
        //
        // H 是 4x8 矩阵, 提取前 4 维 (位置), 忽略后 4 维 (速度):
        //
        //   [cx_meas]   [1 0 0 0 0 0 0 0]
        //   [cy_meas] = [0 1 0 0 0 0 0 0]
        //   [a_meas ]   [0 0 1 0 0 0 0 0]
        //   [h_meas ]   [0 0 0 1 0 0 0 0]
        //
        // 4x8 单位矩阵天然就是 H (因为单位矩阵的前 4 列就是 I4, 后 4 列是 0)
        this->_update_mat = Eigen::MatrixXf::Identity(4, 8);

        // ---------------------------------------------------------
        // 第 3 件事: 设置噪声权重
        // ---------------------------------------------------------
        //
        // 这两个值在 ByteTrack 和 DeepSORT 中完全一样,
        // 是经过大量实验验证的"黄金参数"。
        this->_std_weight_position = 1.0f / 20.0f;   // = 0.05
        this->_std_weight_velocity = 1.0f / 160.0f;  // = 0.00625
    }

    /***
     * @description: 析构函数: 默认 (没有需要手动释放的资源)
     * @return
     */
    ~KalmanFilter() = default;

    /***
     * @description: 初始化新轨迹
     * 当一个新目标第一次被检测到时 (比如一个人刚进入画面), 我们需要为它创建一个"初始状态"。
     *
     * 为什么速度初始化为 0 但协方差很大?
     *      因为第一帧我们完全不知道目标的速度。设速度为 0 是"最保险的猜测",
     *      但给很大的协方差表示"我猜它可能是 0, 但我不确定"。
     *      这样当第二帧检测到来时, 卡尔曼增益会给检测值很大权重, 速度会迅速收敛到真实值。
     *
     * 为什么宽高比 a 的协方差特别小 (1e-2)?
     *      因为宽高比在第一帧就确定下来了, 后续变化很慢。
     *      所以我们很确定初始的宽高比就是正确的。
     *
     * @param measurement KAL_HMEAN& : 检测器输出的第一个检测框 [cx, cy, a, h]; (1, 4)
     * @return:
     * mean       : 初始状态 [cx, cy, a, h, 0, 0, 0, 0]
     *              前 4 维 = 检测框, 后 4 维 = 0 (速度未知, 先设为 0)
     * covariance : 初始协方差矩阵 (对角阵)
     *              对角线的值表示我们对初始状态的"不确定度"
     */
    KAL_DATA initiate(const KAL_HMEAN& measurement)
    {
        // ---- 第一步: 构建初始状态均值 mean ----
        // mean 是 (1, 8) : [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
        KAL_MEAN mean;

        // 前 4 维 (位置): 直接复制测量值
        // block<行数, 列数>(起始行, 起始列)
        // 这里把 measurement (1x4) 复制到 mean 的前 4 个位置 (0行0列开始, 1行4列)
        mean.block<1, 4>(0, 0) = measurement;

        // 后 4 维 (速度): 设为 0
        // setZero() 把指定区域的所有元素设为 0
        mean.block<1, 4>(0, 4).setZero();

        // ---- 第二步: 构建初始协方差矩阵 covariance ----
        // 协方差矩阵是 8x8 的对角阵 (假设各维度独立, 不相关)
        // 对角线上的值 = (标准差)^2

        // 获取目标高度, 用于缩放噪声
        // 原因: 大目标的位置不确定度比小目标大
        float32 height = measurement(3);

        // 位置不确定度的标准差 (4 个值)
        // 标准差 = 倍数 * weight * height
        // 倍数 = 2: 给位置一个适中的不确定度
        float32 std_pos[] = {
            2.0f * this->_std_weight_position * height,  // cx: 位置不确定度
            2.0f * this->_std_weight_position * height,  // cy: 位置不确定度
            1e-2f,                                       // a:  不确定度很小! 因为第一帧的宽高比通常很准
            2.0f * this->_std_weight_position * height   // h:  位置不确定度
        };

        // 速度不确定度的标准差 (4 个值)
        // 标准差 = 倍数 * weight * height
        // 倍数 = 10: 给速度很大的不确定度 (因为我们不知道速度)
        float32 std_vel[] = {
            10.0f * this->_std_weight_velocity * height,  // v_cx: 速度不确定度很大
            10.0f * this->_std_weight_velocity * height,  // v_cy: 速度不确定度很大
            1e-5f,                                        // v_a:  几乎为 0 (宽高比变化率基本为 0)
            10.0f * this->_std_weight_velocity * height   // v_h:  速度不确定度很大
        };

        // 把上面的 8 个标准差组合成一个 8 维向量
        KAL_MEAN std;  // (1, 8)

        std.block<1, 4>(0, 0) = Eigen::Map<KAL_HMEAN>(std_pos);  // 前 4 个 = 位置标准差
        std.block<1, 4>(0, 4) = Eigen::Map<KAL_HMEAN>(std_vel);  // 后 4 个 = 速度标准差

        // 协方差 = 每个标准差平方后放在对角线上
        // .array() 把矩阵转为数组 (支持逐元素运算)
        // .square() 每个元素平方
        // .asDiagonal() 把向量转为 对角矩阵[除对角线外所有元素为0]
        KAL_MEAN tmp = std.array().square();  // (1, 8)
        // (1, 8) -> (8, 8)
        KAL_COVA covariance = tmp.asDiagonal();

        // 返回: {初始均值, 初始协方差}
        return std::make_pair(mean, covariance);
    }

    /***
     * @description: 预测步骤; 每帧开始时对每条轨迹调用一次,
     *               在检测结果出来之前预测目标的新位置;
     *               公式(1): mean' = F * mean (恒定速度假设更新位置);
     *               公式(2): P' = F * P * F^T + Q (协方差传播 + 过程噪声);
     * @param mean       [in/out] 进入时是旧状态均值, 返回时是预测的新状态均值;
     * @param covariance [in/out] 进入时是旧协方差矩阵, 返回时是预测的新协方差矩阵;
     * @return
     */
    void predict(KAL_MEAN& mean, KAL_COVA& covariance)
    {
        // ---- 第一步: 计算过程噪声 Q ----
        // Q 是 8x8 对角阵, 表示"模型预测不准确的程度"
        // Q 越大, 说明我们越不相信自己的预测,
        // 后续的更新步骤就会给检测值更大权重。

        // 当前状态的高度, 用于缩放噪声
        float32 height = mean(3);

        // 位置过程噪声标准差
        // 这些值乘以 height 是因为大目标运动时像素变化更大
        float32 std_pos[] = {
            this->_std_weight_position * height,  // cx: 位置噪声
            this->_std_weight_position * height,  // cy: 位置噪声
            1e-2f,                                // a:  宽高比变化很小
            this->_std_weight_position * height   // h:  位置噪声
        };

        // 速度过程噪声标准差
        // 速度噪声远小于位置噪声, 因为我们假设速度是平滑变化的
        float32 std_vel[] = {
            this->_std_weight_velocity * height,  // v_cx: 速度噪声
            this->_std_weight_velocity * height,  // v_cy: 速度噪声
            1e-5f,                                // v_a:  宽高比变化率几乎不变
            this->_std_weight_velocity * height   // v_h:  速度噪声
        };

        // 组合成 8 维噪声标准差向量
        KAL_MEAN tmp;
        tmp.block<1, 4>(0, 0) = Eigen::Map<KAL_HMEAN>(std_pos);  // 位置噪声
        tmp.block<1, 4>(0, 4) = Eigen::Map<KAL_HMEAN>(std_vel);  // 速度噪声

        // 标准差平方后 -> 方差 -> 对角阵 = 过程噪声协方差 Q
        // .matrix() 把 Array 转回 Matrix (asDiagonal 需要 Matrix) [8, 8]
        KAL_COVA motion_cov = tmp.array().square().matrix().asDiagonal();

        // ---- 第二步: 执行预测 ----
        // 公式 (1): mean' = F * mean
        // mean 是 1x8 行向量, F 是 8x8 矩阵
        // mean * F^T 或 F * mean^T, 结果再转置回来
        // 这里用 F * mean^T: mean^T 变成 8x1, 乘 F (8x8) 得到 8x1, 再转置回 1x8
        KAL_MEAN predicted_mean = (this->_motion_mat * mean.transpose()).transpose();

        // 公式 (2): P' = F * P * F^T + Q
        // F * P * F^T: 旧协方差经过线性变换传播
        // + Q: 加上过程噪声, 使协方差变大
        KAL_COVA predicted_cov = this->_motion_mat * covariance * this->_motion_mat.transpose();
        predicted_cov += motion_cov;

        // 更新输出参数
        mean = predicted_mean;
        covariance = predicted_cov;
    }

    /***
     * @description: 投影步骤(内部使用); 将8维状态空间的均值和协方差投影到4维观测空间;
     *               被 update() 和 gating_distance() 内部调用;
     *               公式: z_pred = H * mean, S = H * P * H^T + R;
     * @param mean       8维状态均值向量 [cx, cy, a, h, v_cx, v_cy, v_a, v_h];
     * @param covariance 8x8状态协方差矩阵;
     * @return           预测的测量值对 (KAL_HDATA):
     *                   first  = projected_mean (4维观测均值 [cx, cy, a, h]);
     *                   second = projected_cov  (4x4观测协方差, 包含测量噪声R);
     */
    KAL_HDATA project(const KAL_MEAN& mean, const KAL_COVA& covariance)
    {
        // ---- 第一步: 计算测量噪声 R ----
        // 测量噪声表示"检测器输出的边界框有多不准确"
        // 注意: 这里的宽高比 a 的噪声设得比较大 (1e-1),
        // 因为检测器对宽高比的估计通常不太稳定
        float32 height = mean(3);

        // 测量噪声标准差 (4 维)
        float32 std_meas[] = {
            this->_std_weight_position * height,  // cx: 检测器 x 误差
            this->_std_weight_position * height,  // cy: 检测器 y 误差
            1e-1f,                                // a:  宽高比误差 (比过程噪声大 10 倍!)
            this->_std_weight_position * height   // h:  检测器高度误差
        };

        // ---- 第二步: 预测测量值 ----
        // z_pred = H * mean
        // H 取前 4 维, 所以 z_pred = [cx, cy, a, h] (预测的检测框)
        // ( (4, 8) @ (1, 8).T ).T = (4, 1).T = (1, 4)
        KAL_HMEAN projected_mean = (this->_update_mat * mean.transpose()).transpose();

        // ---- 第三步: 计算新息协方差 S ----
        // S = H * P * H^T + R
        // 这表示"预测的检测框有多不准" (包含模型误差 + 检测器误差)
        // (4, 8) @ (8, 8) @ (8, 4) = (4, 4)
        KAL_HCOVA projected_cov = this->_update_mat * covariance * this->_update_mat.transpose();

        // 加上测量噪声 R (4x4 对角阵)
        KAL_HCOVA diag = Eigen::Map<KAL_HMEAN>(std_meas).array().square().matrix().asDiagonal();
        projected_cov += diag;

        return std::make_pair(projected_mean, projected_cov);
    }

    /***
     * @description: 更新步骤; 检测框与轨迹匹配成功后,
     *               用实际检测位置修正轨迹的预测位置;
     *               步骤: 1)project投影 2)计算新息y 3)计算卡尔曼增益K
     *               4)修正均值mean=预测+K*y 5)更新协方差P=(I-K*H)*P;
     *               使用Cholesky分解求解线性方程组, 避免直接求逆矩阵;
     * @param mean        预测的状态均值 (来自 predict());
     * @param covariance  预测的状态协方差 (来自 predict());
     * @param measurement 实际检测到的边界框 [cx, cy, a, h];
     * @return            更新后的状态对 (KAL_DATA):
     *                    first  = new_mean       (修正后的8维状态均值);
     *                    second = new_covariance (修正后的8x8协方差矩阵);
     */
    KAL_DATA update(const KAL_MEAN& mean, const KAL_COVA& covariance, const KAL_HMEAN& measurement)
    {
        // ---- 第 1 步: 投影到测量空间 ----
        // 看看"按照我的预测, 检测器应该看到什么"
        KAL_HDATA pa = this->project(mean, covariance);
        // z_pred: 预测的测量值 (1x4)
        KAL_HMEAN projected_mean = pa.first;
        // S: 新息协方差 (4x4); NOTE: S 是对称矩阵 S = S.T
        KAL_HCOVA projected_cov = pa.second;

        // ---- 第 2 步: 计算新息 (Innovation) ----
        // 新息 = 实际值 - 预测值 ; (1, 4)
        // 如果新息大, 说明预测不准, 需要大幅修正
        // 如果新息小, 说明预测很准, 只需小幅修正
        KAL_HMEAN innovation = measurement - projected_mean;

        // ---- 第 3 步: 计算卡尔曼增益 K ----
        // K 决定了修正的力度
        //
        // 理论公式: K = P * H^T * S^(-1)
        //          -> K * S = P * H^T -> S * K^T =  H * P^T
        // 实际实现用 Cholesky 分解解: S * K^T = H * P^T
        //
        // B = (covariance * H^T)^T = H * covariance (因为协方差P 对称)
        // 所以我们需要解: S * K^T = B
        // 等价于: S * K^T = H * covariance
        // 解得 K^T, 再转置得到 K
        // ( (8, 8) @ (4, 8).T ).T = (4, 8)
        Eigen::Matrix<float32, 4, 8> B = (covariance * this->_update_mat.transpose()).transpose();

        // projected_cov.llt(): 对 S 做 Cholesky 分解 S = L * L^T
        // .solve(B): 解 L * L^T * X = B, 这里 X = K^T
        // .transpose(): K^T 转置得到 K (8x4)
        Eigen::Matrix<float32, 8, 4> kalman_gain = projected_cov.llt().solve(B).transpose();

        // ---- 第 4 步: 更新状态均值 ----
        // 新均值 = 预测均值 + K * 新息
        // K * 新息 = 修正量
        // 新息是 1x4, K^T 是 4x8, 结果 1x8
        // (1, 8) + (1, 4) * (8, 4).T = (1, 8)
        KAL_MEAN new_mean = mean + innovation * kalman_gain.transpose();

        // ---- 第 5 步: 更新状态协方差 ----
        // 新协方差 = 旧协方差 - K * S * K^T
        // 引入测量后, 不确定度减小 (减掉的部分就是"测量带来的信息")
        KAL_COVA new_covariance = covariance - kalman_gain * projected_cov * kalman_gain.transpose();

        return std::make_pair(new_mean, new_covariance);
    }

    /***
     * @description: 计算马氏距离(DeepSORT级联匹配用);
     *               衡量预测位置与实际检测位置之间的"归一化距离";
     *               马氏距离考虑了各维度的方差和相关性, 相比欧氏距离更合理;
     *               门控条件: 当 d^2 < chi2inv95[4]=9.4877 时认为可匹配;
     * @param mean          预测的状态均值;
     * @param covariance    预测的状态协方差;
     * @param measurements  多个检测框的集合 (std::vector<KAL_HMEAN>);
     * @param only_position 是否仅使用位置(cx,cy)计算距离 (暂未实现, 默认false);
     * @return              每个检测框的马氏距离平方 (1xN 矩阵);
     */
    Eigen::Matrix<float32, 1, Eigen::Dynamic> gating_distance(const KAL_MEAN& mean, const KAL_COVA& covariance,
                                                              const std::vector<KAL_HMEAN>& measurements,
                                                              bool only_position = false)
    {
        // ---- 第 1 步: 投影到测量空间 ----
        // 获得预测的测量值和新息协方差
        KAL_HDATA pa = this->project(mean, covariance);
        KAL_HMEAN projected_mean = pa.first;  // z_pred (1x4)
        KAL_HCOVA projected_cov = pa.second;  // S (4x4)

        // ---- only_position 模式 (暂未实现) ----
        // 如果开启, 只使用 (cx, cy) 两个维度计算距离
        // 本实现暂不支持, 全部使用 4 维
        if (only_position)
        {
            // TODO: 实现仅使用位置信息的马氏距离计算
        }

        // ---- 第 2 步: 构建差值矩阵 ----
        // 计算每个检测框与预测值的差值
        // diff 是 Nx4 矩阵, N = 检测框数量
        Eigen::Matrix<float32, Eigen::Dynamic, 4, Eigen::RowMajor> diff(measurements.size(), 4);

        for (size_t i = 0; i < measurements.size(); i++)
        {
            diff.row(static_cast<int32>(i)) = measurements[i] - projected_mean;
        }

        // ---- 第 3 步: 计算马氏距离平方 ----
        // 用 Cholesky 分解求 d^2 = z^T * S^(-1) * z
        //
        // 具体做法:
        //   1. S = L * L^T (Cholesky 分解)
        //   2. z = L^(-1) * diff^T   (归一化: 消除尺度和相关性)
        //   3. d^2 = sum(z * z)      (按列求和, 每个检测框一个距离)
        //
        // projected_cov.llt(): Cholesky 分解 S = L * L^T
        // .matrixL(): 获取下三角矩阵 L
        Eigen::Matrix<float32, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> L = projected_cov.llt().matrixL();

        // 用三角回代法解 L * z = diff^T
        // triangularView<Lower>: 利用 L 的下三角结构加速求解
        // OnTheRight: 求解 X * L^T = diff (右边乘)
        // 结果转置: 每列对应一个检测框的归一化残差
        Eigen::Matrix<float32, Eigen::Dynamic, Eigen::Dynamic> z =
            L.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(diff).transpose();

        // 马氏距离平方 = sum(z^2) 沿列求和
        // 每个元素对应一个检测框的马氏距离平方
        Eigen::Matrix<float32, 1, Eigen::Dynamic> square_maha = z.array().square().colwise().sum();

        return square_maha;
    }
};

}  // namespace tracker

#endif  // !__KALMANFILTER__H__
