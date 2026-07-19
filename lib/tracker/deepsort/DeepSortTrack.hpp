/***
 * @Author       : gxs
 * @Date         : 2026-06-22 10:36:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 14:50:00
 * @FilePath     : /visionAlgorithm/lib/tracker/deepsort/DeepSortTrack.hpp
 * @Description  : DeepSORT 算法的轨迹类实现
 *                 参考: /mnt/E/CodeFiles/C++/DeepSORT/tracker/deepsort/include/track.h
 *                       /mnt/E/CodeFiles/C++/DeepSORT/tracker/deepsort/src/track.cpp
 *
 *                 边界框存储策略 (BoxObject 架构):
 *                 - ltwh[4]: 原始检测框 (保存后永不修改)
 *                 - ltwh_expand[4]: 扩展后的检测框 (用于所有卡尔曼操作)
 *                 - 所有 getter 方法基于 ltwh_expand 计算
 *
 *                 DeepSORT 轨迹的特点:
 *                 1. 不持有卡尔曼滤波器对象, 通过指针引用 KalmanFilter
 *                 2. 使用 mean 直接转换为扩展框 (不额外存储框数据)
 *                 3. 状态流: Tentative -> Confirmed <-> Deleted
 *                 4. 有 hits/age/time_since_update 统计字段
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __DEEPSORTTRACK__H__
#define __DEEPSORTTRACK__H__

#include "tracker/BaseTrack.hpp"
#include "tracker/BoxObject.hpp"
#include "tracker/KalmanFilter.hpp"
#include "tracker/TrackState.hpp"

namespace tracker
{
namespace deepsort
{

/***
 * @description: DeepSORT 算法的轨迹类
 *               参考 deepsort 的 Track 实现;
 *
 *               和 ByteTrack 最大的区别:
 *               - DeepSORT 不持有 KalmanFilter, 通过指针引用 (逐个预测, 无需批量)
 *               - DeepSORT 有 hits/age/time_since_update 统计字段
 *               - DeepSORT 的轨迹状态只有三个: Tentative -> Confirmed -> Deleted
 *
 * @note 状态流说明:
 *   Tentative: 刚创建的轨迹, 需要连续匹配 n_init 帧才能转为 Confirmed
 *   Confirmed: 稳定跟踪中的轨迹
 *   Deleted:   已被移除的轨迹 (跟踪器会定期清理)
 */
class DeepSortTrack : public BaseTrack<DeepSortState>
{
   public:
    // hits: 匹配成功次数 (每 update() 一次 +1), 用于确认轨迹
    // 参考 deepsort Track::hits
    int32 hits = 0;

    // age: 轨迹存在帧数 (每 predict() 一次 +1)
    // 参考 deepsort Track::age
    int32 age = 0;

    // time_since_update: 自上次更新以来的帧数
    // 参考 deepsort Track::time_since_update
    int32 time_since_update = 0;

   protected:
    // _n_init: 轨迹确认所需的最少匹配帧数 (Tentative -> Confirmed 需要 hits >= _n_init)
    // 参考 deepsort Track::_n_init
    int32 _n_init = 3;

    // _max_age: 轨迹最大丢失帧数 (time_since_update > _max_age 时标记为 Deleted)
    // 参考 deepsort Track::_max_age
    int32 _max_age = 30;

   public:
    // ================================================================
    // 构造函数
    // ================================================================

    /***
     * @description: 有参构造函数, 创建 DeepSORT 的轨迹对象
     *
     *               DeepSORT 的轨迹创建方式和 ByteTrack 不同:
     *               ByteTrack 先创建空轨迹, 再 activate() 初始化卡尔曼;
     *               DeepSORT 创建时直接传入卡尔曼状态 (先 initiate, 再创建轨迹);
     *
     *               参考 deepsort Track(mean, covariance, track_id, n_init, max_age, feature)
     *
     *               NOTE: mean 是由外部 KalmanFilter::initiate() 创建的初始状态;
     *               mean 中的位置分量 (cx, cy, a, h) 已在"扩展空间"中 (如果扩展率 > 0);
     *               构造函数中同时初始化 ltwh (原始框) 和 ltwh_expand (扩展框);
     *
     * @param mean             const KAL_MEAN&  : 卡尔曼初始状态均值 (由 KalmanFilter::initiate() 创建)
     * @param covariance       const KAL_COVA& : 卡尔曼初始状态协方差
     * @param track_id         int32 : 轨迹 ID
     * @param frame_id         int32 : 创建时的帧 ID
     * @param n_init           int32 : 轨迹确认所需的最少匹配帧数
     * @param max_age          int32 : 轨迹最大丢失帧数
     * @param det_box          const BoxObject& : 当前帧检测框信息 (包含 ltwh 原始框 和 ltwh_expand 扩展框);
     * @param expand_box_rate  float32 : 边界框扩展率, 从配置传入, 默认 0.0
     */
    DeepSortTrack(const KAL_MEAN& mean,        //
                  const KAL_COVA& covariance,  //
                  int32 track_id,              //
                  int32 frame_id,              //
                  int32 n_init,                //
                  int32 max_age,               //
                  const BoxObject& det_box,    //
                  float32 expand_box_rate = 0.0f)
        : BaseTrack(track_id, frame_id, expand_box_rate), _n_init(n_init), _max_age(max_age)
    {
        // ---- 第 1 步: 保存卡尔曼状态 ----
        this->mean = mean;
        this->covariance = covariance;

        // ---- 第 2 步: 初始化边界框 ----
        // ltwh (原始框): 直接使用检测框的原始 ltwh, 确保外部读取 track->ltwh 时有有效值;
        this->ltwh[0] = det_box.ltwh[0];
        this->ltwh[1] = det_box.ltwh[1];
        this->ltwh[2] = det_box.ltwh[2];
        this->ltwh[3] = det_box.ltwh[3];

        // ltwh_expand (扩展框): 使用检测框的扩展版本, 与 mean 在同一个扩展空间中;
        // 注意: mean 由外部 KalmanFilter::initiate(det_box.ltwh_expand 转 xyah) 创建,
        // 因此 mean 和 ltwh_expand 在同一个扩展空间中, 直接赋值即可;
        // 后续 update_ltwh() 会从 mean 更新 ltwh_expand, 但首次赋值使用 det_box 的扩展框更直接;
        this->ltwh_expand[0] = det_box.ltwh_expand[0];
        this->ltwh_expand[1] = det_box.ltwh_expand[1];
        this->ltwh_expand[2] = det_box.ltwh_expand[2];
        this->ltwh_expand[3] = det_box.ltwh_expand[3];

        // ---- 第 3 步: 初始化检测信息 ----
        // 从 BoxObject 中读取 score 和 cls_id, 与轨迹初始化时传递给 KalmanFilter::initiate()
        // 的检测框保持一致;
        this->score = det_box.score;
        this->cls_id = det_box.cls_id;

        // ---- 第 4 步: 初始化 DeepSORT 特有的字段 ----
        this->state = DeepSortState::Tentative;
        this->hits = 1;
        this->age = 1;
        this->time_since_update = 0;
        this->track_len = 0;
    }

    /***
     * @description: 默认构造函数 (用于容器, 如 std::vector 扩容)
     */
    DeepSortTrack() : BaseTrack(-1, 0, 0.0f), _n_init(3), _max_age(30) {}

    /***
     * @description: 析构函数
     */
    virtual ~DeepSortTrack() = default;

    // ================================================================
    // 核心方法 (参考 deepsort Track)
    // ================================================================

    /***
     * @description: 预测步骤, 将状态分布传播到当前时刻
     *               每帧对所有轨迹调用一次, 在检测结果出来之前预测新位置;
     *
     *               预测后, 通过 update_ltwh_from_mean() 将预测后的 mean
     *               写入 ltwh_expand, 确保扩展框与预测状态一致;
     *               ltwh (原始框) 不受影响;
     *
     * @param kf KalmanFilter* : 卡尔曼滤波器指针 (所有轨迹共享)
     */
    void predict(KalmanFilter* kf)
    {
        // 调用卡尔曼预测: mean' = F * mean, cov' = F * cov * F^T + Q;
        kf->predict(this->mean, this->covariance);

        // 更新统计字段;
        this->age += 1;
        this->time_since_update += 1;

        // 从预测后的 mean 更新 ltwh_expand (扩展空间);
        // mean 存储的是扩展空间中的状态, 直接写入 ltwh_expand;
        this->update_ltwh();
    }

    /***
     * @description: 更新步骤, 用检测框修正轨迹状态
     *               匹配成功后调用;
     *               参考 deepsort Track::update() -- track.cpp:35
     *
     *               更新流程:
     *               1. 从 BoxObject 中获取 ltwh (原始框) 写入 this->ltwh
     *               2. 从 BoxObject 中获取 ltwh_expand (扩展框) 写入 this->ltwh_expand
     *               3. 基于 ltwh_expand 计算 xyah, 进行卡尔曼更新
     *               4. 从更新后的 mean 写回 ltwh_expand
     *
     *               注意: this->ltwh (原始框) 在步骤 1 中被赋值;
     *                     卡尔曼更新仅在扩展空间 (ltwh_expand) 中进行;
     *                     this->ltwh 始终保持原始检测框的值, 不会被卡尔曼覆盖;
     *
     * @param kf          KalmanFilter* : 卡尔曼滤波器指针
     * @param detection   const BoxObject& : 当前帧匹配到的检测框信息
     *                    包含 ltwh (原始框) 和 ltwh_expand (扩展框);
     */
    void update(KalmanFilter* kf, const BoxObject& detection)
    {
        // ---- 第 1 步: 更新原始框 (来自检测数据的原始 ltwh) ----
        // this->ltwh 始终保持检测器原始输出, 永不包含卡尔曼的修正;
        // 这样外部读取 track->ltwh 时, 得到的是"检测器的直接结果";
        this->ltwh[0] = detection.ltwh[0];
        this->ltwh[1] = detection.ltwh[1];
        this->ltwh[2] = detection.ltwh[2];
        this->ltwh[3] = detection.ltwh[3];

        // ---- 第 2 步: 更新扩展框 (来自检测数据的 ltwh_expand) ----
        // ltwh_expand 是扩展后的框, 用于卡尔曼更新;
        // 注意: 这是"观测"的扩展框, 卡尔曼 update 后还会修正;
        this->ltwh_expand[0] = detection.ltwh_expand[0];
        this->ltwh_expand[1] = detection.ltwh_expand[1];
        this->ltwh_expand[2] = detection.ltwh_expand[2];
        this->ltwh_expand[3] = detection.ltwh_expand[3];

        // ---- 第 3 步: 同步检测信息 ----
        // 从 BoxObject 中读取 score 和 cls_id;
        this->score = detection.score;
        this->cls_id = detection.cls_id;

        // ---- 第 4 步: 将扩展框转为 xyah 进行卡尔曼更新 ----
        // 使用基类的 get_xyah() 方法, 该方法基于 ltwh_expand 计算;
        // 返回 [cx, cy, a, h] (扩展空间);
        std::array<float32, 4> xyah_arr = this->get_xyah();
        KAL_HMEAN xyah;
        xyah << xyah_arr[0], xyah_arr[1], xyah_arr[2], xyah_arr[3];

        // 卡尔曼更新: K, mean = mean' + K*(z - H*mean'), cov = (I - K*H)*P';
        KAL_DATA pa = kf->update(this->mean, this->covariance, xyah);
        this->mean = pa.first;
        this->covariance = pa.second;

        // ---- 第 4 步: 从更新后的 mean 写回 ltwh_expand ----
        // mean 经过卡尔曼修正后, 已经是"后验估计"的扩展空间状态;
        // 直接写入 ltwh_expand, 无需收缩;
        this->update_ltwh();

        // 更新统计字段;
        this->hits += 1;
        this->time_since_update = 0;

        // 状态自动提升: Tentative + hits >= n_init -> Confirmed;
        if (this->state == DeepSortState::Tentative && this->hits >= this->_n_init)
        {
            this->state = DeepSortState::Confirmed;
        }
    }

    /***
     * @description: 标记轨迹为缺失 (未匹配到检测框)
     *               Tentative 状态直接删除, Confirmed 状态检查是否超时;
     *               参考 deepsort Track::mark_missed()
     */
    void mark_missed()
    {
        if (this->state == DeepSortState::Tentative)
        {
            // 未确认的轨迹一旦丢失就直接删除;
            this->state = DeepSortState::Deleted;
        }
        else if (this->time_since_update > this->_max_age)
        {
            // 已确认轨迹丢失超过 max_age 帧后删除;
            this->state = DeepSortState::Deleted;
        }
    }

    // ================================================================
    // 状态查询
    // ================================================================

    /***
     * @description: 判断轨迹是否已确认
     *               参考 deepsort Track::is_confirmed()
     */
    bool is_confirmed() const { return this->state == DeepSortState::Confirmed; }

    /***
     * @description: 判断轨迹是否已删除
     *               参考 deepsort Track::is_deleted()
     */
    bool is_deleted() const { return this->state == DeepSortState::Deleted; }

    /***
     * @description: 判断轨迹是否处于 Tentative 状态
     *               参考 deepsort Track::is_tentative()
     */
    bool is_tentative() const { return this->state == DeepSortState::Tentative; }

    // ================================================================
    // 状态标记 (继承自 BaseTrack)
    // ================================================================

    /***
     * @description: 标记轨迹为丢失状态
     *               DeepSORT 使用 mark_missed() 处理丢失;
     */
    void mark_lost() override { this->mark_missed(); }

    /***
     * @description: 标记轨迹为已移除状态 (state -> Deleted)
     */
    void mark_removed() override { this->state = DeepSortState::Deleted; }

    /***
     * @description: 从卡尔曼 mean 更新 this->ltwh_expand (扩展框)
     *
     *               mean 存储 xyah: cx=mean[0], cy=mean[1], a=mean[2], h=mean[3]
     *               通过基类的 update_ltwh_from_mean() 统一处理转换;
     *               mean 和 ltwh_expand 在同一个扩展空间中, 无需 shrink;
     *
     *               参考 deepsort Track::to_tlwh()
     */
    void update_ltwh()
    {
        // 委托基类: mean xyah -> ltwh_expand 转换;
        // 这是扩展空间内的直接映射, 无需扩展或收缩;
        this->update_ltwh_from_mean();
    }
};

}  // namespace deepsort
}  // namespace tracker

#endif  // !__DEEPSORTTRACK__H__