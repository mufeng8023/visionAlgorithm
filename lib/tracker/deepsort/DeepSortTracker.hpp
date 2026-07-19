/***
 * @Author       : gxs
 * @Date         : 2026-06-22 21:39:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 21:39:00
 * @FilePath     : /visionAlgorithm/lib/tracker/deepsort/DeepSortTracker.hpp
 * @Description  : DeepSORT 跟踪器实现;
 *                 基于 SORT + ReID 外观特征 + 级联匹配;
 *                 参考 /mnt/E/CodeFiles/C++/DeepSORT/tracker/deepsort/src/tracker.cpp;
 *
 *                 核心流程:
 *                 1. 卡尔曼预测所有已有轨迹;
 *                 2. 级联匹配 (马氏距离 + 余弦距离): 已确认轨迹 x 检测;
 *                 3. IoU 匹配: 未匹配轨迹 + 未确认轨迹 x 未匹配检测;
 *                 4. 初始化新轨迹 (Tentative 状态);
 *                 5. 清理已删除轨迹;
 *
 *                 ============================================================
 *                 DeepSORT 与 SORT 的核心区别:
 *                 ============================================================
 *
 *                 SORT 只有 1 次关联 (全部 IoU);
 *                 DeepSORT 有 2 次关联 (级联匹配 + IoU 兜底);
 *
 *                 级联匹配的意义:
 *                 如果轨迹 A 刚更新 (time_since_update=1) 且轨迹 B 丢失 10 帧,
 *                 级联匹配让 A 先挑, B 只能捡 A 剩下的.
 *                 这避免了 B 的"不确定光环"抢走 A 该得的检测框.
 *
 *                 为什么有了级联还需要 IoU 匹配?
 *                 级联匹配用的是马氏距离, 需要卡尔曼滤波器预测.
 *                 当轨迹刚初始化 (Tentative), 预测协方差很大, 马氏距离不可靠.
 *                 所以对未确认轨迹直接用 IoU 匹配更鲁棒.
 *                 ============================================================
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __DEEPSORTTRACKER__H__
#define __DEEPSORTTRACKER__H__

#include <memory>  // std::shared_ptr
#include <vector>  // std::vector

#include "tracker/BaseTracker.hpp"
#include "tracker/BoxObject.hpp"
#include "tracker/KalmanFilter.hpp"
#include "tracker/deepsort/DeepSortTrack.hpp"
#include "tracker/deepsort/matching.hpp"

namespace tracker
{
namespace deepsort
{

/***
 * @description: DeepSORT 跟踪器;
 *               继承 BaseTracker, 实现 SORT + ReID 级联匹配;
 *
 *               生命周期管理:
 *               - 所有轨迹存储在 tracks 向量中 (shared_ptr 管理);
 *               - 外部可通过 get_active_tracks() 获取已确认轨迹;
 *               - 未确认轨迹 (Tentative) 经过 n_init 帧确认后变为 Confirmed;
 *               - 丢失 max_age 帧的轨迹自动标记为 Deleted;
 */
class DeepSORTTracker : public BaseTracker
{
   public:
    // tracks: 当前所有轨迹 (Tentative + Confirmed + 待删除);
    // 使用 shared_ptr 管理生命周期, 外部可持有轨迹指针而不用担心野指针;
    std::vector<std::shared_ptr<DeepSortTrack>> tracks;

   public:
    /***
     * @description: 构造函数;
     *               基类 BaseTracker(config) 负责初始化:
     *               _frame_id = 0, _next_id = 1, _kalman_filter, _config;
     * @param config const TrackerConfig& : 跟踪器配置;
     */
    explicit DeepSORTTracker(const TrackerConfig& config) : BaseTracker(config)
    {
        // 构造函数体为空, 初始化由基类完成;
    }

    /***
     * @description: 析构函数;
     */
    virtual ~DeepSORTTracker() = default;

    /***
     * @description: 更新跟踪器 (每帧调用);
     *               实现 DeepSORT 的完整跟踪流程;
     *
     *               参考 deepsort tracker::update();
     *
     *               3 个步骤:
     *               1. _predict_all(): 卡尔曼预测所有轨迹;
     *               2. _match(): 数据关联 (级联匹配 + IoU);
     *               3. _remove_deleted_tracks(): 清理无效轨迹;
     *
     * @param detections const std::vector<BoxObject>& : 当前帧检测结果;
     * @param frame_id   int32 : 当前帧 ID (默认 -1 表示自增);
     */
    void update(const std::vector<BoxObject>& detections, int32 frame_id = -1) override
    {
        // ---- 第 0 步: 帧 ID 管理 ----
        if (frame_id > 0)
        {
            this->_frame_id = frame_id;
        }
        else
        {
            this->_frame_id++;
        }

        // ---- 第 1 步: 预测所有轨迹 ----
        this->_predict_all();

        // ---- 第 2 步: 数据关联 ----
        this->_match(detections);

        // ---- 第 3 步: 清理已删除轨迹 ----
        this->_remove_deleted_tracks();
    }

    /***
     * @description: 获取当前所有活跃的轨迹指针 (Confirmed 状态);
     *               DeepSORT 只输出已确认的轨迹;
     *               Tentative 状态的轨迹还不够稳定, 不输出;
     * @return std::vector<DeepSortTrack*> : 活跃轨迹指针列表;
     */
    std::vector<DeepSortTrack*> get_active_tracks()
    {
        std::vector<DeepSortTrack*> active;
        for (size_t i = 0; i < this->tracks.size(); i++)
        {
            if (this->tracks[i]->is_confirmed())
            {
                active.push_back(this->tracks[i].get());
            }
        }
        return active;
    }

    /***
     * @description: 重置跟踪器状态;
     *               清空所有轨迹, 重置帧计数器和轨迹 ID 计数器;
     */
    void reset() override
    {
        // 调用基类 reset: 重置 _frame_id 和 _next_id;
        BaseTracker::reset();
        this->tracks.clear();
    }

   private:
    /***
     * @description: 预测所有轨迹 (卡尔曼滤波预测);
     *               对每条轨迹调用 KalmanFilter::predict() 更新 mean 和 covariance;
     *               每帧必须在匹配之前调用, 让轨迹"向前走一步";
     */
    void _predict_all()
    {
        for (size_t i = 0; i < this->tracks.size(); i++)
        {
            this->tracks[i]->predict(&this->_kalman_filter);
        }
    }

    /***
     * @description: 数据关联 (匹配阶段);
     *               参考 deepsort tracker::_match();
     *
     *               步骤:
     *               1. 分流: 已确认轨迹 vs 未确认轨迹;
     *               2. 级联匹配: 已确认轨迹 x 所有检测框;
     *               3. 构建 IoU 匹配候选: 未确认轨迹 + 级联未匹配的确认轨迹;
     *               4. IoU 匹配: 候选轨迹 x 级联未匹配的检测框;
     *               5. 处理匹配结果: update / mark_missed / _initiate_track;
     *
     * @param detections const std::vector<BoxObject>& : 当前帧检测结果;
     */
    void _match(const std::vector<BoxObject>& detections)
    {
        // ---- 分流: 确认态 + 未确认态 ----
        // 确认态轨迹: 参与级联匹配;
        // 未确认态轨迹: 等待 IoU 匹配;
        std::vector<int32> confirmed_tracks;
        std::vector<int32> unconfirmed_tracks;

        for (size_t i = 0; i < this->tracks.size(); i++)
        {
            if (this->tracks[i]->is_confirmed())
            {
                confirmed_tracks.push_back(static_cast<int32>(i));
            }
            else if (!this->tracks[i]->is_deleted())
            {
                unconfirmed_tracks.push_back(static_cast<int32>(i));
            }
        }

        // ---- 构建轨迹指针向量 (用于匹配函数) ----
        // 匹配函数需要 DeepSortTrack* 类型的输入;
        std::vector<DeepSortTrack*> track_ptrs;
        for (size_t i = 0; i < this->tracks.size(); i++)
        {
            track_ptrs.push_back(this->tracks[i].get());
        }

        // ---- 级联匹配: 已确认轨迹 x 检测 ----
        // 注意: 当没有 ReID 特征时, features / track_features 传入空列表;
        // 余弦距离矩阵将返回零矩阵, 级联匹配退化为纯马氏距离匹配;
        std::vector<std::vector<float32>> features;        // 空: 不使用 ReID;
        std::vector<std::vector<float32>> track_features;  // 空: 不使用 ReID;

        MatchResult match_cascade = cascade_matching(&this->_kalman_filter,              //
                                                     this->_config.max_iou_distance,     //
                                                     this->_config.max_age,              //
                                                     track_ptrs,                         //
                                                     detections,                         //
                                                     confirmed_tracks,                   //
                                                     features,                           //
                                                     track_features,                     //
                                                     this->_config.max_cosine_distance,  //
                                                     this->_config.lambda_cosine_weight  //
        );

        // ---- 准备 IoU 匹配的轨迹候选 ----
        // 包括: 未确认轨迹 + 级联匹配中 time_since_update == 1 的未匹配确认轨迹;
        // 为什么只包括 time_since_update == 1?
        // 丢失超过 1 帧的轨迹预测框不可靠, IoU 计算没有意义;
        std::vector<int32> iou_track_candidates;
        iou_track_candidates.assign(unconfirmed_tracks.begin(), unconfirmed_tracks.end());

        for (size_t i = 0; i < match_cascade.unmatched_tracks.size(); i++)
        {
            int32 idx = match_cascade.unmatched_tracks[i];
            if (this->tracks[idx]->time_since_update == 1)
            {
                // 仅丢失了 1 帧的确认轨迹也参与 IoU 匹配;
                iou_track_candidates.push_back(idx);
            }
        }

        // ---- IoU 匹配: 剩余轨迹 x 未匹配检测 ----
        MatchResult match_iou = iou_matching(track_ptrs,                          //
                                             detections,                          //
                                             iou_track_candidates,                //
                                             match_cascade.unmatched_detections,  //
                                             this->_config.max_iou_distance       //
        );

        // ================================================================
        // 处理匹配结果
        // ================================================================

        // ---- 处理级联匹配的匹配对 ----
        // 级联匹配成功的轨迹, 用检测框更新卡尔曼状态;
        for (size_t i = 0; i < match_cascade.matches.size(); i++)
        {
            int32 track_idx = match_cascade.matches[i].first;
            int32 det_idx = match_cascade.matches[i].second;
            this->tracks[track_idx]->update(&this->_kalman_filter, detections[det_idx]);
        }

        // ---- 处理 IoU 匹配的匹配对 ----
        // IoU 匹配成功的轨迹 (包括未确认轨迹和丢失 1 帧的确认轨迹);
        for (size_t i = 0; i < match_iou.matches.size(); i++)
        {
            int32 track_idx = match_iou.matches[i].first;
            int32 det_idx = match_iou.matches[i].second;
            this->tracks[track_idx]->update(&this->_kalman_filter, detections[det_idx]);
        }

        // ---- 处理未匹配的轨迹 (标记丢失) ----
        // 级联匹配中未匹配的确认轨迹 (time_since_update > 1 的);
        // 注意: time_since_update == 1 的确认轨迹会进入 IoU 匹配, 不在这个列表里;
        for (size_t i = 0; i < match_cascade.unmatched_tracks.size(); i++)
        {
            int32 idx = match_cascade.unmatched_tracks[i];
            if (this->tracks[idx]->time_since_update > 1)
            {
                // time_since_update > 1 的轨迹标记为丢失;
                this->tracks[idx]->mark_missed();
            }
        }

        // IoU 匹配中未匹配的轨迹;
        for (size_t i = 0; i < match_iou.unmatched_tracks.size(); i++)
        {
            int32 idx = match_iou.unmatched_tracks[i];
            // Tentative 轨迹未匹配 -> mark_missed (直接删除);
            // Confirmed 轨迹未匹配 -> 先检查 time_since_update, 由 mark_missed 处理;
            this->tracks[idx]->mark_missed();
        }

        // ---- 处理未匹配的检测 (初始化新轨迹) ----
        std::vector<int32> unmatched_dets = match_iou.unmatched_detections;
        for (size_t i = 0; i < unmatched_dets.size(); i++)
        {
            int32 det_idx = unmatched_dets[i];
            this->_initiate_track(detections[det_idx]);
        }
    }

    /***
     * @description: 初始化新轨迹;
     *               使用检测框创建 Tentative 状态的新轨迹;
     *               参考 deepsort tracker::_initiate_track();
     *
     *               新轨迹的生命周期:
     *               刚创建: Tentative 状态;
     *               连续 n_init 帧都匹配成功 -> 提升为 Confirmed;
     *               连续 max_age 帧未匹配 -> 标记为 Deleted;
     *
     * @param detection const BoxObject& : 检测框;
     */
    void _initiate_track(const BoxObject& detection)
    {
        // ---- 使用扩展后的框 (ltwh_expand) 进行卡尔曼初始化 ----
        // 扩展框比原始框大, 包含了边界框的缩放裕量;
        // 这是 DeepSORT 原版的做法, 可以提高跟踪的鲁棒性;
        std::array<float32, 4> xyah_arr = BoxObject::ltwh_to_xyah(detection.ltwh_expand);
        KAL_HMEAN xyah;
        xyah << xyah_arr[0], xyah_arr[1], xyah_arr[2], xyah_arr[3];

        // ---- 卡尔曼初始化 ----
        // initiate() 返回 {初始状态均值, 初始状态协方差};
        KAL_DATA init_data = this->_kalman_filter.initiate(xyah);

        // ---- 分配 track_id ----
        // next_track_id() 自增基类的 _next_id, 保证 ID 唯一;
        int32 track_id = this->next_track_id();

        // ---- 创建新轨迹 ----
        std::shared_ptr<DeepSortTrack> new_track =
            std::make_shared<DeepSortTrack>(init_data.first,               // mean
                                            init_data.second,              // covariance
                                            track_id,                      // track_id
                                            this->_frame_id,               // frame_id
                                            this->_config.n_init,          // n_init (需要连续匹配帧数)
                                            this->_config.max_age,         // max_age (丢失后最大生存帧数)
                                            detection,                     // det_box (包含 ltwh / ltwh_expand)
                                            this->_config.expand_box_rate  // expand_box_rate
            );

        this->tracks.push_back(new_track);
    }

    /***
     * @description: 清理已删除轨迹;
     *               移除 state == Deleted 的轨迹;
     *               参考 deepsort tracker::update() 中 erase 逻辑;
     *
     *               为什么需要清理?
     *               如果轨迹被标记为 Deleted 后不及时清理,
     *               下次匹配时这些轨迹会继续占用资源,
     *               还可能被意外地重新激活;
     */
    void _remove_deleted_tracks()
    {
        std::vector<std::shared_ptr<DeepSortTrack>>::iterator it = this->tracks.begin();
        while (it != this->tracks.end())
        {
            if ((*it)->is_deleted())
            {
                // 释放 shared_ptr 引用, 如果外部没有持有, 自动销毁轨迹对象;
                it = this->tracks.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
};

}  // namespace deepsort
}  // namespace tracker

#endif  // !__DEEPSORTTRACKER__H__