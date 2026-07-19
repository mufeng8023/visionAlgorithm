/***
 * @Author       : gxs
 * @Date         : 2026-06-22 21:42:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 21:45:00
 * @FilePath     : /visionAlgorithm/lib/tracker/bytetrack/ByteTracker.hpp
 * @Description  : ByteTrack 跟踪器实现;
 *                 基于高/低分检测框两次 IoU 关联的鲁棒跟踪器;
 *                 参考 /mnt/E/CodeFiles/C++/bytetracker/src/BYTETracker.cpp;
 *
 *                 ============================================================
 *                 ByteTrack 核心思想
 *                 ============================================================
 *
 *                 ByteTrack 的"Byte"来源于"Byte" (字节) ,
 *                 意思是"不放过任何一个检测框, 就像不放过任何一个字节".
 *
 *                 传统跟踪器 (如 SORT) 只使用高置信度检测框进行关联,
 *                 低置信度检测框 (比如被遮挡时的检测) 就直接丢弃了.
 *
 *                 ByteTrack 的创新在于: "低分检测框也有价值".
 *                 它通过两次关联来利用所有检测框:
 *
 *                 第一次关联 (高-高): 高置信度轨迹 x 高置信度检测;
 *                 第二次关联 (高-低): 未匹配高置信度轨迹 x 低置信度检测;
 *
 *                 这样做的假设:
 *                 - 高分检测框通常是正确的检测 (目标清晰可见);
 *                 - 低分检测框通常是部分遮挡或模糊的检测 (目标被遮挡);
 *                 - 如果轨迹被遮挡, 它很可能匹配不到高分检测;
 *                 - 但它有可能匹配到一个低分检测 (同一个目标, 只是被遮挡了);
 *                 ============================================================
 *
 *                 核心流程:
 *                 1. 按置信度分流检测框 (高/低);
 *                 2. 卡尔曼预测所有已有轨迹;
 *                 3. 第一次关联: 已确认轨迹 x 高置信度检测 (IoU);
 *                 4. 第二次关联: 未匹配已确认轨迹 x 低置信度检测 (IoU);
 *                 5. 第三次关联: 未确认轨迹 x 剩余高置信度检测 (IoU);
 *                 6. 初始化新轨迹 (高置信度);
 *                 7. 轨迹状态维护 (New->Tracked->Lost->Removed);
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __BYTETRACKER__H__
#define __BYTETRACKER__H__

#include <vector>

#include "tracker/BaseTracker.hpp"
#include "tracker/BoxObject.hpp"
#include "tracker/KalmanFilter.hpp"
#include "tracker/bytetrack/BytetrackTrack.hpp"
#include "tracker/bytetrack/matching.hpp"

namespace tracker
{
namespace bytetrack
{

/***
 * @description: ByteTrack 跟踪器;
 *               继承 BaseTracker, 实现高低分检测框两次关联;
 *
 *               轨迹状态机:
 *                New (新创建) --(连续 n_init 帧匹配成功)--> Tracked
 *                Tracked --(连续 max_age 帧未匹配)--> Lost
 *                Lost --(再次匹配成功)--> Tracked (重新激活)
 *                Lost --(超时)--> Removed
 *
 *               每个轨迹有 4 种状态 (ByteTrackState):
 *               - New:      刚创建, 等待确认;
 *               - Tracked:  已确认, 正在跟踪;
 *               - Lost:     丢失, 等待重新激活;
 *               - Removed:  已移除, 将被清理;
 */
class ByteTracker : public BaseTracker
{
   public:
    // tracked_stracks: 已确认跟踪中的轨迹 (Tracked 和 New 状态);
    // 注意! 这里用的是值对象 (不是指针), 生命周期由 vector 自身管理;
    std::vector<BytetrackTrack> tracked_stracks;

    // lost_stracks: 丢失的轨迹 (Lost 状态);
    // 这些轨迹可能被重新激活 (re_activate);
    std::vector<BytetrackTrack> lost_stracks;

    // removed_stracks: 已移除的轨迹 (Removed 状态);
    // 仅用于记录, 不再参与任何匹配;
    std::vector<BytetrackTrack> removed_stracks;

    // output_stracks: 上一次 update() 的输出结果 (活跃轨迹);
    // 外部通过 get_active_tracks() 或直接读取此变量获取跟踪结果;
    std::vector<BytetrackTrack> output_stracks;

   public:
    /***
     * @description: 构造函数;
     *               @note ByteTrack 的初始化完全由基类完成;
     *               轨迹列表初始为空, 第一次 update 时开始填充;
     * @param config const TrackerConfig& : 跟踪器配置;
     */
    explicit ByteTracker(const TrackerConfig& config) : BaseTracker(config)
    {
        // 构造函数体为空, 初始化由基类完成;
    }

    /***
     * @description: 析构函数;
     */
    virtual ~ByteTracker() = default;

    /***
     * @description: 更新跟踪器 (每帧调用);
     *               实现 ByteTrack 的完整跟踪流程;
     *               结果存储在 this->output_stracks 中;
     *
     *               参考 bytetracker BYTETracker::update();
     *
     *               6 个步骤:
     *               1. 检测框分流 + 轨迹寿命检查;
     *               2. 第一次关联: 轨迹池 x 高置信度检测;
     *               3. 第二次关联: 未匹配轨迹 x 低置信度检测;
     *               4. 第三次关联: 未确认轨迹 x 剩余高置信度检测;
     *               5. 初始化新轨迹;
     *               6. 更新状态 (合并/去重/输出);
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

        // !Step 1: 获取检测结果并分流
        // 按置信度分流: 高分检测 (>= track_thresh) / 低分检测;
        // 同时保存 BoxObject (用于 update/re_activate) 和 BytetrackTrack (用于 IoU);
        std::vector<BytetrackTrack> detections_high_track;
        std::vector<BytetrackTrack> detections_low_track;
        std::vector<BoxObject> detections_high_box;  // 平行于 detections_high_track;
        std::vector<BoxObject> detections_low_box;   // 平行于 detections_low_track;

        for (size_t i = 0; i < detections.size(); i++)
        {
            const BoxObject& det = detections[i];

            // 创建 BytetrackTrack (用于 IoU 计算);
            // track_id = -1 表示"尚未分配 ID" (新检测);
            BytetrackTrack strack(det, this->_frame_id, -1, this->_config.expand_box_rate);

            if (det.score >= this->_config.track_thresh)
            {
                // 高分检测: 优先级高, 参与第一次和第三次关联;
                detections_high_track.push_back(strack);
                detections_high_box.push_back(det);
            }
            else if (det.score >= this->_config.track_thresh * 0.5f)
            {
                // 低分检测: 阈值在 track_thresh * 0.5 ~ track_thresh 之间;
                // 只有这个区间的检测框才会被保留; 低于 track_thresh * 0.5 的直接丢弃;
                detections_low_track.push_back(strack);
                detections_low_box.push_back(det);
            }
        }

        // ---- 对已有轨迹做寿命检查 ----
        // 如果轨迹超过 max_age 帧没有更新, 标记为 Removed;
        // 注意! tracked_stracks 和 lost_stracks 都需要做寿命检查;
        {
            // 检查 tracked_stracks;
            std::vector<BytetrackTrack> updated_tracked;
            for (size_t i = 0; i < this->tracked_stracks.size(); i++)
            {
                if (this->_frame_id - this->tracked_stracks[i].frame_id > this->_config.max_age)
                {
                    this->tracked_stracks[i].mark_removed();
                    this->removed_stracks.push_back(this->tracked_stracks[i]);
                }
                else
                {
                    updated_tracked.push_back(this->tracked_stracks[i]);
                }
            }
            this->tracked_stracks = updated_tracked;
        }

        {
            // 检查 lost_stracks;
            std::vector<BytetrackTrack> updated_lost;
            for (size_t i = 0; i < this->lost_stracks.size(); i++)
            {
                if (this->_frame_id - this->lost_stracks[i].frame_id > this->_config.max_age)
                {
                    this->lost_stracks[i].mark_removed();
                    this->removed_stracks.push_back(this->lost_stracks[i]);
                }
                else
                {
                    updated_lost.push_back(this->lost_stracks[i]);
                }
            }
            this->lost_stracks = updated_lost;
        }

        // !Step 2: 第一次关联, 基于 IoU
        // 合并 tracked_stracks + lost_stracks 构成轨迹池;
        // 对所有轨迹进行卡尔曼预测;
        std::vector<BytetrackTrack*> strack_pool = joint_stracks(this->tracked_stracks, this->lost_stracks);
        BytetrackTrack::multi_predict(strack_pool, this->_kalman_filter);

        // 计算轨迹池与高分检测间的 IoU 距离矩阵;
        std::vector<std::vector<float32>> dists_1 = cal_iou_distance(strack_pool, detections_high_track);
        ByteMatchResult match1 = linear_assignment(dists_1, this->_config.match_thresh);

        // 处理第一次匹配结果;
        std::vector<BytetrackTrack> activated_stracks;
        std::vector<BytetrackTrack> refind_stracks;

        for (size_t i = 0; i < match1.matches.size(); i++)
        {
            int32 track_idx = match1.matches[i].first;
            int32 det_idx = match1.matches[i].second;

            BytetrackTrack* track = strack_pool[track_idx];
            const BoxObject& det_box = detections_high_box[det_idx];

            if (track->state == ByteTrackState::Tracked)
            {
                // 已确认轨迹: 用检测更新卡尔曼状态和边界框;
                track->update(det_box, this->_frame_id);
                activated_stracks.push_back(*track);
            }
            else
            {
                // 丢失轨迹: 重新激活 (从 Lost 状态恢复到 Tracked);
                track->re_activate(det_box, this->_frame_id, false);
                refind_stracks.push_back(*track);
            }
        }

        // !Step 3: 第二次关联, 使用低分检测
        // 收集第一次匹配中未匹配的 Tracked 轨迹;
        std::vector<BytetrackTrack*> r_tracked_stracks;
        for (size_t i = 0; i < match1.unmatched_tracks.size(); i++)
        {
            int32 idx = match1.unmatched_tracks[i];
            if (strack_pool[idx]->state == ByteTrackState::Tracked)
            {
                r_tracked_stracks.push_back(strack_pool[idx]);
            }
        }

        // 计算这些轨迹与低分检测间的 IoU 距离;
        std::vector<std::vector<float32>> dists_2 = cal_iou_distance(r_tracked_stracks, detections_low_track);
        ByteMatchResult match2 = linear_assignment(dists_2, this->_config.match_thresh);

        // 处理第二次匹配结果;
        std::vector<BytetrackTrack> lost_stracks_new;
        for (size_t i = 0; i < match2.matches.size(); i++)
        {
            int32 track_idx = match2.matches[i].first;
            int32 det_idx = match2.matches[i].second;

            BytetrackTrack* track = r_tracked_stracks[track_idx];
            const BoxObject& det_box = detections_low_box[det_idx];

            if (track->state == ByteTrackState::Tracked)
            {
                track->update(det_box, this->_frame_id);
                activated_stracks.push_back(*track);
            }
            else
            {
                track->re_activate(det_box, this->_frame_id, false);
                refind_stracks.push_back(*track);
            }
        }

        // 第二次匹配中未匹配的轨迹标记为 Lost;
        for (size_t i = 0; i < match2.unmatched_tracks.size(); i++)
        {
            int32 idx = match2.unmatched_tracks[i];
            BytetrackTrack* track = r_tracked_stracks[idx];
            if (track->state != ByteTrackState::Lost)
            {
                track->mark_lost();
                lost_stracks_new.push_back(*track);
            }
        }

        // !Step 4: 处理未确认轨迹 (New 状态)
        // 收集第一次匹配中未匹配的高分检测 (用于匹配未确认轨迹);
        std::vector<BytetrackTrack> detections_cp_track;
        std::vector<BoxObject> detections_cp_box;
        for (size_t i = 0; i < match1.unmatched_detections.size(); i++)
        {
            int32 idx = match1.unmatched_detections[i];
            detections_cp_track.push_back(detections_high_track[idx]);
            detections_cp_box.push_back(detections_high_box[idx]);
        }

        // 收集未确认轨迹 (New 状态, is_activated = false);
        std::vector<BytetrackTrack*> unconfirmed;
        for (size_t i = 0; i < this->tracked_stracks.size(); i++)
        {
            if (!this->tracked_stracks[i].is_activated())
            {
                unconfirmed.push_back(&this->tracked_stracks[i]);
            }
        }

        // 计算未确认轨迹与剩余高分检测间的 IoU 距离;
        std::vector<std::vector<float32>> dists_3 = cal_iou_distance(unconfirmed, detections_cp_track);
        ByteMatchResult match3 = linear_assignment(dists_3, this->_config.match_thresh);

        // 处理第三次匹配结果;
        for (size_t i = 0; i < match3.matches.size(); i++)
        {
            int32 track_idx = match3.matches[i].first;
            int32 det_idx = match3.matches[i].second;

            unconfirmed[track_idx]->update(detections_cp_box[det_idx], this->_frame_id);
            activated_stracks.push_back(*unconfirmed[track_idx]);
        }

        // 第三次匹配中未匹配的未确认轨迹直接删除;
        for (size_t i = 0; i < match3.unmatched_tracks.size(); i++)
        {
            int32 idx = match3.unmatched_tracks[i];
            unconfirmed[idx]->mark_removed();
            this->removed_stracks.push_back(*unconfirmed[idx]);
        }

        // !Step 5: 初始化新轨迹
        // 第三次匹配中未匹配的高分检测 -> 初始化新轨迹;
        std::vector<int32> unmatched_dets_3 = match3.unmatched_detections;
        for (size_t i = 0; i < unmatched_dets_3.size(); i++)
        {
            int32 idx = unmatched_dets_3[i];
            BytetrackTrack& det = detections_cp_track[idx];
            // 只有高分检测才能创建新轨迹;
            // 低分检测太不可靠, 创建新轨迹会导致大量假阳性;
            if (det.score >= this->_config.high_thresh)
            {
                det.activate(this->_kalman_filter, this->_frame_id);
                activated_stracks.push_back(det);
            }
        }

        // !Step 6: 更新状态
        // 更新 tracked_stracks: 只保留 Tracked 状态;
        {
            std::vector<BytetrackTrack> tracked_swap;
            for (size_t i = 0; i < this->tracked_stracks.size(); i++)
            {
                if (this->tracked_stracks[i].state == ByteTrackState::Tracked)
                {
                    tracked_swap.push_back(this->tracked_stracks[i]);
                }
            }
            this->tracked_stracks = tracked_swap;
        }

        // 合并激活和重新找到的轨迹;
        this->tracked_stracks = joint_stracks_val(this->tracked_stracks, activated_stracks);
        this->tracked_stracks = joint_stracks_val(this->tracked_stracks, refind_stracks);

        // 从 lost 中移除已找回的轨迹 (防止同一个轨迹同时出现在 tracked 和 lost 中);
        this->lost_stracks = sub_stracks(this->lost_stracks, this->tracked_stracks);

        // 添加新丢失的轨迹;
        for (size_t i = 0; i < lost_stracks_new.size(); i++)
        {
            this->lost_stracks.push_back(lost_stracks_new[i]);
        }

        // 从 lost 中移除已标记为 removed 的轨迹;
        this->lost_stracks = sub_stracks(this->lost_stracks, this->removed_stracks);

        // 移除 tracked 和 lost 之间的重复轨迹;
        {
            std::vector<BytetrackTrack> resa, resb;
            remove_duplicate_stracks(resa, resb, this->tracked_stracks, this->lost_stracks);
            this->tracked_stracks = resa;
            this->lost_stracks = resb;
        }

        // 输出活跃的轨迹 (is_activated == true);
        this->output_stracks.clear();
        for (size_t i = 0; i < this->tracked_stracks.size(); i++)
        {
            if (this->tracked_stracks[i].is_activated())
            {
                this->output_stracks.push_back(this->tracked_stracks[i]);
            }
        }
    }

    /***
     * @description: 获取当前活跃的轨迹指针列表;
     *               @note 返回的是指向 this->tracked_stracks 内部元素的指针,
     *               如果后续调用了 push_back 导致 vector 扩容, 指针可能失效;
     *               建议在每次 update() 后立即读取;
     * @return std::vector<BytetrackTrack*> : 活跃轨迹指针列表;
     */
    std::vector<BytetrackTrack*> get_active_tracks()
    {
        std::vector<BytetrackTrack*> active;
        for (size_t i = 0; i < this->tracked_stracks.size(); i++)
        {
            if (this->tracked_stracks[i].is_activated())
            {
                active.push_back(&this->tracked_stracks[i]);
            }
        }
        return active;
    }

    /***
     * @description: 重置跟踪器状态;
     *               清空所有轨迹列表, 重置帧计数器和轨迹 ID 计数器;
     *               通常在视频序列切换时调用;
     */
    void reset() override
    {
        BaseTracker::reset();
        this->tracked_stracks.clear();
        this->lost_stracks.clear();
        this->removed_stracks.clear();
        this->output_stracks.clear();

        // 重置轨迹 ID 计数器 (ByteTrack 使用静态计数器);
        BytetrackTrack::next_id(0);
    }
};

}  // namespace bytetrack
}  // namespace tracker

#endif  // !__BYTETRACKER__H__