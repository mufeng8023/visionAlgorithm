/***
 * @Author       : gxs
 * @Date         : 2026-06-22 21:42:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-19 14:00:00
 * @FilePath     : /visionAlgorithm/lib/tracker/bytetrack/ByteTracker.hpp
 * @Description  : ByteTrack 跟踪器实现;
 *                 基于高/低分检测框两次 IoU 关联的鲁棒跟踪器;
 *
 *                 ============================================================
 *                 ByteTrack 核心思想
 *                 ============================================================
 *
 *                 ByteTrack 的"Byte"来源于"Byte" (字节),
 *                 意思是"不放过任何一个检测框, 就像不放过任何一个字节";
 *
 *                 传统跟踪器 (如 SORT) 只使用高置信度检测框进行关联,
 *                 低置信度检测框 (比如被遮挡时的检测) 就直接丢弃了;
 *
 *                 ByteTrack 的创新在于: "低分检测框也有价值";
 *                 它通过两次关联来利用所有检测框:
 *
 *                 关联设计 (三轮 IoU 匹配):
 *                 - 轨迹池仅包含 Tracked+Lost 状态的轨迹 (New 状态不参与卡尔曼预测);
 *                 - 未确认轨迹 (New 状态) 在分离后单独参与第三次关联;
 *
 *                 这样做的假设:
 *                 - 高分检测框通常是正确的检测 (目标清晰可见);
 *                 - 低分检测框通常是部分遮挡或模糊的检测 (目标被遮挡);
 *                 - 如果轨迹被遮挡, 它很可能匹配不到高分检测;
 *                 - 但它有可能匹配到一个低分检测 (同一个目标, 只是被遮挡了);
 *                 ============================================================
 *
 *                 核心流程:
 *                 - 按置信度分流检测框 (高/低);
 *                 - 将 tracked_stracks 分离为 active_tracked (Tracked) 和 unconfirmed (New);
 *                 - 轨迹池 = active_tracked + lost_stracks, 进行卡尔曼预测;
 *                 - 关联一: 轨迹池 x 高分检测 (阈值 match_thresh);
 *                 - 关联二: 未匹配 Tracked 轨迹 x 低分检测 (阈值 match_thresh_low);
 *                 - 关联三: 未确认轨迹 x 剩余高分检测 (阈值 match_thresh_unconfirmed);
 *                 - 初始化新轨迹 (仅高分检测);
 *                 - 更新状态列表并检查 lost_stracks 生命周期;
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __BYTETRACKER__H__
#define __BYTETRACKER__H__

#include <map>
#include <vector>

#include "tracker/BaseTracker.hpp"
#include "tracker/BoxObject.hpp"
#include "tracker/TrackResult.hpp"
#include "tracker/bytetrack/BytetrackTrack.hpp"
#include "tracker/bytetrack/matching.hpp"

namespace tracker
{
namespace bytetrack
{

/***
 * @description: ByteTrack 跟踪器;
 *               继承 BaseTracker, 实现高低分检测框三轮 IoU 关联;
 *
 *               轨迹状态机:
 *                New (新创建) --(连续 n_init 帧匹配成功)--> Tracked
 *                Tracked --(未匹配)--> Lost
 *                Lost --(再次匹配成功)--> Tracked (重新激活)
 *                Lost --(超过 max_age 帧)--> Removed
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
    // tracked_stracks: 包含 Tracked 状态 (已确认) 和 New 状态 (未确认) 的轨迹;
    // 注意! 这里用的是值对象 (不是指针), 生命周期由 vector 自身管理;
    std::vector<BytetrackTrack> tracked_stracks;

    // lost_stracks: 丢失的轨迹 (Lost 状态);
    // 这些轨迹可能被重新激活 (re_activate);
    std::vector<BytetrackTrack> lost_stracks;

    // removed_stracks: 已移除的轨迹 (Removed 状态);
    // 仅用于记录, 不再参与任何匹配;
    std::vector<BytetrackTrack> removed_stracks;

    // output_stracks: 上一次 update() 的输出结果 (已确认活跃轨迹);
    // 外部通过 get_active_tracks() 或直接读取此变量获取跟踪结果;
    std::vector<BytetrackTrack> output_stracks;

   public:
    /***
     * @description: 构造函数;
     *               ByteTrack 的初始化完全由基类完成;
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
     *               关键修正 (对比原始实现):
     *               - tracked_stracks 在构建轨迹池之前先分离为 active_tracked 和 unconfirmed;
     *               - 轨迹池仅含 active_tracked (Tracked) + lost_stracks, New 状态不参与预测;
     *               - 三次关联分别使用不同阈值: match_thresh / match_thresh_low / match_thresh_unconfirmed;
     *               - lost_stracks 的生命周期检查在所有关联完成之后进行;
     *
     * @param detections const std::vector<BoxObject>& : 当前帧检测结果;
     * @param frame_id   int32 : 当前帧 ID (默认 -1 表示自增);
     */
    void update(const std::vector<BoxObject>& detections,  //
                std::vector<TrackResult>& results,         //
                int32 frame_id = -1) override
    {
        results.clear();

        // ---- 帧 ID 管理 ----
        // 负数时自增, >= 0 时使用传入的帧号(含第0帧);
        if (frame_id >= 0)
        {
            this->_frame_id = frame_id;
        }
        else
        {
            this->_frame_id++;
        }

        // ================================================================
        // 关联准备: 检测框分流 + 轨迹预分类
        // ================================================================

        // 按置信度分流检测框, 同时记录原始 detections[] 中的下标;
        // idx_high[j] = detections_high_box[j] 在原始 detections 中的位置;
        // idx_low[j]  = detections_low_box[j]  在原始 detections 中的位置;
        std::vector<BytetrackTrack> detections_high_track;
        std::vector<BytetrackTrack> detections_low_track;
        std::vector<BoxObject> detections_high_box;
        std::vector<BoxObject> detections_low_box;
        std::vector<int32> idx_high;  // 高分检测的原始下标
        std::vector<int32> idx_low;   // 低分检测的原始下标

        for (size_t i = 0; i < detections.size(); i++)
        {
            const BoxObject& det = detections[i];
            // track_id = -1 表示尚未分配 ID (新检测临时占位);
            BytetrackTrack strack(det, this->_frame_id, -1, this->_config.expand_box_rate);

            if (det.score >= this->_config.track_thresh)
            {
                // 高分检测: 参与关联一 (vs 轨迹池) 和关联三 (vs 未确认轨迹);
                detections_high_track.push_back(strack);
                detections_high_box.push_back(det);
                idx_high.push_back(static_cast<int32>(i));
            }
            else if (det.score >= this->_config.track_thresh * 0.5f)
            {
                // 低分检测: 只保留 [track_thresh*0.5, track_thresh) 区间;
                // 低于 track_thresh*0.5 的直接丢弃, 太不可靠;
                detections_low_track.push_back(strack);
                detections_low_box.push_back(det);
                idx_low.push_back(static_cast<int32>(i));
            }
        }

        // track_det_map: track_id → 匹配到的 detections[] 原始下标;
        // 在三轮关联过程中逐步填充, 最终用于构建 results;
        std::map<int32, int32> track_det_map;

        // 关键修正: 在构建轨迹池之前, 先将 tracked_stracks 按激活状态分离;
        // - active_tracked: Tracked 状态 (is_activated == true), 进入轨迹池;
        // - unconfirmed: New 状态 (is_activated == false), 不进入轨迹池 (卡尔曼未初始化);
        // 原因: New 状态的轨迹卡尔曼还未正式激活, 如果参与 multi_predict 会产生错误结果;
        std::vector<BytetrackTrack*> unconfirmed;
        std::vector<BytetrackTrack*> active_tracked;
        for (size_t i = 0; i < this->tracked_stracks.size(); i++)
        {
            if (this->tracked_stracks[i].is_activated())
            {
                // 已激活轨迹: 进入轨迹池, 参与第一次和第二次关联;
                active_tracked.push_back(&this->tracked_stracks[i]);
            }
            else
            {
                // 未确认轨迹 (New 状态): 稍后单独参与第三次关联;
                unconfirmed.push_back(&this->tracked_stracks[i]);
            }
        }

        // ================================================================
        // 关联一: 轨迹池 x 高分检测 (阈值 match_thresh)
        // ================================================================

        // 轨迹池 = Tracked 状态 + Lost 状态 (不含 New 状态);
        // 使用 joint_stracks_ptr 合并指针列表 (active_tracked) 和对象列表 (lost_stracks);
        std::vector<BytetrackTrack*> strack_pool = joint_stracks_ptr(active_tracked, this->lost_stracks);

        // 卡尔曼预测: 将所有轨迹的位置向前推进一步;
        // 注意! 只有 Tracked+Lost 状态的轨迹才进行预测, New 状态已被排除在外;
        BytetrackTrack::multi_predict(strack_pool, this->_kalman_filter);

        // 计算 IoU 距离矩阵并求解线性分配;
        std::vector<std::vector<float32>> dists_1 = cal_iou_distance(strack_pool, detections_high_track);
        ByteMatchResult match1 = linear_assignment(dists_1, this->_config.match_thresh);

        // 处理关联一结果;
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
                // Tracked 轨迹: 卡尔曼更新, 更新边界框;
                track->update(det_box, this->_frame_id);
                activated_stracks.push_back(*track);
            }
            else
            {
                // Lost 轨迹匹配到高分检测: 重新激活;
                track->re_activate(det_box, this->_frame_id, false);
                refind_stracks.push_back(*track);
            }
            // 记录 track_id → 原始 detections[] 下标的映射 (供 results 构建使用);
            track_det_map[track->track_id] = idx_high[det_idx];
        }

        // ================================================================
        // 关联二: 未匹配 Tracked 轨迹 x 低分检测 (阈值 match_thresh_low)
        // ================================================================

        // 收集关联一未匹配的 Tracked 状态轨迹 (Lost 状态的不参与关联二);
        // 原因: 低分检测用于修复被遮挡的 Tracked 轨迹, 不用于重新激活 Lost 轨迹;
        std::vector<BytetrackTrack*> r_tracked_stracks;
        for (size_t i = 0; i < match1.unmatched_tracks.size(); i++)
        {
            int32 idx = match1.unmatched_tracks[i];
            if (strack_pool[idx]->state == ByteTrackState::Tracked)
            {
                r_tracked_stracks.push_back(strack_pool[idx]);
            }
        }

        // 使用更宽松的阈值 match_thresh_low (默认 0.5, 比 match_thresh 低);
        // 允许低分检测框的模糊匹配, 适应目标被遮挡时的低置信度情况;
        std::vector<std::vector<float32>> dists_2 = cal_iou_distance(r_tracked_stracks, detections_low_track);
        ByteMatchResult match2 = linear_assignment(dists_2, this->_config.match_thresh_low);

        // 处理关联二结果;
        std::vector<BytetrackTrack> lost_stracks_new;
        for (size_t i = 0; i < match2.matches.size(); i++)
        {
            int32 track_idx = match2.matches[i].first;
            int32 det_idx = match2.matches[i].second;

            BytetrackTrack* track = r_tracked_stracks[track_idx];
            const BoxObject& det_box = detections_low_box[det_idx];

            if (track->state == ByteTrackState::Tracked)
            {
                // 用低分检测更新被遮挡的 Tracked 轨迹;
                track->update(det_box, this->_frame_id);
                activated_stracks.push_back(*track);
            }
            else
            {
                track->re_activate(det_box, this->_frame_id, false);
                refind_stracks.push_back(*track);
            }
            // 记录 track_id → 原始 detections[] 下标的映射 (供 results 构建使用);
            track_det_map[track->track_id] = idx_low[det_idx];
        }

        // 关联二未匹配的 Tracked 轨迹标记为 Lost;
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

        // ================================================================
        // 关联三: 未确认轨迹 x 剩余高分检测 (阈值 match_thresh_unconfirmed)
        // ================================================================

        // 收集关联一中未匹配的高分检测 (轨迹池没有认领的检测框);
        // 这些检测框将与未确认轨迹 (New 状态) 进行匹配;
        std::vector<BytetrackTrack> detections_cp_track;
        std::vector<BoxObject> detections_cp_box;
        std::vector<int32> idx_cp;  // 关联三检测在原始 detections[] 中的下标映射;
        for (size_t i = 0; i < match1.unmatched_detections.size(); i++)
        {
            int32 idx = match1.unmatched_detections[i];
            detections_cp_track.push_back(detections_high_track[idx]);
            detections_cp_box.push_back(detections_high_box[idx]);
            idx_cp.push_back(idx_high[idx]);
        }

        // 使用阈值 match_thresh_unconfirmed (默认 0.7, 对未确认轨迹稍严格);
        // 未确认轨迹只出现了 1 帧, 需要更高可信度的匹配才能继续存活;
        std::vector<std::vector<float32>> dists_3 = cal_iou_distance(unconfirmed, detections_cp_track);
        ByteMatchResult match3 = linear_assignment(dists_3, this->_config.match_thresh_unconfirmed);

        // 关联三匹配成功: 未确认轨迹继续存活并更新;
        for (size_t i = 0; i < match3.matches.size(); i++)
        {
            int32 track_idx = match3.matches[i].first;
            int32 det_idx = match3.matches[i].second;

            unconfirmed[track_idx]->update(detections_cp_box[det_idx], this->_frame_id);
            activated_stracks.push_back(*unconfirmed[track_idx]);
            // 记录 track_id → 原始 detections[] 下标的映射 (供 results 构建使用);
            track_det_map[unconfirmed[track_idx]->track_id] = idx_cp[det_idx];
        }

        // 关联三未匹配的未确认轨迹直接删除 (New 状态一帧未能确认就销毁);
        for (size_t i = 0; i < match3.unmatched_tracks.size(); i++)
        {
            int32 idx = match3.unmatched_tracks[i];
            unconfirmed[idx]->mark_removed();
            this->removed_stracks.push_back(*unconfirmed[idx]);
        }

        // ================================================================
        // 初始化新轨迹
        // ================================================================

        // 关联三未匹配的高分检测 -> 初始化 New 状态轨迹;
        // 只有高分检测 (>= high_thresh) 才允许创建新轨迹, 防止大量假阳性;
        std::vector<int32> unmatched_dets_3 = match3.unmatched_detections;
        for (size_t i = 0; i < unmatched_dets_3.size(); i++)
        {
            int32 idx = unmatched_dets_3[i];
            BytetrackTrack& det = detections_cp_track[idx];
            if (det.score >= this->_config.high_thresh)
            {
                // activate() 分配 track_id, 初始化卡尔曼滤波器, 状态设为 New;
                det.activate(this->_kalman_filter, this->_frame_id);
                activated_stracks.push_back(det);
                // activate() 执行后 det.track_id 才有效; 记录新轨迹的检测索引映射;
                track_det_map[det.track_id] = idx_cp[idx];
            }
        }

        // ================================================================
        // 状态更新
        // ================================================================

        // 将 tracked_stracks 中非 Tracked 状态的轨迹移除 (只保留 Tracked 状态);
        // 此时 New 状态轨迹通过 activated_stracks 重新加回来;
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

        // 合并本帧激活的轨迹 (包含新创建的 New 状态和更新的 Tracked 状态);
        this->tracked_stracks = joint_stracks_val(this->tracked_stracks, activated_stracks);
        // 合并重新找到的轨迹 (从 Lost 恢复为 Tracked);
        this->tracked_stracks = joint_stracks_val(this->tracked_stracks, refind_stracks);

        // 从 lost 中移除已找回的轨迹 (防止同一轨迹同时出现在 tracked 和 lost 中);
        this->lost_stracks = sub_stracks(this->lost_stracks, this->tracked_stracks);

        // 将本帧新丢失的轨迹加入 lost_stracks;
        for (size_t i = 0; i < lost_stracks_new.size(); i++)
        {
            this->lost_stracks.push_back(lost_stracks_new[i]);
        }

        // 从 lost 中移除已标记为 removed 的轨迹;
        this->lost_stracks = sub_stracks(this->lost_stracks, this->removed_stracks);

        // 移除 tracked 和 lost 之间 IoU 重叠的重复轨迹 (保留生存时间更长的);
        {
            std::vector<BytetrackTrack> resa, resb;
            remove_duplicate_stracks(resa, resb, this->tracked_stracks, this->lost_stracks);
            this->tracked_stracks = resa;
            this->lost_stracks = resb;
        }

        // ================================================================
        // 生命周期检查: 仅对 lost_stracks (关联完成后再检查, 避免过早移除)
        // ================================================================

        // 超过 max_age 帧未匹配的 Lost 轨迹标记为 Removed;
        // 修正: 生命周期检查移至所有关联完成之后, 只检查 lost_stracks;
        // 原因: 若在关联前提前移除, 可能错过本帧能重新匹配的轨迹;
        {
            std::vector<BytetrackTrack> survived_lost;
            for (size_t i = 0; i < this->lost_stracks.size(); i++)
            {
                if (this->_frame_id - this->lost_stracks[i].frame_id > this->_config.max_age)
                {
                    // 超时: 标记为 Removed 并移入 removed_stracks;
                    this->lost_stracks[i].mark_removed();
                    this->removed_stracks.push_back(this->lost_stracks[i]);
                }
                else
                {
                    // 未超时: 保留在 lost_stracks 中等待下帧匹配;
                    survived_lost.push_back(this->lost_stracks[i]);
                }
            }
            this->lost_stracks = survived_lost;
        }

        // 输出已确认的活跃轨迹 (is_activated == true 表示 Tracked 状态且已通过 n_init 帧确认);
        this->output_stracks.clear();
        for (size_t i = 0; i < this->tracked_stracks.size(); i++)
        {
            if (this->tracked_stracks[i].is_activated())
            {
                this->output_stracks.push_back(this->tracked_stracks[i]);
            }
        }

        // ================================================================
        // 构建 TrackResult 输出
        // ================================================================

        // 遍历已确认的活跃轨迹, 结合 track_det_map 填充 det_index;
        results.reserve(this->output_stracks.size());
        for (size_t i = 0; i < this->output_stracks.size(); i++)
        {
            const BytetrackTrack& track = this->output_stracks[i];

            TrackResult result;
            result.track_id = track.track_id;
            result.cls_id = track.cls_id;
            result.score = track.score;
            result.ltwh[0] = track.ltwh[0];
            result.ltwh[1] = track.ltwh[1];
            result.ltwh[2] = track.ltwh[2];
            result.ltwh[3] = track.ltwh[3];

            // 从 track_det_map 查找本帧匹配到的原始 detections[] 下标;
            // ByteTrack 的 output_stracks 均应有对应匹配, 若未找到则 det_index = -1;
            std::map<int32, int32>::const_iterator map_it = track_det_map.find(track.track_id);
            result.det_index = (map_it != track_det_map.end()) ? map_it->second : -1;

            results.push_back(result);
        }
    }

    /***
     * @description: 获取当前活跃的轨迹指针列表;
     *               返回的是指向 this->tracked_stracks 内部元素的指针;
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
