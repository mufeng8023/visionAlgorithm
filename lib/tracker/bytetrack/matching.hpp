/***
 * @Author       : gxs
 * @Date         : 2026-06-22 21:40:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 21:45:00
 * @FilePath     : /visionAlgorithm/lib/tracker/bytetrack/matching.hpp
 * @Description  : ByteTrack 匹配算法实现;
 *                 包括 IoU 距离计算和匈牙利匹配 (LAPJV);
 *
 *                 ============================================================
 *                 ByteTrack 专用工具函数
 *                 ============================================================
 *
 *                 这些函数为 ByteTrack 的"两次关联"提供支持:
 *                 - cal_iou_distance(): 计算 IoU 距离矩阵;
 *                 - linear_assignment(): LAPJV 求解线性分配;
 *                 - joint_stracks(): 合并轨迹列表 (去重);
 *                 - sub_stracks(): 轨迹列表差集;
 *                 - remove_duplicate_stracks(): 基于 IoU 的重复轨迹检测;
 *                 ============================================================
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __BYTETRACK_MATCHING__H__
#define __BYTETRACK_MATCHING__H__

#include <map>     // std::map (用于去重)
#include <vector>  // std::vector

#include "tracker/BoxObject.hpp"
#include "tracker/bytetrack/BytetrackTrack.hpp"
#include "tracker/bytetrack/lapjv.hpp"

namespace tracker
{
namespace bytetrack
{

// 匹配结果结构 (ByteTrack 专用);
struct ByteMatchResult
{
    // matches: 匹配对列表, first = 轨迹索引, second = 检测索引;
    std::vector<std::pair<int32, int32>> matches;

    // unmatched_tracks: 未匹配的轨迹索引列表;
    std::vector<int32> unmatched_tracks;

    // unmatched_detections: 未匹配的检测索引列表;
    std::vector<int32> unmatched_detections;
};

/***
 * @description: 计算 BytetrackTrack 指针列表与对象列表间的 IoU 距离矩阵;
 *               IoU 距离 = 1 - IoU;
 *
 *               使用扩展框 (ltwh_expand) 计算 IoU.
 *               扩展框的计算方式:
 *                 cx = l + w/2, cy = t + h/2;
 *                 new_w = w * expand_ratio, new_h = h * expand_ratio;
 *                 new_l = cx - new_w/2, new_t = cy - new_h/2;
 *
 * @param atracks std::vector<BytetrackTrack*>& : 轨迹指针列表 (行);
 * @param btracks std::vector<BytetrackTrack>& : 检测对象列表 (列);
 * @return std::vector<std::vector<float32>> : IoU 距离矩阵 [n_atracks x n_btracks];
 */
inline std::vector<std::vector<float32>> cal_iou_distance(std::vector<BytetrackTrack*>& atracks,  //
                                                          std::vector<BytetrackTrack>& btracks)   //
{
    int32 n_rows = static_cast<int32>(atracks.size());
    int32 n_cols = static_cast<int32>(btracks.size());

    // 参考 bytetracker iou_distance: 任一维度为 0 时返回空矩阵 (size == 0);
    // 不能返回 n_rows 个空行, 否则 linear_assignment 会误判为非空矩阵;
    if (n_rows == 0 || n_cols == 0)
    {
        return {};
    }

    // 初始化为 1.0 (最大距离, 即 IoU = 0);
    std::vector<std::vector<float32>> cost_matrix(n_rows, std::vector<float32>(n_cols, 1.0f));

    for (int32 i = 0; i < n_rows; i++)
    {
        // ---- 获取轨迹的 xyxy 坐标 ----
        std::array<float32, 4> xyxy_a = atracks[i]->get_xyxy();
        float32 area_a = (xyxy_a[2] - xyxy_a[0]) * (xyxy_a[3] - xyxy_a[1]);

        for (int32 j = 0; j < n_cols; j++)
        {
            // ---- 获取检测的 xyxy 坐标 ----
            std::array<float32, 4> xyxy_b = btracks[j].get_xyxy();
            float32 area_b = (xyxy_b[2] - xyxy_b[0]) * (xyxy_b[3] - xyxy_b[1]);

            // ---- 计算交集 (Intersection) ----
            float32 ix1 = (xyxy_a[0] > xyxy_b[0]) ? xyxy_a[0] : xyxy_b[0];
            float32 iy1 = (xyxy_a[1] > xyxy_b[1]) ? xyxy_a[1] : xyxy_b[1];
            float32 ix2 = (xyxy_a[2] < xyxy_b[2]) ? xyxy_a[2] : xyxy_b[2];
            float32 iy2 = (xyxy_a[3] < xyxy_b[3]) ? xyxy_a[3] : xyxy_b[3];

            float32 iw = ix2 - ix1;
            float32 ih = iy2 - iy1;
            if (iw < 0.0f)
                iw = 0.0f;
            if (ih < 0.0f)
                ih = 0.0f;

            float32 area_intersection = iw * ih;
            float32 area_union = area_a + area_b - area_intersection;
            float32 iou = (area_union > 0.0f) ? (area_intersection / area_union) : 0.0f;
            cost_matrix[i][j] = 1.0f - iou;
        }
    }

    return cost_matrix;
}

/***
 * @description: 计算两个 BytetrackTrack 对象列表间的 IoU 距离矩阵 (重载);
 *               两个参数都是对象列表 (非指针列表);
 *               内部逻辑与指针版本完全一致;
 * @param atracks std::vector<BytetrackTrack>& : 轨迹对象列表 (行);
 * @param btracks std::vector<BytetrackTrack>& : 检测对象列表 (列);
 * @return std::vector<std::vector<float32>> : IoU 距离矩阵 [n_atracks x n_btracks];
 */
inline std::vector<std::vector<float32>> cal_iou_distance(std::vector<BytetrackTrack>& atracks,  //
                                                          std::vector<BytetrackTrack>& btracks)  //
{
    int32 n_rows = static_cast<int32>(atracks.size());
    int32 n_cols = static_cast<int32>(btracks.size());

    // 参考 bytetracker iou_distance: 任一维度为 0 时返回空矩阵 (size == 0);
    if (n_rows == 0 || n_cols == 0)
    {
        return {};
    }

    std::vector<std::vector<float32>> cost_matrix(n_rows, std::vector<float32>(n_cols, 1.0f));

    for (int32 i = 0; i < n_rows; i++)
    {
        std::array<float32, 4> xyxy_a = atracks[i].get_xyxy();
        float32 area_a = (xyxy_a[2] - xyxy_a[0]) * (xyxy_a[3] - xyxy_a[1]);

        for (int32 j = 0; j < n_cols; j++)
        {
            std::array<float32, 4> xyxy_b = btracks[j].get_xyxy();
            float32 area_b = (xyxy_b[2] - xyxy_b[0]) * (xyxy_b[3] - xyxy_b[1]);

            float32 ix1 = (xyxy_a[0] > xyxy_b[0]) ? xyxy_a[0] : xyxy_b[0];
            float32 iy1 = (xyxy_a[1] > xyxy_b[1]) ? xyxy_a[1] : xyxy_b[1];
            float32 ix2 = (xyxy_a[2] < xyxy_b[2]) ? xyxy_a[2] : xyxy_b[2];
            float32 iy2 = (xyxy_a[3] < xyxy_b[3]) ? xyxy_a[3] : xyxy_b[3];

            float32 iw = ix2 - ix1;
            float32 ih = iy2 - iy1;
            if (iw < 0.0f)
                iw = 0.0f;
            if (ih < 0.0f)
                ih = 0.0f;

            float32 area_intersection = iw * ih;
            float32 area_union = area_a + area_b - area_intersection;
            float32 iou = (area_union > 0.0f) ? (area_intersection / area_union) : 0.0f;
            cost_matrix[i][j] = 1.0f - iou;
        }
    }

    return cost_matrix;
}

/***
 * @description: 线性分配 (匈牙利匹配), 使用 LAPJV 算法求解 IoU 距离矩阵;
 *
 *               LAPJV 与 Munkres 的区别:
 *               - LAPJV: Jonker-Volgenant 算法, 在稠密矩阵上 O(n^3);
 *               - Munkres: 经典匈牙利算法, 也 O(n^3);
 *               ByteTrack 选择 LAPJV 是因为原版 Python 实现使用 lapjv 库,
 *               这里为了 API 一致性保留了 lapjv 接口;
 *
 *               参考 bytetracker utils.cpp BYTETracker::linear_assignment;
 *               原版使用 (cost_matrix, cost_matrix_size, cost_matrix_size_size, thresh)
 *               的接口来传递矩阵实际行列数;
 *               因为 cal_iou_distance 在任一输入为空时返回空矩阵,
 *               必须依赖外部传入的 n_tracks/n_dets 来正确标记未匹配项;
 *
 * @param cost_matrix const std::vector<std::vector<float32>>& : IoU 距离矩阵;
 * @param n_tracks    int32 : 实际轨迹数量 (成本矩阵的行数, 即使矩阵为空也需要);
 * @param n_dets      int32 : 实际检测数量 (成本矩阵的列数, 即使矩阵为空也需要);
 * @param thresh      float32 : 匹配阈值 (超过此值的匹配被拒绝);
 * @return ByteMatchResult : 匹配结果;
 */
inline ByteMatchResult linear_assignment(const std::vector<std::vector<float32>>& cost_matrix,  //
                                         int32 n_tracks,                                        //
                                         int32 n_dets,                                          //
                                         float32 thresh)                                        //
{
    ByteMatchResult res;

    // ---- 空矩阵处理 ----
    // 参考 bytetracker utils.cpp: 当 cost_matrix.size() == 0 时,
    // 所有轨迹和检测都标记为未匹配;
    // 这在第一帧 (无轨迹) 或所有轨迹丢失时发生;
    if (cost_matrix.size() == 0)
    {
        for (int32 i = 0; i < n_tracks; i++)
            res.unmatched_tracks.push_back(i);
        for (int32 i = 0; i < n_dets; i++)
            res.unmatched_detections.push_back(i);
        return res;
    }

    // ---- 调用 LAPJV 算法求解线性分配 ----
    // 参考 bytetracker utils.cpp BYTETracker::linear_assignment;
    // extend_cost=true: 自动将非方阵扩展为 (n_rows+n_cols) 方阵;
    // cost_limit=thresh: 通过 cost_limit/2.0 填充实现隐式阈值过滤;
    std::vector<int32> rowsol;
    std::vector<int32> colsol;
    lapjv(cost_matrix, rowsol, colsol, true, thresh);

    // ---- 解析匹配结果 ----
    // 参考 bytetracker: rowsol[i] >= 0 表示行 i 匹配到了真实列;
    // rowsol[i] == -1 表示行 i 被分配到了虚拟列 (即未匹配);
    for (int32 i = 0; i < static_cast<int32>(rowsol.size()); i++)
    {
        if (rowsol[i] >= 0)
        {
            // 找到有效匹配;
            res.matches.push_back(std::make_pair(i, rowsol[i]));
        }
        else
        {
            // 未匹配的轨迹;
            res.unmatched_tracks.push_back(i);
        }
    }

    // ---- 收集未匹配的检测 ----
    for (int32 j = 0; j < static_cast<int32>(colsol.size()); j++)
    {
        if (colsol[j] < 0)
        {
            res.unmatched_detections.push_back(j);
        }
    }

    return res;
}

/***
 * @description: 合并指针列表与对象列表为统一的指针列表 (joint_stracks_ptr);
 *               用于将已分离的 active_tracked (指针) 与 lost_stracks (值) 合并为轨迹池;
 *               指针列表中的轨迹优先入队, 对象列表中未重复的轨迹追加到末尾;
 *               按 track_id 去重;
 *
 *               为什么需要这个函数?
 *               在修正后的 ByteTrack 流程中, 构建轨迹池之前已将 tracked_stracks
 *               按激活状态分为 active_tracked (指针) 和 unconfirmed (指针);
 *               因此合并 active_tracked (指针) + lost_stracks (值) 时,
 *               不能直接用 joint_stracks (需要两个值列表);
 *               joint_stracks_ptr 专门处理"指针列表 + 值列表"的合并场景;
 *
 * @param ptr_list std::vector<BytetrackTrack*>& : 指针列表 (如 active_tracked);
 * @param val_list std::vector<BytetrackTrack>&  : 对象列表 (如 lost_stracks);
 * @return std::vector<BytetrackTrack*> : 合并后的指针列表 (去重);
 */
inline std::vector<BytetrackTrack*> joint_stracks_ptr(std::vector<BytetrackTrack*>& ptr_list,  //
                                                      std::vector<BytetrackTrack>& val_list)   //
{
    std::map<int32, int32> exists;
    std::vector<BytetrackTrack*> res;

    // 先将指针列表中的轨迹加入结果, 并记录已存在的 track_id;
    for (size_t i = 0; i < ptr_list.size(); i++)
    {
        exists[ptr_list[i]->track_id] = 1;
        res.push_back(ptr_list[i]);
    }
    // 再将对象列表中未重复的轨迹加入结果;
    for (size_t i = 0; i < val_list.size(); i++)
    {
        int32 tid = val_list[i].track_id;
        if (exists.find(tid) == exists.end())
        {
            exists[tid] = 1;
            res.push_back(&val_list[i]);
        }
    }
    return res;
}

/***
 * @description: 合并两个 BytetrackTrack 对象向量为指针向量 (joint_stracks);
 *               返回合并后的指针向量, 自动根据 track_id 去重;
 *
 *               为什么需要去重?
 *               在 ByteTrack 的两次关联中, 同一个轨迹可能同时出现在
 *               tracked_stracks 和 lost_stracks 中 (例如轨迹刚从 Tracked 转入 Lost).
 *               joint_stracks 会确保每个 track_id 只出现一次;
 *
 * @param track_list_a std::vector<BytetrackTrack>& : 第一个轨迹对象列表;
 * @param track_list_b std::vector<BytetrackTrack>& : 第二个轨迹对象列表;
 * @return std::vector<BytetrackTrack*> : 合并后的指针列表 (去重);
 */
inline std::vector<BytetrackTrack*> joint_stracks(std::vector<BytetrackTrack>& track_list_a,  //
                                                  std::vector<BytetrackTrack>& track_list_b)  //
{
    std::map<int32, int32> exists;
    std::vector<BytetrackTrack*> res;

    for (size_t i = 0; i < track_list_a.size(); i++)
    {
        exists[track_list_a[i].track_id] = 1;
        res.push_back(&track_list_a[i]);
    }
    for (size_t i = 0; i < track_list_b.size(); i++)
    {
        int32 tid = track_list_b[i].track_id;
        if (exists.find(tid) == exists.end())
        {
            exists[tid] = 1;
            res.push_back(&track_list_b[i]);
        }
    }
    return res;
}

/***
 * @description: 合并两个 BytetrackTrack 对象向量 (值版本);
 *               返回合并后的对象列表, 自动去重;
 *               与 joint_stracks 的区别: 返回的是对象副本, 不是指针;
 *
 * @param track_list_a std::vector<BytetrackTrack>& : 第一个轨迹对象列表;
 * @param track_list_b std::vector<BytetrackTrack>& : 第二个轨迹对象列表;
 * @return std::vector<BytetrackTrack> : 合并后的对象列表 (去重);
 */
inline std::vector<BytetrackTrack> joint_stracks_val(std::vector<BytetrackTrack>& track_list_a,  //
                                                     std::vector<BytetrackTrack>& track_list_b)  //
{
    std::map<int32, int32> exists;
    std::vector<BytetrackTrack> res;
    for (size_t i = 0; i < track_list_a.size(); i++)
    {
        exists[track_list_a[i].track_id] = 1;
        res.push_back(track_list_a[i]);
    }
    for (size_t i = 0; i < track_list_b.size(); i++)
    {
        int32 tid = track_list_b[i].track_id;
        if (exists.find(tid) == exists.end())
        {
            exists[tid] = 1;
            res.push_back(track_list_b[i]);
        }
    }
    return res;
}

/***
 * @description: 从 first 中减去 second 中的轨迹 (集合差);
 *               用于从 lost_stracks 中移除已经找回的轨迹;
 *
 *               例如:
 *                 lost_stracks 有 {A, B, C};
 *                 tracked_stracks 有 {A, D};
 *                 sub_stracks(lost, tracked) = {B, C};
 *
 * @param track_list_a std::vector<BytetrackTrack>& : 被减数向量;
 * @param track_list_b std::vector<BytetrackTrack>& : 减数向量;
 * @return std::vector<BytetrackTrack> : 差集结果;
 */
inline std::vector<BytetrackTrack> sub_stracks(std::vector<BytetrackTrack>& track_list_a,  //
                                               std::vector<BytetrackTrack>& track_list_b)  //
{
    std::map<int32, int32> track_list_b_map;
    for (size_t i = 0; i < track_list_b.size(); i++)
        track_list_b_map[track_list_b[i].track_id] = 1;

    std::vector<BytetrackTrack> res;
    for (size_t i = 0; i < track_list_a.size(); i++)
    {
        if (track_list_b_map.find(track_list_a[i].track_id) == track_list_b_map.end())
            res.push_back(track_list_a[i]);
    }
    return res;
}

/***
 * @description: 移除两个轨迹列表中的重复轨迹;
 *               基于 IoU 距离检测重复, 保留存在时间更长的轨迹;
 *
 *               为什么会有重复轨迹?
 *               如果某个目标被检测器连续检测到, 但中途短暂丢失后又重新出现,
 *               可能会产生两个 track_id 不同的轨迹实际上跟踪的是同一个目标.
 *               remove_duplicate_stracks 会通过 IoU 重叠检测这些重复,
 *               并保留"年龄"更大的那个轨迹;
 *
 * @param resa     std::vector<BytetrackTrack>& : 输出, tracks_a 的非重复结果;
 * @param resb     std::vector<BytetrackTrack>& : 输出, tracks_b 的非重复结果;
 * @param tracks_a std::vector<BytetrackTrack>& : 第一个输入轨迹列表;
 * @param tracks_b std::vector<BytetrackTrack>& : 第二个输入轨迹列表;
 */
inline void remove_duplicate_stracks(std::vector<BytetrackTrack>& resa,      //
                                     std::vector<BytetrackTrack>& resb,      //
                                     std::vector<BytetrackTrack>& tracks_a,  //
                                     std::vector<BytetrackTrack>& tracks_b)  //
{
    // ---- 计算 IoU 距离矩阵 ----
    std::vector<std::vector<float32>> dists = cal_iou_distance(tracks_a, tracks_b);
    // 重复检测的 IoU 阈值: 如果 IoU > 0.85 (1 - 0.15), 认为两个轨迹跟踪的是同一目标;
    constexpr float32 DUP_IOU_THRESH = 0.15f;

    // ---- 找出所有重复对 ----
    std::vector<std::pair<int32, int32>> dup_pairs;
    for (size_t i = 0; i < dists.size(); i++)
    {
        for (size_t j = 0; j < dists[i].size(); j++)
        {
            if (dists[i][j] < (1.0f - DUP_IOU_THRESH))
            {
                // 轨迹 i 和轨迹 j 高度重叠, 可能是重复;
                dup_pairs.push_back(std::make_pair(static_cast<int32>(i), static_cast<int32>(j)));
            }
        }
    }

    // ---- 标记需要删除的轨迹 ----
    std::vector<bool> dup_a(tracks_a.size(), false);
    std::vector<bool> dup_b(tracks_b.size(), false);

    for (size_t k = 0; k < dup_pairs.size(); k++)
    {
        int32 i = dup_pairs[k].first;
        int32 j = dup_pairs[k].second;
        // 比较轨迹的生命周期长度 (frame_id - start_frame);
        int32 time_a = tracks_a[i].frame_id - tracks_a[i].start_frame;
        int32 time_b = tracks_b[j].frame_id - tracks_b[j].start_frame;
        // 保留生存时间更长的轨迹, 删除生存时间更短的;
        if (time_a > time_b)
            dup_b[j] = true;
        else
            dup_a[i] = true;
    }

    // ---- 构建输出: 只保留非重复的轨迹 ----
    for (size_t i = 0; i < tracks_a.size(); i++)
        if (!dup_a[i])
            resa.push_back(tracks_a[i]);
    for (size_t j = 0; j < tracks_b.size(); j++)
        if (!dup_b[j])
            resb.push_back(tracks_b[j]);
}

}  // namespace bytetrack
}  // namespace tracker

#endif  // !__BYTETRACK_MATCHING__H__