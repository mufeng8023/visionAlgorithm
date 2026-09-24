/***
 * @Author       : gxs
 * @Date         : 2026-07-19 17:00:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-19 17:00:00
 * @FilePath     : /visionAlgorithm/lib/tracker/TrackHistory.hpp
 * @Description  : 跟踪历史数据结构定义;
 *
 *                 设计目标:
 *                 为上层应用 (客流量统计, 行为分析, 轨迹回溯等) 提供跨帧目标关联数据;
 *                 TrackerMiddleware 在每帧更新后将结果填充到此结构中,
 *                 应用层可直接读取 TrackHistory 实现各类高级分析;
 *
 *                 核心数据流:
 *                 检测器输出 YoloObject → TrackerMiddleware 转换 + 跟踪 →
 *                 TrackResult (单帧结果) → TrackHistory (多帧累积)
 *
 *                 数据结构层次:
 *                   TrackHistoryFrame : 轨迹在某一帧的单帧记录 (含 det_index + 原始 ltwh);
 *                   TrackHistory      : 某条轨迹的完整历史帧列表 (deque 滚动窗口) + 最新帧 YoloObject;
 *                   TrackResult       : 某帧所有活跃轨迹的简洁摘要 (供应用层快速遍历);
 *
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __TRACK_HISTORY__H__
#define __TRACK_HISTORY__H__

#include <deque>
#include <vector>

#include "detector/YoloObject.h"    // yolo::YoloObject (含 Box, KeyPoint, mask 等完整检测信息)
#include "tracker/TrackResult.hpp"  // TrackResult (单帧跟踪结果, 供 update() 输出使用)
#include "types.hpp"                // float32, int32

namespace tracker
{

/***
 * @description: 轨迹在某一帧的单帧检测记录;
 *
 *               保存某条轨迹在某帧中对应的位置信息与索引;
 *               历史队列仅记录原始检测框坐标, 不存储卡尔曼修正框,
 *               以减少内存占用并保留检测器原始数据;
 *               当该帧没有匹配到检测 (纯卡尔曼预测) 时, det_index = -1;
 */
struct TrackHistoryFrame
{
    // frame_id: 该帧的帧号 (由上层传入或自增);
    int32 frame_id = 0;

    // det_index: 在该帧输入检测列表 (YoloObject 或 BoxObject) 中的索引;
    // >= 0 : 本帧匹配到了检测框, 此索引对应输入列表中的位置;
    // -1   : 本帧没有匹配到检测 (纯卡尔曼预测, 仅 DeepSORT 会出现此情况);
    int32 det_index = -1;

    // cls_id: 检测框类别 ID;
    // 由最近一次匹配到的检测框更新, 纯预测帧使用上一帧的值;
    int32 cls_id = -1;

    // score: 检测框置信度分数;
    // 由最近一次匹配到的检测框更新;
    float32 score = 0.0f;

    // ltwh: 原始检测框 [left, top, width, height];
    // 来自检测器的原始输出, 未经卡尔曼修正;
    float32 ltwh[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

/***
 * @description: 某条轨迹的完整历史记录;
 *
 *               跟踪器为每条分配了 track_id 的轨迹维护一个 TrackHistory;
 *               历史帧以 deque 形式存储, 超出 max_history_frames 时自动滚动删除最旧帧;
 *
 *               典型应用:
 *               - 客流量统计: 判断轨迹是否穿过统计线 (用 frames 中的位置序列判断);
 *               - 行为分析:  对 frames 中的位置序列做速度/加速度/方向分析;
 *               - 轨迹回溯:  可视化某段时间内目标的移动路径;
 */
struct TrackHistory
{
    // track_id: 轨迹唯一 ID, 整个视频生命周期内全局唯一, 不重复;
    int32 track_id = -1;

    // cls_id: 最新类别 ID (每帧匹配后更新为当前帧的类别);
    int32 cls_id = -1;

    // is_active: 当前帧该轨迹是否仍然活跃 (出现在跟踪器的输出中);
    // true  : 当前帧可见 (被跟踪器输出);
    // false : 已消失 (丢失/超时删除), 但历史仍保留以供分析;
    bool is_active = false;

    // frames: 按时间顺序排列的历史帧记录 (从旧到新);
    // front() = 最旧帧, back() = 最新帧;
    // 使用 deque 实现 O(1) 的首尾插入/删除;
    std::deque<TrackHistoryFrame> frames;

    // last_detection: 该轨迹最近一次匹配到检测时的完整 YoloObject;
    // 每当 det_index >= 0 时更新 (即本帧有真实检测匹配时);
    // 保留完整 YoloObject 信息 (Box + kpts + mask + angle 等),
    // 供上层应用 (姿态估计, 分割等) 直接访问最新检测结果;
    // 注: 纯卡尔曼预测帧不更新此字段, 保持上一次匹配时的值;
    yolo::YoloObject last_detection;
};

}  // namespace tracker

#endif  // !__TRACK_HISTORY__H__
