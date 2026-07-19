/***
 * @Author       : gxs
 * @Date         : 2026-07-19 18:00:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-19 18:00:00
 * @FilePath     : /visionAlgorithm/lib/tracker/TrackResult.hpp
 * @Description  : 单帧跟踪结果数据结构;
 *
 *                 TrackerRuntime::update() 每帧返回此类型的 vector;
 *                 同时作为各跟踪器 update() 的输出参数, 由跟踪器内部填充;
 *
 *                 核心设计:
 *                 跟踪器内部在匹配阶段直接建立 track_id ↔ BoxObject (det_index) 的映射,
 *                 不再依赖外部的 ltwh 浮点比较回溯;
 *
 *                 det_index 含义:
 *                 - >= 0 : 本帧匹配到了检测框, 此索引对应调用方传入的 BoxObject 列表位置;
 *                          在 TrackerRuntime 中, 经过排序回映射后, 指向原始 YoloObject 列表;
 *                 - -1   : 本帧没有匹配 (DeepSORT 已确认轨迹的纯卡尔曼预测帧);
 *
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __TRACK_RESULT__H__
#define __TRACK_RESULT__H__

#include "types.hpp"  // float32, int32

namespace tracker
{

/***
 * @description: 单帧跟踪结果;
 *
 *               由跟踪器 update() 填充, 每个元素对应当前帧一条活跃轨迹的摘要;
 *               TrackerRuntime::update() 返回此类型的 vector;
 *
 *               关键字段说明:
 *               - track_id  : 全局唯一轨迹 ID;
 *               - det_index : 在 BoxObject 输入列表中的索引;
 *                             TrackerRuntime 会将此索引回映射到原始 YoloObject 列表;
 *               - ltwh      : 来自检测器的原始框坐标 (未经卡尔曼修正);
 */
struct TrackResult
{
    // track_id: 轨迹唯一 ID (全局唯一, 整个视频生命周期内不重复);
    int32 track_id = -1;

    // det_index: 在本帧输入 BoxObject 列表中的索引;
    // 由跟踪器内部在匹配阶段直接赋值, 无需外部 ltwh 比较;
    // >= 0 : 本帧匹配到了检测框;
    // -1   : 本帧无匹配 (仅 DeepSORT 已确认轨迹在丢帧时会出现);
    int32 det_index = -1;

    // cls_id: 检测框类别 ID (来自最近一次匹配的检测框);
    int32 cls_id = -1;

    // score: 检测框置信度分数;
    float32 score = 0.0f;

    // ltwh: 原始检测框 [left, top, width, height];
    // 来自检测器的原始输出, 未经卡尔曼修正;
    float32 ltwh[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

}  // namespace tracker

#endif  // !__TRACK_RESULT__H__
