/***
 * @Author       : gxs
 * @Date         : 2026-06-21 16:37:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-21 16:37:00
 * @FilePath     : /visionAlgorithm/lib/tracker/TrackState.hpp
 * @Description  : 各跟踪算法的轨迹状态枚举定义 + 枚举转字符串函数
 *                 每种算法的状态枚举按各自语义独立定义, 互不干扰;
 *                 新增算法时, 只需在此文件中添加对应的枚举定义和转换函数;
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __TRACKSTATE__H__
#define __TRACKSTATE__H__

#include <string>

#include "types.hpp"

namespace tracker
{

// ====================================================================
// !SORT 轨迹状态枚举
// ====================================================================
/***
 * @description: SORT 轨迹状态枚举
 * SORT 是最基础的跟踪器, 仅有确认和丢失两种状态;
 */
enum class SortState : uint8
{
    Confirmed = 0,  // 已确认轨迹
    Lost            // 丢失轨迹
};

/***
 * @description: SortState -> 字符串
 * @param state SortState : SORT 轨迹状态枚举
 * @return string : 对应的字符串, 未知枚举返回 "UnknownSortState"
 */
inline std::string to_string(SortState state)
{
    switch (state)
    {
        case SortState::Confirmed:
            return "SortState::Confirmed";
        case SortState::Lost:
            return "SortState::Lost";
        default:
            return "UnknownSortState";
    }
}

// ====================================================================
// !DeepSORT 轨迹状态枚举
// ====================================================================
/***
 * @description: DeepSORT 轨迹状态枚举
 * 在 SORT 基础上增加 Tentative 状态, 需要连续匹配 n_init 帧才能转为 Confirmed;
 */
enum class DeepSortState : uint8
{
    Tentative = 0,  // 未确认轨迹, 初始状态, 需匹配 n_init 帧后转为 Confirmed
    Confirmed,      // 已确认轨迹
    Deleted         // 已删除轨迹
};

/***
 * @description: DeepSortState -> 字符串
 * @param state DeepSortState : DeepSORT 轨迹状态枚举
 * @return string : 对应的字符串, 未知枚举返回 "UnknownDeepSortState"
 */
inline std::string to_string(DeepSortState state)
{
    switch (state)
    {
        case DeepSortState::Tentative:
            return "DeepSortState::Tentative";
        case DeepSortState::Confirmed:
            return "DeepSortState::Confirmed";
        case DeepSortState::Deleted:
            return "DeepSortState::Deleted";
        default:
            return "UnknownDeepSortState";
    }
}

// ====================================================================
// !ByteTrack 轨迹状态枚举
// ====================================================================
/***
 * @description: ByteTrack 轨迹状态枚举
 * 状态生命周期: New -> Tracked <-> Lost -> Removed
 */
enum class ByteTrackState : uint8
{
    New = 0,  // 新建轨迹, 未激活(仅一帧)
    Tracked,  // 正常跟踪中, 连续匹配成功
    Lost,     // 暂时丢失, 等待重连
    Removed   // 已移除, 超过 max_age 仍未找回
};

/***
 * @description: ByteTrackState -> 字符串
 * @param state ByteTrackState : ByteTrack 轨迹状态枚举
 * @return string : 对应的字符串, 未知枚举返回 "UnknownByteTrackState"
 */
inline std::string to_string(ByteTrackState state)
{
    switch (state)
    {
        case ByteTrackState::New:
            return "ByteTrackState::New";
        case ByteTrackState::Tracked:
            return "ByteTrackState::Tracked";
        case ByteTrackState::Lost:
            return "ByteTrackState::Lost";
        case ByteTrackState::Removed:
            return "ByteTrackState::Removed";
        default:
            return "UnknownByteTrackState";
    }
}

// ====================================================================
// !OC_SORT 轨迹状态枚举
// ====================================================================
/***
 * @description: OC_SORT 轨迹状态枚举
 * 在 ByteTrack 基础上增加了 Occluded 状态, 用于标记被遮挡的轨迹;
 */
enum class OCSortState : uint8
{
    New = 0,   // 新建轨迹
    Tracked,   // 正常跟踪
    Occluded,  // 被遮挡(OC_SORT 特有)
    Lost,      // 丢失
    Removed    // 已移除
};

/***
 * @description: OCSortState -> 字符串
 * @param state OCSortState : OC_SORT 轨迹状态枚举
 * @return string : 对应的字符串, 未知枚举返回 "UnknownOCSortState"
 */
inline std::string to_string(OCSortState state)
{
    switch (state)
    {
        case OCSortState::New:
            return "OCSortState::New";
        case OCSortState::Tracked:
            return "OCSortState::Tracked";
        case OCSortState::Occluded:
            return "OCSortState::Occluded";
        case OCSortState::Lost:
            return "OCSortState::Lost";
        case OCSortState::Removed:
            return "OCSortState::Removed";
        default:
            return "UnknownOCSortState";
    }
}

}  // namespace tracker

#endif  // !__TRACKSTATE__H__