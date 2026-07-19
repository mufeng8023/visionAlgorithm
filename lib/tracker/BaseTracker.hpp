/***
 * @Author       : gxs
 * @Date         : 2026-06-22 21:37:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 21:37:00
 * @FilePath     : /visionAlgorithm/lib/tracker/BaseTracker.hpp
 * @Description  : 跟踪器抽象基类定义;
 *                 所有跟踪器的公共接口和公共成员;
 *                 DeepSORT / ByteTrack / SORT / OC_SORT 均继承此类;
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __BASETRACKER__H__
#define __BASETRACKER__H__

#include <memory>  // std::shared_ptr
#include <vector>

#include "tracker/BoxKalmanFilter.hpp"
#include "tracker/BoxObject.hpp"
#include "tracker/TrackResult.hpp"
#include "tracker/TrackerConfig.hpp"

namespace tracker
{

/***
 * @description: 跟踪器抽象基类;
 *
 * 所有跟踪器 (DeepSORT / ByteTrack / SORT / OC_SORT) 的公共接口;
 * 子类必须实现 update() 方法;
 *
 * 公共成员:
 *   _frame_id        : 当前帧 ID;
 *   _next_id         : 下一个轨迹 ID (自增);
 *   _kalman_filter   : 卡尔曼滤波器实例 (所有轨迹共享);
 *   _config          : 跟踪器配置;
 *
 * =====================================================================
 * 各跟踪器核心差异速览
 * =====================================================================
 *
 *  ┌────────────┬──────────────────────────────────────────────────┐
 *  │ Tracker    │ 匹配策略                                         │
 *  ├────────────┼──────────────────────────────────────────────────┤
 *  │ SORT       │ 单次 IoU 关联 (卡尔曼预测 + 匈牙利匹配)           │
 *  │ DeepSORT   │ 级联匹配 (马氏距离 + 余弦距离) + 二次 IoU 关联   │
 *  │ ByteTrack  │ 两次 IoU 关联 (高分检测优先, 低分检测兜底)        │
 *  │ OC_SORT    │ ByteTrack 基础上增加观测置信度修正                │
 *  └────────────┴──────────────────────────────────────────────────┘
 * =====================================================================
 */
class BaseTracker
{
   public:
    // _frame_id: 当前帧 ID (每调用一次 update 自增);
    int32 _frame_id = 0;

    // _next_id: 下一个轨迹 ID (每次创建新轨迹时自增);
    // 从 1 开始, 与 DeepSORT 原版 _next_idx 保持一致;
    int32 _next_id = 1;

    // _kalman_filter: KFBox 卡尔曼滤波器实例;
    // 注意! DeepSORT 是"所有轨迹共享同一个 KFBox 对象"(指针传递);
    // ByteTrack 是"每个轨迹持有自己的 KFBox 结果副本";
    // 虽然用法不同, 但 KFBox 对象本身是无状态的, 两种方式都正确;
    KFBox _kalman_filter;

    // _config: 跟踪器配置 (从 ini 文件读取);
    // 包含 track_thresh, high_thresh, match_thresh, max_age, n_init 等参数;
    TrackerConfig _config;

   public:
    /***
     * @description: 构造函数;
     *               接收配置参数并初始化基类成员;
     *               @note 子类构造函数中无需再初始化这些成员;
     * @param config const TrackerConfig& : 跟踪器配置 (from ini or hard-code);
     */
    explicit BaseTracker(const TrackerConfig& config) : _config(config)
    {
        // 构造函数体为空, 所有初始化在初始化列表中完成;
    }

    /***
     * @description: 虚析构函数;
     *               确保派生类对象通过基类指针删除时能正确调用派生类析构函数;
     */
    virtual ~BaseTracker() = default;

    /***
     * @description: 更新跟踪器 (纯虚函数);
     *               子类必须实现具体的跟踪逻辑;
     *               每帧调用一次, 输入当前帧的检测结果, 更新内部轨迹状态;
     *               跟踪结果通过 results 输出, 每条记录包含 track_id 和对应的 det_index;
     *
     *               @note 各跟踪器的 update 内部流程不同:
     *               - ByteTrack: 三轮 IoU 关联, 在匹配阶段直接建立 track_id ↔ det_index 映射;
     *               - DeepSORT: 级联匹配 + IoU 二次匹配, 同样直接建立映射;
     *
     *               det_index 含义:
     *               - >= 0 : 本帧匹配到了 detections[det_index] 这个检测框;
     *               - -1   : 本帧无匹配 (DeepSORT 已确认轨迹的纯卡尔曼预测帧);
     *
     * @param detections const std::vector<BoxObject>& : 当前帧检测结果 (调用方负责按分数排序);
     * @param results    std::vector<TrackResult>&     : 输出参数, 本帧活跃轨迹结果;
     * @param frame_id   int32 : 当前帧 ID (默认 -1 表示自增);
     */
    virtual void update(const std::vector<BoxObject>& detections,  //
                        std::vector<TrackResult>& results,         //
                        int32 frame_id = -1) = 0;

    /***
     * @description: 重置跟踪器状态;
     *               清空所有轨迹, 重置帧计数器和轨迹 ID 计数器;
     *               通常在视频序列切换或跟踪异常恢复时调用;
     */
    virtual void reset()
    {
        this->_frame_id = 0;
        this->_next_id = 1;
    }

    /***
     * @description: 分配并返回下一个轨迹 ID;
     *               每次调用自增 _next_id, 保证每个轨迹的 ID 唯一;
     *               参考 DeepSORT 的 _next_idx 机制;
     * @return int32 : 新分配的轨迹 ID;
     */
    inline int32 next_track_id() { return this->_next_id++; }

    /***
     * @description: 获取当前帧 ID;
     *               @note 初始值为 0, 第一帧 update 后变为 1;
     * @return int32 : 当前帧 ID;
     */
    inline int32 frame_id() const { return this->_frame_id; }

    /***
     * @description: 获取卡尔曼滤波器引用;
     *               用于 predict / update / gating_distance 等操作;
     * @return KFBox& : 当前跟踪器持有的 KFBox 实例引用;
     */
    inline KFBox& kalman_filter() { return this->_kalman_filter; }

    /***
     * @description: 获取跟踪器配置;
     *               外部可通过此接口读取跟踪参数;
     * @return const TrackerConfig&;
     */
    inline const TrackerConfig& config() const { return this->_config; }
};

}  // namespace tracker

#endif  // !__BASETRACKER__H__