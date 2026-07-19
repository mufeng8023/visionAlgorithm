/***
 * @Author       : gxs
 * @Date         : 2026-07-19 17:00:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-19 18:00:00
 * @FilePath     : /visionAlgorithm/lib/tracker/TrackerRuntime.hpp
 * @Description  : 跟踪器运行时 (高级接口);
 *
 *                 ============================================================
 *                 设计目标
 *                 ============================================================
 *                 将底层跟踪算法 (ByteTrack / DeepSORT) 封装为统一运行时;
 *                 上层应用无需关心具体的跟踪算法细节, 只需:
 *                   1. 调用 init() 加载 ini 配置文件, 自动初始化跟踪器;
 *                   2. 每帧调用 update() 传入 YoloObject 检测结果;
 *                   3. 通过返回的 TrackResult 列表, 获取:
 *                      - 每个活跃目标的轨迹 ID (track_id);
 *                      - 该目标在本帧检测列表中的索引 (det_index);
 *                   4. 通过 get_histories() 获取所有轨迹的历史信息;
 *
 *                 ============================================================
 *                 det_index 回溯机制
 *                 ============================================================
 *                 update() 内部先对 YoloObject 按置信度排序, 再转换为 BoxObject 送入跟踪器;
 *                 跟踪器内部在匹配阶段直接记录 track_id → BoxObject 下标映射 (track_det_map);
 *                 排序后的下标由 sorted_indices 回映射至原始 detections[] 下标;
 *                 因此 TrackResult.det_index 直接对应调用方的 detections[det_index];
 *
 *                 ============================================================
 *                 TrackHistory 维护机制
 *                 ============================================================
 *                 内部维护一个 map<track_id, TrackHistory>;
 *                 每帧 update() 后自动:
 *                   - 将本帧活跃轨迹的检测信息追加到对应 TrackHistory 的 frames 队列;
 *                   - 超出 max_history_frames 时滚动删除最旧帧;
 *                   - 将失活轨迹的 is_active 标记为 false;
 *                   - 若本帧有真实检测匹配, 更新 TrackHistory.last_detection (完整 YoloObject);
 *
 *                 典型应用场景:
 *                   - 客流量统计: 判断轨迹是否穿过计数线;
 *                   - 行为分析:  对轨迹位置序列计算速度/方向/停留时长;
 *                   - 轨迹可视化: 绘制每个目标的历史运动路径;
 *
 *                 ============================================================
 *                 支持的跟踪器
 *                 ============================================================
 *                 - bytetrack  : ByteTrack (推荐, 无需 ReID 模型, 速度快, 鲁棒性强);
 *                 - deepsort   : DeepSORT (需要 ReID 特征, 精度更高, 适合需要外观匹配的场景);
 *                 - sort       : 占位, 暂未实现;
 *                 - ocsort     : 占位, 暂未实现;
 *
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __TRACKER_RUNTIME__H__
#define __TRACKER_RUNTIME__H__

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "detector/YoloObject.h"
#include "tracker/BoxObject.hpp"
#include "tracker/TrackHistory.hpp"
#include "tracker/TrackerConfig.hpp"
#include "tracker/bytetrack/ByteTracker.hpp"
#include "tracker/deepsort/DeepSortTracker.hpp"
#include "tracker/utils.hpp"

namespace tracker
{

/***
 * @description: 跟踪器运行时;
 *
 *               封装 ByteTrack 和 DeepSORT, 提供统一的高级接口;
 *               根据 ini 配置文件自动选择并初始化跟踪器;
 *
 *               核心接口:
 *                 init()          : 加载 ini 文件, 创建跟踪器实例;
 *                 update()        : 每帧输入 YoloObject 检测结果, 返回 TrackResult;
 *                 get_histories() : 返回所有轨迹的历史记录 map;
 *                 reset()         : 重置跟踪器和历史记录;
 *
 *               成员变量布局 (遵循 C++ 类规范, 变量定义在最前面):
 */
class TrackerRuntime
{
   public:
    // ================================================================
    // 公开成员变量
    // ================================================================

    // _config: 从 ini 文件解析的跟踪器配置;
    TrackerConfig _config;

    // _current_frame_id: 当前帧 ID (每次 update 时更新);
    int32 _current_frame_id = 0;

    // _histories: 轨迹历史 map;
    // key = track_id, value = TrackHistory (含完整历史帧列表 + 最新 YoloObject);
    // 轨迹一旦创建就会持续保留 (即使消失), is_active 标记活跃状态;
    // 应用层可以遍历此 map 实现客流统计、轨迹分析等功能;
    std::map<int32, TrackHistory> _histories;

    // _max_history_frames: 每条轨迹最多保留的历史帧数 (滚动窗口);
    // 超出时自动删除最旧帧, 防止内存无限增长;
    // 默认 150 帧 (约 5 秒 @30fps);
    int32 _max_history_frames = 150;

   private:
    // ================================================================
    // 私有成员变量
    // ================================================================

    // _byte_tracker: ByteTrack 实例 (tracker_type == bytetrack 时创建);
    std::unique_ptr<bytetrack::ByteTracker> _byte_tracker;

    // _deepsort_tracker: DeepSORT 实例 (tracker_type == deepsort 时创建);
    std::unique_ptr<deepsort::DeepSORTTracker> _deepsort_tracker;

    // _is_initialized: 是否已完成初始化;
    bool _is_initialized = false;

   public:
    // ================================================================
    // 构造函数 / 析构函数
    // ================================================================

    /***
     * @description: 默认构造函数;
     *               构造后需调用 init() 才能使用;
     */
    TrackerRuntime() = default;

    /***
     * @description: 析构函数;
     */
    ~TrackerRuntime() = default;

    // ================================================================
    // 核心公开接口
    // ================================================================

    /***
     * @description: 初始化跟踪器运行时;
     *               从 ini 配置文件解析跟踪器参数, 自动创建对应的跟踪器实例;
     *
     *               ini 文件格式 (参考 config/tracker/bytetrack.ini):
     *               [track]
     *               tracker_type = bytetrack  ; 或 deepsort
     *               track_thresh = 0.5
     *               ...
     *
     * @param ini_path           const std::string& : ini 配置文件路径;
     * @param max_history_frames int32              : 每条轨迹最多保留的历史帧数, 默认 150;
     * @throws std::runtime_error : ini 文件加载失败时抛出;
     * @throws std::runtime_error : 不支持的跟踪器类型时抛出;
     */
    void init(const std::string& ini_path, int32 max_history_frames = 150)
    {
        this->_max_history_frames = max_history_frames;

        // 从 ini 文件解析配置 (tracker/utils.hpp 中定义);
        parser_ini_tracker_config(ini_path, this->_config);

        // 根据 tracker_type 创建对应的跟踪器实例;
        if (this->_config.tracker_type == TrackerType::bytetrack)
        {
            this->_byte_tracker = std::unique_ptr<bytetrack::ByteTracker>(new bytetrack::ByteTracker(this->_config));
            LOG_DEFAULT_INFO("TrackerRuntime: ByteTrack initialized;");
        }
        else if (this->_config.tracker_type == TrackerType::deepsort)
        {
            this->_deepsort_tracker =
                std::unique_ptr<deepsort::DeepSORTTracker>(new deepsort::DeepSORTTracker(this->_config));
            LOG_DEFAULT_INFO("TrackerRuntime: DeepSORT initialized;");
        }
        else
        {
            // sort 和 ocsort 暂未实现, 抛出运行时错误;
            std::string type_name = tracker_type_to_string(this->_config.tracker_type);
            LOG_DEFAULT_ERROR("TrackerRuntime: unsupported tracker type: %s;", type_name.c_str());
            throw std::runtime_error("TrackerRuntime: unsupported tracker type: " + type_name);
        }

        this->_is_initialized = true;
        LOG_DEFAULT_INFO("TrackerRuntime: init done, max_history_frames=%d;", max_history_frames);
    }

    /***
     * @description: 每帧更新跟踪器;
     *               将 YoloObject 检测结果送入跟踪器进行多目标关联;
     *               返回本帧所有活跃轨迹的 TrackResult 列表;
     *               同时自动更新内部的 TrackHistory 记录;
     *
     *               返回值说明:
     *               - 每个 TrackResult 对应一条活跃轨迹;
     *               - TrackResult.det_index >= 0 : 该轨迹本帧匹配到了检测框;
     *               - TrackResult.det_index == -1 : 该轨迹本帧无匹配 (DeepSORT 预测帧);
     *
     *               使用示例:
     *               std::vector<TrackResult> results = runtime.update(yolo_dets, frame_id);
     *               for (const TrackResult& r : results) {
     *                   if (r.det_index >= 0) {
     *                       // r.det_index 对应 yolo_dets[r.det_index]
     *                       // r.track_id 是全局唯一轨迹 ID
     *                   }
     *               }
     *
     * @param detections const std::vector<yolo::YoloObject>& : 当前帧检测结果;
     * @param frame_id   int32 : 当前帧 ID (-1 表示自增);
     * @return std::vector<TrackResult> : 本帧活跃轨迹结果列表;
     * @throws std::runtime_error : 未调用 init() 时抛出;
     */
    std::vector<TrackResult> update(const std::vector<yolo::YoloObject>& detections, int32 frame_id = -1)
    {
        if (!this->_is_initialized)
        {
            throw std::runtime_error("TrackerRuntime: not initialized, call init() first;");
        }

        // 帧 ID 管理: 负数时自增;
        if (frame_id > 0)
        {
            this->_current_frame_id = frame_id;
        }
        else
        {
            this->_current_frame_id++;
        }

        // 按置信度从高到低排序, 记录原始下标;
        // sorted_indices[i] = 排序后第 i 个检测在原始 detections[] 中的位置;
        std::vector<int32> sorted_indices(detections.size());
        for (int32 i = 0; i < static_cast<int32>(detections.size()); i++)
        {
            sorted_indices[static_cast<size_t>(i)] = i;
        }
        std::sort(
            sorted_indices.begin(), sorted_indices.end(), [&detections](int32 a, int32 b) -> bool
            { return detections[static_cast<size_t>(a)].box.score > detections[static_cast<size_t>(b)].box.score; });

        // 按排序后顺序构建 BoxObject 列表;
        // box_objects[i] 对应原始 detections[sorted_indices[i]];
        std::vector<BoxObject> box_objects;
        box_objects.reserve(detections.size());
        for (size_t i = 0; i < sorted_indices.size(); i++)
        {
            const yolo::Box& b = detections[static_cast<size_t>(sorted_indices[i])].box;
            // xyxy → ltwh 转换: [x1, y1, x2-x1, y2-y1];
            float32 ltwh[4] = {b.x1, b.y1, b.x2 - b.x1, b.y2 - b.y1};
            BoxObject box(ltwh, b.score, static_cast<int32>(b.cls_id), this->_config.expand_box_rate);
            box_objects.push_back(box);
        }

        // 调用跟踪器 update(); 结果中 det_index 指向排序后 box_objects 的下标;
        std::vector<TrackResult> results;
        if (this->_config.tracker_type == TrackerType::bytetrack)
        {
            this->_byte_tracker->update(box_objects, results, this->_current_frame_id);
        }
        else if (this->_config.tracker_type == TrackerType::deepsort)
        {
            this->_deepsort_tracker->update(box_objects, results, this->_current_frame_id);
        }

        // 将 det_index 从排序后下标回映射到原始 detections[] 下标;
        // 回映射后 results[i].det_index 直接对应调用方的 detections[det_index];
        for (size_t i = 0; i < results.size(); i++)
        {
            if (results[i].det_index >= 0)
            {
                results[i].det_index = sorted_indices[static_cast<size_t>(results[i].det_index)];
            }
        }

        // 更新轨迹历史记录 (传入原始未排序检测列表, 此时 det_index 已完成回映射);
        this->_update_histories(results, detections, this->_current_frame_id);

        return results;
    }

    /***
     * @description: 获取所有轨迹的历史记录;
     *               返回内部 _histories map 的常量引用;
     *               key = track_id, value = TrackHistory (含完整历史帧列表 + 最新 YoloObject);
     *
     *               注意: 历史记录中包含所有曾经出现过的轨迹 (含已消失的);
     *               通过 TrackHistory.is_active 可以区分活跃轨迹和已消失轨迹;
     *
     * @return const std::map<int32, TrackHistory>& : 只读轨迹历史 map;
     */
    const std::map<int32, TrackHistory>& get_histories() const { return this->_histories; }

    /***
     * @description: 获取当前帧活跃轨迹数量;
     *               快速统计当前帧可见目标数量 (用于调试或状态监控);
     * @return size_t : 活跃轨迹数量;
     */
    size_t active_track_count() const
    {
        size_t cnt = 0;
        for (std::map<int32, TrackHistory>::const_iterator it = this->_histories.begin(); it != this->_histories.end();
             ++it)
        {
            if (it->second.is_active)
            {
                cnt++;
            }
        }
        return cnt;
    }

    /***
     * @description: 获取历史上出现过的轨迹总数;
     *               即自 reset() 以来累计出现过的唯一轨迹数量;
     *               可用于统计"通过某区域的目标总数";
     * @return size_t : 历史总轨迹数量;
     */
    size_t total_track_count() const { return this->_histories.size(); }

    /***
     * @description: 重置跟踪器和所有历史记录;
     *               清空轨迹历史, 重置帧计数器;
     *               通常在视频序列切换时调用;
     */
    void reset()
    {
        if (this->_byte_tracker)
        {
            this->_byte_tracker->reset();
        }
        if (this->_deepsort_tracker)
        {
            this->_deepsort_tracker->reset();
        }
        this->_histories.clear();
        this->_current_frame_id = 0;

        LOG_DEFAULT_INFO("TrackerRuntime: reset done;");
    }

    /***
     * @description: 获取当前跟踪器配置 (只读);
     * @return const TrackerConfig& : 当前配置;
     */
    const TrackerConfig& config() const { return this->_config; }

    /***
     * @description: 判断运行时是否已初始化;
     * @return bool : true 表示已初始化;
     */
    bool is_initialized() const { return this->_is_initialized; }

   private:
    // ================================================================
    // 私有辅助方法
    // ================================================================

    /***
     * @description: 更新轨迹历史记录;
     *               将本帧的 TrackResult 列表写入对应的 TrackHistory;
     *               同时将上帧活跃但本帧消失的轨迹标记为 is_active = false;
     *
     *               更新流程:
     *               将所有已有历史的 is_active 标记为 false;
     *               遍历本帧 TrackResult, 更新或创建对应的 TrackHistory;
     *               追加 TrackHistoryFrame (仅含原始 ltwh) 到 frames 队列;
     *               若本帧有真实检测匹配 (det_index >= 0), 更新 last_detection;
     *               超出 _max_history_frames 时滚动删除最旧帧;
     *
     * @param results    const std::vector<TrackResult>&      : 本帧跟踪结果;
     * @param detections const std::vector<yolo::YoloObject>& : 本帧原始检测列表 (未排序);
     * @param frame_id   int32                                : 当前帧 ID;
     */
    void _update_histories(const std::vector<TrackResult>& results,          //
                           const std::vector<yolo::YoloObject>& detections,  //
                           int32 frame_id)
    {
        // 将所有已有历史的轨迹标记为失活 (后续再按本帧结果更新活跃状态);
        for (std::map<int32, TrackHistory>::iterator it = this->_histories.begin(); it != this->_histories.end(); ++it)
        {
            it->second.is_active = false;
        }

        // 遍历本帧活跃轨迹, 更新或创建对应的 TrackHistory;
        for (size_t i = 0; i < results.size(); i++)
        {
            const TrackResult& result = results[i];
            int32 tid = result.track_id;

            // 获取或创建 TrackHistory (若轨迹第一次出现, 默认构造一个新的);
            TrackHistory& history = this->_histories[tid];
            history.track_id = tid;
            history.cls_id = result.cls_id;
            history.is_active = true;

            // 构建本帧的 TrackHistoryFrame (仅保存原始检测框, 不存储卡尔曼修正框);
            TrackHistoryFrame hframe;
            hframe.frame_id = frame_id;
            hframe.det_index = result.det_index;
            hframe.cls_id = result.cls_id;
            hframe.score = result.score;
            hframe.ltwh[0] = result.ltwh[0];
            hframe.ltwh[1] = result.ltwh[1];
            hframe.ltwh[2] = result.ltwh[2];
            hframe.ltwh[3] = result.ltwh[3];

            // 追加到历史队列末尾 (最新帧在 back());
            history.frames.push_back(hframe);

            // 超出滚动窗口时, 删除最旧帧 (最旧帧在 front());
            while (static_cast<int32>(history.frames.size()) > this->_max_history_frames)
            {
                history.frames.pop_front();
            }

            // 若本帧匹配到真实检测框 (det_index >= 0), 更新 last_detection;
            // 纯卡尔曼预测帧 (det_index == -1) 不更新, 保持上一次匹配时的值;
            if (result.det_index >= 0 && result.det_index < static_cast<int32>(detections.size()))
            {
                history.last_detection = detections[static_cast<size_t>(result.det_index)];
            }
        }
    }
};

}  // namespace tracker

#endif  // !__TRACKER_RUNTIME__H__
