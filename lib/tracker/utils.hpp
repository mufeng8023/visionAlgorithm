/***
 * @Author       : gxs
 * @Date         : 2026-06-21 16:07:28
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-21 16:07:28
 * @FilePath     : /visionAlgorithm/lib/tracker/utils.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __UTILS__H__
#define __UTILS__H__

#include "tracker/TrackerConfig.hpp"

namespace tracker
{

/***
 * @description: 从 ini 配置文件解析跟踪配置
 * 解析 [track] section 下的所有配置项并填充 TrackerConfig 结构体;
 * @param ini_path const string& : ini 配置文件的完整路径
 * @param config TrackerConfig& : 输出参数, 解析后填充的跟踪配置
 * @return void
 * @throws std::runtime_error ini 文件加载失败时抛出
 * @throws std::invalid_argument tracker_type 字符串无法识别时抛出
 */
void parser_ini_tracker_config(const std::string& ini_path, TrackerConfig& config)
{
    // 创建 IniParser 解析器实例
    IniParser ini_parser;

    // 加载并解析 ini 文件
    try
    {
        ini_parser.load(ini_path);
        LOG_DEFAULT_INFO("Load tracker ini file: %s", ini_path.c_str());
    }
    catch (const std::exception& e)
    {
        LOG_DEFAULT_ERROR("Failed to load tracker ini file: %s; %s", ini_path.c_str(), e.what());
        // 重新抛出异常, 交由上层处理
        throw std::runtime_error("Failed to load tracker ini file: " + ini_path);
    }

    // 以下逐一解析 [track] section 的所有配置项;
    // 1. 跟踪器类型: 从字符串解析, 自动转小写匹配
    try
    {
        // 从 ini 中读取字符串, 默认值为 "bytetrack"
        std::string type_str = ini_parser.get_string("track", "tracker_type", "bytetrack");
        // 调用小写敏感的字符串转枚举函数
        config.tracker_type = tracker_type_from_string(type_str);
        LOG_DEFAULT_INFO("tracker_type:%s", tracker_type_to_string(config.tracker_type).c_str());
    }
    catch (const std::exception& e)
    {
        LOG_DEFAULT_WARN("Parse tracker_type failed: %s; Use default: bytetrack", e.what());
        config.tracker_type = TrackerType::bytetrack;
    }

    // 2. 视频帧率
    config.frame_rate = static_cast<uint32>(ini_parser.get_int("track", "frame_rate", 30));
    LOG_DEFAULT_INFO("frame_rate:%d", config.frame_rate);

    // 3. 轨迹保留帧缓冲数
    config.track_buffer = static_cast<uint32>(ini_parser.get_int("track", "track_buffer", 30));
    LOG_DEFAULT_INFO("track_buffer:%d", config.track_buffer);

    // 4. 检测框置信度阈值
    config.track_thresh = static_cast<float32>(ini_parser.get_double("track", "track_thresh", 0.5));
    LOG_DEFAULT_INFO("track_thresh:%f", config.track_thresh);

    // 5. 高分检测框阈值
    config.high_thresh = static_cast<float32>(ini_parser.get_double("track", "high_thresh", 0.6));
    LOG_DEFAULT_INFO("high_thresh:%f", config.high_thresh);

    // 6. 关联匹配阈值
    config.match_thresh = static_cast<float32>(ini_parser.get_double("track", "match_thresh", 0.8));
    LOG_DEFAULT_INFO("match_thresh:%f", config.match_thresh);

    // 7. 轨迹最大丢失帧数
    config.max_age = static_cast<int32>(ini_parser.get_int("track", "max_age", 30));
    LOG_DEFAULT_INFO("max_age:%d", config.max_age);

    // 8. 新轨迹确认所需初始帧数
    config.n_init = static_cast<int32>(ini_parser.get_int("track", "n_init", 3));
    LOG_DEFAULT_INFO("n_init:%d", config.n_init);

    // 9. IoU 匹配阈值
    config.max_iou_distance = static_cast<float32>(ini_parser.get_double("track", "max_iou_distance", 0.7));
    LOG_DEFAULT_INFO("max_iou_distance:%f", config.max_iou_distance);

    // 10. 是否使用 ReID 特征(DeepSORT 专用)
    config.use_reid = ini_parser.get_bool("track", "use_reid", false);
    LOG_DEFAULT_INFO("use_reid:%d", config.use_reid);

    // 11. ReID 特征维度(DeepSORT 专用)
    config.reid_feature_dim = static_cast<uint32>(ini_parser.get_int("track", "reid_feature_dim", 0));
    LOG_DEFAULT_INFO("reid_feature_dim:%d", config.reid_feature_dim);

    // 12. 余弦距离阈值(DeepSORT 专用)
    config.max_cosine_distance = static_cast<float32>(ini_parser.get_double("track", "max_cosine_distance", 0.2));
    LOG_DEFAULT_INFO("max_cosine_distance:%f", config.max_cosine_distance);

    LOG_DEFAULT_INFO("Successfully loaded tracker config from: %s", ini_path.c_str());
}

}  // namespace tracker

#endif  // !__UTILS__H__