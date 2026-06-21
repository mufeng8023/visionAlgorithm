/***
 * @Author       : gxs
 * @Date         : 2026-06-21 15:30:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-21 15:30:00
 * @FilePath     : /visionAlgorithm/lib/tracker/TrackerConfig.hpp
 * @Description  : 跟踪器配置定义, 包括跟踪器类型枚举、配置结构体
 *                 以及枚举<->字符串互转函数;
 *                 设计风格与 lib/detector/NetConfig.hpp 保持完全一致;
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __TRACKERCONFIG__H__
#define __TRACKERCONFIG__H__

#include <algorithm>  // transform
#include <cctype>     // tolower
#include <map>        // map
#include <string>     // string
#include <vector>     // vector

#include "ini_parser.hpp"  // IniParser 配置解析类
#include "logging.hpp"     // 日志
#include "types.hpp"       // uint8, uint32, float32 等基础类型

namespace tracker
{

/***
 * @description: 跟踪器类型枚举
 * 放在最后自动代表枚举的总数; 如果修改了枚举值, 需要同步修改映射函数 get_tracker_type_map()
 */
enum class TrackerType : uint8
{
    sort = 0,   // SORT 算法: 简单在线实时跟踪, 仅基于 IoU 匹配;
    deepsort,   // DeepSORT 算法: SORT + ReID 特征余弦距离级联匹配;
    bytetrack,  // ByteTrack 算法: 高低分检测框两次关联, 鲁棒性强;
    ocsort,     // OC_SORT 算法: 基于观测置信度的 SORT 改进版本;
    count       // 技巧: 放在最后自动代表枚举的总数
};  // !如果修改了枚举值, 需要同步修改 get_tracker_type_map() 中的映射表

/***
 * @description: 跟踪器配置结构体
 * 用于存储所有跟踪器相关的配置参数, 可通过 ini 配置文件或直接赋值填充;
 * 默认值为 ByteTrack 算法常用的参数;
 */
typedef struct
{
    // 跟踪器类型: sort / deepsort / bytetrack / ocsort
    TrackerType tracker_type = TrackerType::count;

    // 通用跟踪参数
    // 视频帧率, 用于计算轨迹最大丢失帧数
    uint32 frame_rate = 30;
    // 轨迹保留的帧缓冲数, frame_rate / 30.0 * track_buffer 为最大丢失帧数
    uint32 track_buffer = 30;
    // 检测框置信度阈值, 高于此阈值的视为高分检测框用于第一次关联
    float32 track_thresh = 0.5;
    // 高分检测框阈值(ByteTrack 用), 高于此阈值的检测框才允许创建新轨迹
    float32 high_thresh = 0.6;
    // 关联匹配阈值, 用于匈牙利算法匹配时的 IoU 代价上限
    float32 match_thresh = 0.8;

    // 跟踪超参数
    // 轨迹最大丢失帧数, 超过此帧数未匹配的 Lost 轨迹将被标记为 Removed
    int32 max_age = 30;
    // 新轨迹确认前所需的最少匹配帧数, 达到此数后轨迹状态从 New 转为 Tracked
    int32 n_init = 3;
    // IoU 匹配阈值, 用于第二关联阶段以及未确认轨迹的匹配
    float32 max_iou_distance = 0.7;

    // DeepSORT 专用参数 (仅在 tracker_type == deepsort 时生效)
    // 是否使用 ReID 特征进行级联匹配, DeepSORT 必须为 true
    bool use_reid = false;
    // ReID 特征向量的维度, DeepSORT 通常为 512
    uint32 reid_feature_dim = 0;
    // 余弦距离阈值, 用于 ReID 特征匹配时的代价上限
    float32 max_cosine_distance = 0.2;

} TrackerConfig;

/***
 * @description: 将字符串转换为小写 (局部辅助函数)
 * 此处独立实现避免跨模块依赖;
 * @param str const string& : 输入字符串
 * @return string : 转换为全小写后的字符串
 */
inline std::string to_lower(const std::string& str)
{
    // 复制一份原字符串用于修改
    std::string result = str;
    // 对每个字符使用 tolower 转为小写
    std::transform(str.begin(), str.end(), result.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    // 返回转换后的字符串
    return result;
}

/***
 * @description: 获取 TrackerType 到字符串的映射表 (单例, 延迟初始化)
 * 使用 map 而非 array, 避免枚举值与数组索引的强耦合;
 * 添加新枚举值时, 只需在此函数中增加一行, 无需担心顺序错位;
 * @return {const std::map<TrackerType, std::string>&} 枚举到字符串的只读映射
 */
inline const std::map<TrackerType, std::string>& get_tracker_type_map()
{
    // static local 变量, 在 C++11 中是线程安全的 (magic static)
    static const std::map<TrackerType, std::string> tracker_type_map = {
        {TrackerType::sort, "sort"},            //
        {TrackerType::deepsort, "deepsort"},    //
        {TrackerType::bytetrack, "bytetrack"},  //
        {TrackerType::ocsort, "ocsort"},        //
    };
    return tracker_type_map;
}

/***
 * @description: 枚举转字符串 (O(log n) 性能, 基于 map 的 find)
 * @param type TrackerType : 跟踪器类型枚举
 * @return string : 枚举对应的字符串, 未知枚举返回 "unknown"
 */
inline std::string tracker_type_to_string(TrackerType type)
{
    // 获取 TrackerType 到字符串的映射表
    const std::map<TrackerType, std::string>& tracker_type_map = get_tracker_type_map();
    // 使用 map 的 find 函数查找对应的 TrackerType
    std::map<TrackerType, std::string>::const_iterator it = tracker_type_map.find(type);
    // 如果找到了对应的 TrackerType, 返回对应的字符串
    if (it != tracker_type_map.end())
    {
        return it->second;
    }
    return "unknown";
}

/***
 * @description: 字符串转枚举 (O(n) 遍历, 但代码很干净)
 * 输入字符串会被自动转换为小写后再遍历匹配, 确保大小写不敏感;
 * @param str string : 跟踪器类型字符串, 可大小写混用, 如 "ByteTrack" / "bytetrack" / "BYTETRACK"
 * @return TrackerType : 字符串对应的枚举值
 * @throws std::invalid_argument 当字符串不能匹配任何已知枚举时抛出
 */
inline TrackerType tracker_type_from_string(const std::string& str)
{
    // 将输入字符串转为小写, 确保大小写不敏感匹配
    std::string lower_str = to_lower(str);
    // 获取 TrackerType 到字符串的映射表
    const std::map<TrackerType, std::string>& tracker_type_map = get_tracker_type_map();
    // 映射表 迭代器
    std::map<TrackerType, std::string>::const_iterator it = tracker_type_map.begin();
    // 映射表 尾部迭代器
    std::map<TrackerType, std::string>::const_iterator end = tracker_type_map.end();
    // 遍历映射表, 查找对应的 TrackerType
    for (; it != end; ++it)
    {
        // 根据 小写字符串查找对应的 TrackerType
        if (it->second == lower_str)
        {
            return it->first;
        }
    }
    throw std::invalid_argument("Unknown TrackerType string: " + str);
}

}  // namespace tracker

#endif  // !__TRACKERCONFIG__H__