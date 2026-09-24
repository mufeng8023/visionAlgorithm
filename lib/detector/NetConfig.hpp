/***
 * @Author       : gxs
 * @Date         : 2026-04-11 16:50:59
 * @LastEditors  : gxs
 * @LastEditTime : 2026-04-11 16:50:58
 * @FilePath     : /visionAlgorithm/lib/detector/NetConfig.h
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __NETCONFIG__H__
#define __NETCONFIG__H__

#include <map>
#include <string>
#include <vector>

#include "types.hpp"

namespace yolo
{

enum class ModelType : uint8
{
    yolov4 = 0,
    yolov5,   // anchor-base
    yolov7,   //
    yolov3u,  // anchor-free, u是指ultralytics
    yolov5u,  // anchor-free, u是指ultralytics
    yolov6u,  // anchor-free, u是指ultralytics
    yolov8,
    yolov9,
    yolov10,
    yolo11,
    yolo12,
    yolo26,
    count  // 技巧: 放在最后自动代表枚举的总数
};  // 如果修改了枚举值, 需要修改这个映射数组ModelStrings

enum class TaskType : uint8
{
    classification = 0,
    detection,
    segmentation,
    pose,
    obb,
    count  // 技巧: 放在最后自动代表枚举的总数
};  // 如果修改了枚举值, 需要修改这个映射数组TaskStrings

typedef struct
{
    // 模型名称
    std::string model_name = "";
    // 模型类型: yolov5 / yolov8 / yolo11 / yolo26
    ModelType model_type = ModelType::yolov5;
    // 模型任务: classification / detection / segmentation / pose / obb
    TaskType task = TaskType::detection;
    // 是否使用置信度
    bool has_conf = true;
    // 类别名字
    std::vector<std::string> names = {};
    // 类别个数
    uint32 nc = 0;
    // 反量化系数
    std::vector<float32> scale_outputs = {1.0, 1.0, 1.0};

    // 每个类别的置信度阈值
    std::vector<float32> conf_thrs = {0.1f};
    // 最小的置信度阈值 conf_thrs 的最小值
    float32 min_conf = 0.1f;
    // iou 阈值
    float32 iou_thrs = 0.45f;
    // max_det 每个图片最多检测多少个目标
    uint32 max_det = 300;
    // 是否进行类别区分, false: 不同类别之间不会进行nms
    bool agnostic = false;

    // batch size
    uint32 batch_size = 1;

    // 关键点个数
    uint32 kpt_count = 0;
    // 关键点维度
    uint32 kpt_dim = 0;
    // 关键点置信度阈值
    float32 kpt_conf_thr = 0.5f;
    // 其他模型参数 obb / seg

    // 输入图片大小
    uint32 input_width = 0;
    uint32 input_height = 0;
    uint32 input_channels = 0;

    // 输出特征图的 strides
    std::vector<uint32> strides = {8, 16, 32};
    // anchor 信息, 如果是空的表示是 anchor-free
    std::vector<std::vector<float32>> anchors = {};

    // 需要后期计算的变量
    // 每个位置anchor个数 anchors[0].size(), anchor-free默认为1;
    uint32 na = 0;
    // 每个位置输出的信息数 no
    // anchor-free: 4 + nc + ...
    // anchor-base: 4 + [1 if has_conf else 0] + nc + ...
    uint32 no = 0;
    // 输出层数 scale_outputs.size()
    uint32 nl = 0;
    // 每个输出特征图的宽高
    std::vector<uint32> net_out_h = {};
    std::vector<uint32> net_out_w = {};

} DetectionNetConfig;

/***
 * @description: 获取 ModelType 到字符串的映射表 (单例, 延迟初始化)
 * 使用 map 而非 array, 避免枚举值与数组索引的强耦合;
 * 添加新枚举值时, 只需在此函数中增加一行, 无需担心顺序错位;
 * @return {const std::map<ModelType, std::string>&} 枚举到字符串的只读映射
 */
inline const std::map<ModelType, std::string>& get_model_type_map()
{
    // static local 变量, 在 C++11 中是线程安全的 (magic static)
    static const std::map<ModelType, std::string> model_type_map = {
        {ModelType::yolov4, "yolov4"},    //
        {ModelType::yolov5, "yolov5"},    //
        {ModelType::yolov7, "yolov7"},    //
        {ModelType::yolov3u, "yolov3u"},  // anchor-free
        {ModelType::yolov5u, "yolov5u"},  // anchor-free
        {ModelType::yolov6u, "yolov6u"},  // anchor-free
        {ModelType::yolov8, "yolov8"},    //
        {ModelType::yolov9, "yolov9"},    //
        {ModelType::yolov10, "yolov10"},  //
        {ModelType::yolo11, "yolo11"},    //
        {ModelType::yolo12, "yolo12"},    //
        {ModelType::yolo26, "yolo26"},    //
    };
    return model_type_map;
}

/***
 * @description: 获取 TaskType 到字符串的映射表 (单例, 延迟初始化)
 * @return {const std::map<TaskType, std::string>&} 枚举到字符串的只读映射
 */
inline const std::map<TaskType, std::string>& get_task_type_map()
{
    static const std::map<TaskType, std::string> task_type_map = {
        {TaskType::classification, "classification"},  //
        {TaskType::detection, "detection"},            //
        {TaskType::segmentation, "segmentation"},      //
        {TaskType::pose, "pose"},                      //
        {TaskType::obb, "obb"},                        //
    };
    return task_type_map;
}

/***
 * @description: 枚举转字符串 (O(log n) 性能, 基于 map 的 find)
 * @param type ModelType :
 * @return
 */
inline std::string model_type_to_string(ModelType type)
{
    // 获取 ModelType 到字符串的映射表
    const std::map<ModelType, std::string>& model_type_map = get_model_type_map();
    // 使用 map 的 find 函数查找对应的 ModelType
    std::map<ModelType, std::string>::const_iterator it = model_type_map.find(type);
    // 如果找到了对应的 ModelType, 返回对应的字符串
    if (it != model_type_map.end())
    {
        // 对应 ModelType 的名称
        return it->second;
    }
    return "unknown";
}

/***
 * @description: 字符串转枚举 (O(n) 遍历, 但代码很干净)
 * @param str string :
 * @return
 */
inline ModelType model_type_from_string(const std::string& str)
{
    // 获取 ModelType 到字符串的映射表
    const std::map<ModelType, std::string>& model_type_map = get_model_type_map();
    // 映射表 迭代器
    std::map<ModelType, std::string>::const_iterator it = model_type_map.begin();
    // 映射表 尾部迭代器
    std::map<ModelType, std::string>::const_iterator end = model_type_map.end();
    // 遍历映射表, 查找对应的 ModelType
    for (; it != end; ++it)
    {
        // 根据 字符串查找对应的 ModelType
        if (it->second == str)
        {
            return it->first;
        }
    }
    throw std::invalid_argument("Unknown ModelType string: " + str);
}

/***
 * @description: 枚举转字符串 (O(log n) 性能, 基于 map 的 find)
 * @param type TaskType :
 * @return
 */
inline std::string task_type_to_string(TaskType type)
{
    // 获取 TaskType 到字符串的映射表
    const std::map<TaskType, std::string>& task_type_map = get_task_type_map();
    // 使用 map 的 find 函数查找对应的 TaskType
    std::map<TaskType, std::string>::const_iterator it = task_type_map.find(type);

    if (it != task_type_map.end())
    {
        return it->second;
    }
    return "unknown";
}

/***
 * @description: 字符串转枚举 (O(n) 遍历, 但代码很干净)
 * @param str string :
 * @return
 */
inline TaskType task_type_from_string(const std::string& str)
{
    // 获取 TaskType 到字符串的映射表
    const std::map<TaskType, std::string>& task_type_map = get_task_type_map();
    // 映射表 迭代器
    std::map<TaskType, std::string>::const_iterator it = task_type_map.begin();
    // 映射表 尾部迭代器
    std::map<TaskType, std::string>::const_iterator end = task_type_map.end();
    // 遍历映射表, 查找对应的 TaskType
    for (; it != end; ++it)
    {
        if (it->second == str)
        {
            return it->first;
        }
    }
    throw std::invalid_argument("Unknown TaskType string: " + str);
}

}  // namespace yolo

#endif  // !__NETCONFIG__H__
