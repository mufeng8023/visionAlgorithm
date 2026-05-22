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

#include <array>
#include <string>
#include <vector>

#include "types.hpp"

namespace yolo
{

enum class ModelType : uint8
{
    yolov3 = 0,  //
    yolov4,
    yolov5,  // anchor-base
    yolov6,
    yolov7,
    yolov5u,  // anchor-free, u是指ultralytics
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
    int32 nc = 0;
    // 反量化系数
    std::vector<float32> scale_outputs = {1.0, 1.0, 1.0};

    // 每个类别的置信度阈值
    std::vector<float32> conf_thrs = {0.1};
    // 最小的置信度阈值 conf_thrs 的最小值
    float32 min_conf = 0.1;
    // iou 阈值
    float32 iou_thrs = 0.45;
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
    float32 kpt_conf_thr = 0.5;
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

// 定义映射数组
constexpr std::array<std::string_view, static_cast<size_t>(ModelType::count)> ModelStrings = {
    "yolov3", "yolov4", "yolov5",  "yolov6", "yolov7", "yolov5u",
    "yolov8", "yolov9", "yolov10", "yolo11", "yolo12", "yolo26"};

// 定义映射数组
constexpr std::array<std::string_view, static_cast<size_t>(TaskType::count)> TaskStrings = {
    "classification", "detection", "segmentation", "pose", "obb"};

/***
 * @description: 枚举转字符串 (O(1) 性能)
 * @param type ModelType :
 * @return
 */
inline std::string model_type_to_string(ModelType type)
{
    size_t index = static_cast<size_t>(type);
    if (index < ModelStrings.size())
    {
        return std::string(ModelStrings[index]);
    }
    return "unknown";
}

/***
 * @description: 字符串转枚举 (依然需要遍历，但代码很干净)
 * @param str string_view :
 * @return
 */
inline ModelType model_type_from_string(std::string_view str)
{
    for (size_t i = 0; i < ModelStrings.size(); ++i)
    {
        if (ModelStrings[i] == str)
        {
            return static_cast<ModelType>(i);
        }
    }
    throw std::invalid_argument("Unknown ModelType string");
}

/***
 * @description: 枚举转字符串 (O(1) 性能)
 * @param type TaskType :
 * @return
 */
inline std::string task_type_to_string(TaskType type)
{
    size_t index = static_cast<size_t>(type);
    if (index < TaskStrings.size())
    {
        return std::string(TaskStrings[index]);
    }
    return "unknown";
}

/***
 * @description: 字符串转枚举
 * @param str string_view :
 * @return
 */
inline TaskType task_type_from_string(std::string_view str)
{
    for (size_t i = 0; i < TaskStrings.size(); ++i)
    {
        if (TaskStrings[i] == str)
        {
            return static_cast<TaskType>(i);
        }
    }
    throw std::invalid_argument("unknown");
}

}  // namespace yolo

#endif  // !__NETCONFIG__H__