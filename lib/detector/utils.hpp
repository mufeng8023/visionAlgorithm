/***
 * @Author       : gxs
 * @Date         : 2026-05-09 13:04:56
 * @LastEditors  : gxs
 * @LastEditTime : 2026-05-09 13:04:57
 * @FilePath     : /visionAlgorithm/lib/detector/utils.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __UTILS__H__
#define __UTILS__H__

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "NetConfig.h"
#include "ini_parser.hpp"
#include "logging.hpp"
namespace yolo
{

/***
 * @description: 将字符串转为小写
 * @param str string& : 需要转换的字符串
 * @return
 */
std::string to_lower(const std::string& str)
{
    std::string result = str;                               // 复制一份原字符串用于修改
    std::transform(str.begin(), str.end(), result.begin(),  //
                   [](unsigned char c) { return std::tolower(c); });

    return result;  // 返回修改后的新字符串
}

/***
 * @description:
 * @param ini_path string& :
 * @param config DetectionNetConfig& :
 * @return
 */
void parser_ini_det_net_config(const std::string& ini_path, DetectionNetConfig& config)
{
    IniParser ini_parser;

    try
    {
        ini_parser.load(ini_path);
        LOG_DEFAULT_INFO("Load ini file: %s", ini_path.c_str());
    }
    catch (const std::exception& e)
    {
        LOG_DEFAULT_ERROR("Failed to load ini file: %s; %s", ini_path.c_str(), e.what());
    }

    // 模型名字
    config.model_name = to_lower(ini_parser.get_string("detection", "model_name", "unknow"));
    // 模型类型, 根据名字选择后处理方式
    config.model_type = to_lower(ini_parser.get_string("detection", "model_type", "unknow"));
    // 任务类型, 根据名字选择后处理方式
    config.task = to_lower(ini_parser.get_string("detection", "task", "unknow"));
    // 输出中是否包括置信度, yolov5-face 和 yolov8 / yolo11 / yolo26 是没有的, 为false;
    config.has_conf = ini_parser.get_bool("detection", "has_conf", false);
    // 类别名字
    config.names = ini_parser.get_array1d<std::string>("detection", "names");
    // 是否是量化后的模型
    config.scale_outputs = ini_parser.get_array1d<float32>("detection", "scale_outputs");
    // 置信度阈值, 每个类别有一个
    config.conf_thrs = ini_parser.get_array1d<float32>("detection", "conf_thrs", {0.3});
    // nms iou 阈值
    config.iou_thrs = static_cast<float32>(ini_parser.get_double("detection", "iou_thrs", 0.45));
    // 图片最多检测多少个目标
    config.max_det = static_cast<uint32>(ini_parser.get_int("detection", "max_det", 300));
    // 是否进行类别区分, false: 不同类别之间不会进行nms
    config.agnostic = ini_parser.get_bool("detection", "agnostic", false);
    // Batch size
    config.batch_size = static_cast<uint32>(ini_parser.get_int("detection", "batch_size", 1));
    // 关键点数量
    config.kpt_count = static_cast<uint32>(ini_parser.get_int("detection", "kpt_count", 0));
    // 关键点维度
    config.kpt_dim = static_cast<uint32>(ini_parser.get_int("detection", "kpt_dim", 0));

    // 输入图片的通道数, 高度, 宽度
    std::vector<uint32> input_chw = ini_parser.get_array1d<uint32>("detection", "input_chw");
    config.input_channels = input_chw[0];
    config.input_height = input_chw[1];
    config.input_width = input_chw[2];

    // 每个输出层的步长
    config.strides = ini_parser.get_array1d<uint32>("detection", "strides");
    // 每个输出层的anchor, anchors 的个数为0, 说明是anchor free的模型
    config.anchors = ini_parser.get_array2d<float32>("detection", "anchors");

    // 需要计算的一些步骤和参数
    // 类别数量
    config.nc = config.names.size();
    // 输出层数量
    config.nl = config.strides.size();
    // 每个输出层的anchor数量
    config.na = config.anchors.empty() ? 1 : config.anchors[0].size() / 2;
    // 输出的信息数量
    // anchor-base yolov5: 4 + [1 if has_conf else 0] + nc + kpt_count * kpt_dim
    // anchor-free yolov8: 4 + nc + kpt_count * kpt_dim
    if (config.task == "detection")
    {
        // anchor-base 只有 yolov5-face 才需要 conf
        if (config.model_type == "yolov5" && config.has_conf)
        {
            config.no = 4 + 1 + config.nc;
        }
        else  // yolov8 / yolo11 / yolo26 不需要 conf
        {
            config.no = 4 + config.nc;
        }
    }
    else if (config.task == "pose")
    {
        // anchor-base 只有 yolov5-face 才需要 conf
        if (config.model_type == "yolov5" && config.has_conf)
        {
            config.no = 4 + 1 + config.nc + config.kpt_count * config.kpt_dim;
        }
        else  // yolov8 / yolo11 / yolo26 不需要 conf
        {
            config.no = 4 + config.nc + config.kpt_count * config.kpt_dim;
        }
    }
    else
    {
        LOG_DEFAULT_ERROR("task:%s not support", config.task.c_str());
        // 抛出异常, 退出程序
        throw std::runtime_error("task not support");
    }

    // 根据类别名字, 将 conf_thrs 进行扩充, 保证数量一致, 如果 conf_thrs 个数小于类别个数, 则使用最后一个值进行填充
    if (config.conf_thrs.size() < config.nc)
    {
        // 不修改原来的值, 新扩充的位置使用最后一个值进行填充
        config.conf_thrs.resize(config.nc, config.conf_thrs.back());
    }
    // 获取最小的conf阈值
    config.min_conf = *std::min_element(config.conf_thrs.begin(), config.conf_thrs.end());

    // 确保 scale_outputs / anchors / strides 数量一致
    if (config.scale_outputs.size() != config.nl || config.anchors.size() != config.nl)
    {
        LOG_DEFAULT_ERROR("scale_outputs:%d / anchors:%d / strides:%d size not equal to nl:%d",
                          config.scale_outputs.size(), config.anchors.size(), config.strides.size(), config.nl);
        throw std::runtime_error("scale_outputs / anchors / strides size not equal");
    }
    // 计算特征图的宽高 input_wh / strides[i]
    config.net_out_h.clear();
    config.net_out_w.clear();
    for (uint32 i = 0; i < config.nl; ++i)
    {
        config.net_out_h.push_back(config.input_height / config.strides[i]);
        config.net_out_w.push_back(config.input_width / config.strides[i]);
    }

    LOG_DEFAULT_INFO("load ini_path:%s", ini_path.c_str());
}

}  // namespace yolo

#endif  // !__UTILS__H__