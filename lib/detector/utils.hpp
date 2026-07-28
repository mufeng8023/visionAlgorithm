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
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "detector/NetConfig.hpp"
#include "ini_parser.hpp"
#include "logging.hpp"
#include "types.hpp"

namespace yolo
{

/***
 * @description: 将 vector<T> 转换为单条 string 的函数
 * @param vec std::vector<T>& : 输入的一维 vector
 * @param delimiter string& : 分隔符，默认是逗号 ","
 * @return std::string
 */
template <typename T>
std::string vector_to_string(const std::vector<T>& vec, const std::string& delimiter = ",")
{
    std::ostringstream oss;

    for (size_t i = 0; i < vec.size(); ++i)
    {
        oss << vec[i];
        if (i != vec.size() - 1 && !delimiter.empty())
        {
            oss << delimiter;
        }
    }

    return oss.str();
}

/***
 * @description: 将 vector<vector<T>> 转换为单条 string 的函数
 * 直接复用 vector_to_string 来处理每一行
 * @param table std::vector<std::vector<T>>& : 输入的二维 vector
 * @return std::string
 */
template <typename T>
std::string table_to_string(const std::vector<std::vector<T>>& table)
{
    if (table.empty())
        return "";

    std::ostringstream oss;

    for (size_t i = 0; i < table.size(); ++i)
    {
        // 直接调用 vector_to_string 处理当前行，元素间默认用逗号分隔
        oss << vector_to_string(table[i]);

        // 如果不是最后一行，在行与行之间加上分号 ";"
        if (i != table.size() - 1)
        {
            oss << "; ";
        }
    }

    return oss.str();
}

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
    LOG_DEFAULT_INFO("model_name:%s", config.model_name.c_str());

    // 模型类型, 根据名字选择后处理方式
    config.model_type = model_type_from_string(to_lower(ini_parser.get_string("detection", "model_type", "unknow")));
    LOG_DEFAULT_INFO("model_type:%s", model_type_to_string(config.model_type).c_str());

    // 任务类型, 根据名字选择后处理方式
    config.task = task_type_from_string(to_lower(ini_parser.get_string("detection", "task", "unknow")));
    LOG_DEFAULT_INFO("task:%s", task_type_to_string(config.task).c_str());

    // 输出中是否包括置信度, yolov5-face 和 yolov8 / yolo11 / yolo26 是没有的, 为false;
    config.has_conf = ini_parser.get_bool("detection", "has_conf", false);
    LOG_DEFAULT_INFO("has_conf:%d", config.has_conf);

    // 类别名字
    config.names = ini_parser.get_array1d<std::string>("detection", "names");
    LOG_DEFAULT_INFO("names:%s", vector_to_string(config.names).c_str());

    // 是否是量化后的模型
    config.scale_outputs = ini_parser.get_array1d<float32>("detection", "scale_outputs");
    LOG_DEFAULT_INFO("scale_outputs:%s", vector_to_string(config.scale_outputs).c_str());

    // 置信度阈值, 每个类别有一个
    config.conf_thrs = ini_parser.get_array1d<float32>("detection", "conf_thrs", {0.3});
    LOG_DEFAULT_INFO("conf_thrs:%s", vector_to_string(config.conf_thrs).c_str());

    // nms iou 阈值
    config.iou_thrs = static_cast<float32>(ini_parser.get_double("detection", "iou_thrs", 0.45));
    LOG_DEFAULT_INFO("iou_thrs:%f", config.iou_thrs);

    // 图片最多检测多少个目标
    config.max_det = static_cast<uint32>(ini_parser.get_int("detection", "max_det", 300));
    LOG_DEFAULT_INFO("max_det:%d", config.max_det);

    // 是否进行类别区分, false: 不同类别之间不会进行nms
    config.agnostic = ini_parser.get_bool("detection", "agnostic", false);
    LOG_DEFAULT_INFO("agnostic:%d", config.agnostic);

    // Batch size
    config.batch_size = static_cast<uint32>(ini_parser.get_int("detection", "batch_size", 1));
    LOG_DEFAULT_INFO("batch_size:%d", config.batch_size);

    // 关键点数量
    config.kpt_count = static_cast<uint32>(ini_parser.get_int("detection", "kpt_count", 0));
    LOG_DEFAULT_INFO("kpt_count:%d", config.kpt_count);

    // 关键点维度
    config.kpt_dim = static_cast<uint32>(ini_parser.get_int("detection", "kpt_dim", 0));
    LOG_DEFAULT_INFO("kpt_dim:%d", config.kpt_dim);

    // 输入图片的通道数, 高度, 宽度
    std::vector<uint32> input_chw = ini_parser.get_array1d<uint32>("detection", "input_chw");
    LOG_DEFAULT_INFO("input_chw:%s", vector_to_string(input_chw).c_str());

    config.input_channels = input_chw[0];
    config.input_height = input_chw[1];
    config.input_width = input_chw[2];

    // 每个输出层的步长
    config.strides = ini_parser.get_array1d<uint32>("detection", "strides");
    LOG_DEFAULT_INFO("strides:%s", vector_to_string(config.strides).c_str());

    // 每个输出层的anchor, anchors 的个数为0, 说明是anchor free的模型
    config.anchors = ini_parser.get_array2d<float32>("detection", "anchors");
    LOG_DEFAULT_INFO("anchors:%s", table_to_string(config.anchors).c_str());

    if ((config.model_type == ModelType::yolov4 || config.model_type == ModelType::yolov5)  //
        && config.anchors.empty())
    {
        LOG_DEFAULT_ERROR(
            "The yolov4 or yolov5 model must have anchors. "
            "yolov5u is an anchor-free model. "
            "Please confirm whether you intended to specify yolov5u.");
        throw std::runtime_error(
            "The yolov4 or yolov5 model must have anchors. yolov5u is an anchor-free model. "
            "Please confirm whether you intended to specify yolov5u.");
    }

    // 需要计算的一些步骤和参数
    // 类别数量
    config.nc = config.names.size();
    LOG_DEFAULT_INFO("nc:%d", config.nc);
    // 输出层数量
    config.nl = config.strides.size();
    LOG_DEFAULT_INFO("nl:%d", config.nl);
    // 每个输出层的anchor数量
    config.na = config.anchors.empty() ? 1 : config.anchors[0].size() / 2;
    LOG_DEFAULT_INFO("na:%d", config.na);
    // 输出的信息数量
    // anchor-base yolov5: 4 + [1 if has_conf else 0] + nc + kpt_count * kpt_dim
    // anchor-free yolov8: 4 + nc + kpt_count * kpt_dim
    if (config.task == TaskType::detection)
    {
        // anchor-base 只有 yolov4 / yolov5-face 才需要 conf
        if ((config.model_type == ModelType::yolov4 || config.model_type == ModelType::yolov5)  //
            && config.has_conf)
        {
            config.no = 4 + 1 + config.nc;
        }
        else  // yolov8 / yolo11 / yolo26 不需要 conf
        {
            config.no = 4 + config.nc;
        }
    }
    else if (config.task == TaskType::pose)
    {
        // anchor-base 只有 yolov4 / yolov5-face 才需要 conf
        if ((config.model_type == ModelType::yolov4 || config.model_type == ModelType::yolov5)  //
            && config.has_conf)
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
        LOG_DEFAULT_ERROR("task:%s not support", task_type_to_string(config.task).c_str());
        // 抛出异常, 退出程序
        throw std::runtime_error("task not support");
    }
    LOG_DEFAULT_INFO("no:%d", config.no);

    // 根据类别名字, 将 conf_thrs 进行扩充, 保证数量一致, 如果 conf_thrs 个数小于类别个数, 则使用最后一个值进行填充
    if (config.conf_thrs.size() < static_cast<size_t>(config.nc))
    {
        // 不修改原来的值, 新扩充的位置使用最后一个值进行填充
        config.conf_thrs.resize(config.nc, config.conf_thrs.back());
    }
    // 获取最小的conf阈值
    config.min_conf = *std::min_element(config.conf_thrs.begin(), config.conf_thrs.end());
    LOG_DEFAULT_INFO("min_conf:%f", config.min_conf);

    // 确保 scale_outputs / anchors / strides 数量一致
    if (config.model_type == ModelType::yolov4 || config.model_type == ModelType::yolov5)
    {
        if (config.scale_outputs.size() != config.nl || config.anchors.size() != config.nl)
        {
            LOG_DEFAULT_ERROR("scale_outputs:%d / anchors:%d / strides:%d size not equal to nl:%d",
                              config.scale_outputs.size(), config.anchors.size(), config.strides.size(), config.nl);
            throw std::runtime_error("scale_outputs / anchors / strides size not equal");
        }
    }
    else
    {
        if (config.scale_outputs.size() != config.nl)
        {
            LOG_DEFAULT_ERROR("scale_outputs:%d / strides:%d size not equal to nl:%d",
                              config.scale_outputs.size(),  //
                              config.strides.size(), config.nl);
            throw std::runtime_error("scale_outputs / strides size not equal");
        }
    }
    // 计算特征图的宽高 input_wh / strides[i]
    config.net_out_h.clear();
    config.net_out_w.clear();
    for (uint32 i = 0; i < config.nl; ++i)
    {
        uint32 stride = config.strides[i];
        if (stride == 0)
        {
            LOG_DEFAULT_ERROR("strides[%d] is 0, cannot divide by zero", i);
            throw std::runtime_error("strides[" + std::to_string(i) + "] is 0");
        }

        config.net_out_h.push_back(config.input_height / stride);
        config.net_out_w.push_back(config.input_width / stride);
        LOG_DEFAULT_INFO("net_out_h[%d]:%d, net_out_w[%d]:%d",  //
                         i, config.net_out_h[i], i, config.net_out_w[i]);
    }

    LOG_DEFAULT_INFO("load ini_path:%s", ini_path.c_str());
}

/***
 * @description:
 * @param image Mat& : 输入图像
 * @param target_height int32 : 目标高度
 * @param target_width int32 : 目标宽度
 * @param padded_img Mat& : 处理后的图像
 * @return ratio: resize比例; dw: 左右对称的填充; dh: 上下对称的填充
 * 使用方法, 输出得到的检测结果, 采用如下方式进行还原
 * x = (x - dw) / ratio
 * y = (y - dh) / ratio
 * w = w * ratio
 * h = h * ratio
 */
std::tuple<float32, int32, int32> pre_process_resize_img(const cv::Mat& image,       //
                                                         const int32 target_height,  //
                                                         const int32 target_width,   //
                                                         cv::Mat& padded_img)
{
    // 获取原始图像的宽度和高度
    int32 height = image.rows;
    int32 width = image.cols;

    // 如果输入图片大小刚好和一致，就
    if (height == target_height and width == target_width)
    {
        padded_img = image.clone();

        return std::make_tuple(1.0, 0, 0);
    }

    // 计算宽度和高度的缩放比例
    float32 ratio = std::min(static_cast<float32>(target_width) / width,  //
                             static_cast<float32>(target_height) / height);

    // 计算缩放后的宽度和高度
    int32 new_width = static_cast<int32>(width * ratio);
    if (new_width > target_width)
    {
        new_width = target_width;
    }
    int32 new_height = static_cast<int32>(height * ratio);
    if (new_height > target_height)
    {
        new_height = target_height;
    }

    // 等比例缩放图像
    cv::Mat resized_img;
    cv::resize(image, resized_img, cv::Size(new_width, new_height));

    // 计算需要填充的宽度和高度
    int32 dw = (target_width - new_width) / 2;
    if (dw < 0)
    {
        dw = 0;
    }
    int32 dh = (target_height - new_height) / 2;
    if (dh < 0)
    {
        dh = 0;
    }

    // 先为 padded_img 分配内存 (创建指定大小的黑色图像)
    padded_img = cv::Mat::zeros(target_height, target_width, image.type());

    // 将缩放后的图像放置在中心位置
    resized_img.copyTo(padded_img(cv::Rect(dw, dh, new_width, new_height)));

    return std::make_tuple(ratio, dw, dh);
}

}  // namespace yolo

#endif  // !__UTILS__H__
