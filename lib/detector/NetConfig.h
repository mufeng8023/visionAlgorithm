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

typedef struct
{
    // 模型名称
    std::string model_name = "";
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

    // batch size
    int32 batch_size = 1;

    // 关键点个数
    int32 kpt_count = 0;
    // 关键点维度
    int32 kpt_dim = 0;

    // 输入图片大小
    int32 input_width = 0;
    int32 input_height = 0;
    int32 input_channels = 0;

    // 输出特征图的 strides
    std::vector<int32> strides = {8, 16, 32};
    // anchor 信息, 如果是空的表示是 anchor-free
    std::vector<std::vector<float32>> anchors = {};

} DetectionNetConfig;

#endif  // !__NETCONFIG__H__