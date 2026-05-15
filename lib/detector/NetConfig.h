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
    // 模型类型: yolov5 / yolov8 / yolo11 / yolo26
    std::string model_type = "yolov5";
    // 模型任务: detection / classification / segmentation / pose / obb
    std::string task = "detection";
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

#endif  // !__NETCONFIG__H__