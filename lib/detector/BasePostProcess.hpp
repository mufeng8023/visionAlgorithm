/***
 * @Author       : gxs
 * @Date         : 2026-04-11 21:24:31
 * @LastEditors  : gxs
 * @LastEditTime : 2026-04-11 21:24:31
 * @FilePath     : /visionAlgorithm/lib/detector/BasePostProcess.hpp
 * @Description  :
 * @ 后处理基类
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __BASEPOSTPROCESS__H__
#define __BASEPOSTPROCESS__H__

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "common.hpp"
#include "detector/NetConfig.hpp"
#include "detector/NetOutput.hpp"
#include "detector/ObjectBuffer.hpp"
#include "logging.hpp"
#include "timer.hpp"

#define DET_POSTPROCESS_TIME_NAME "det_postprocess"  // 后处理时间的计时器名称
#define DET_NMS_TIME_NAME "det_nms"                  // nms时间的计时器名称

namespace yolo
{

/***
 * @description: 对一个图片的结果进行nms操作
 * @param output ObjectBuffer& : 一张图片的所有结果
 * @param iou_thr float32 : iou阈值
 * @param agnostic bool : 是否进行类别不敏感的nms, 为 false 时不同类别之间的box不会nms
 * @return
 */
static inline void nms_ops(ObjectBuffer& output, float32 iou_thr = 0.45, bool agnostic = false)
{
    // 获取所有目标的个数
    uint32 count = output.get_obj_count();
    LOG_DEFAULT_DEBUG("before nms_ops: output.size = %d", count);

    // 如果只有一个结果, 则直接返回
    if (count <= 1)
    {
        return;
    }

    // 开始进行nms操作
    LOG_DEFAULT_DEBUG("nms_ops: start nms, get sorted indices");
    //  获取按分数降序排列的索引列表, 添加进来的有效+无效[可能存在, 置信度筛选低于阈值被置为无效]的目标个数
    std::vector<uint32> indices = output.get_sorted_indices();

    LOG_DEFAULT_DEBUG("nms_ops: start for loop to remove overlapped obj");
    //  遍历索引列表
    for (uint32 i = 0; i < indices.size(); ++i)
    {
        uint32 obj_idx_i = indices[i];
        // 如果当前目标是无效的就跳过
        if (!output.is_valid(obj_idx_i))
        {
            LOG_DEFAULT_DEBUG("nms_ops: obj_idx_i = %d is invalid;", obj_idx_i);
            continue;
        }

        // 获取当前目标的边界框信息
        const float32* data_i_fp32p = output.at(obj_idx_i);
        // 解析框 i 的坐标 (存储的是 x_center, y_center, w, h)
        float32 ix1 = data_i_fp32p[ObjectOffset::x_center] - data_i_fp32p[ObjectOffset::width] * 0.5f;
        float32 iy1 = data_i_fp32p[ObjectOffset::y_center] - data_i_fp32p[ObjectOffset::height] * 0.5f;
        float32 ix2 = data_i_fp32p[ObjectOffset::x_center] + data_i_fp32p[ObjectOffset::width] * 0.5f;
        float32 iy2 = data_i_fp32p[ObjectOffset::y_center] + data_i_fp32p[ObjectOffset::height] * 0.5f;
        float32 area_i = data_i_fp32p[ObjectOffset::width] * data_i_fp32p[ObjectOffset::height];
        uint32 cls_id_i = static_cast<uint32>(data_i_fp32p[ObjectOffset::cls_id]);

        for (uint32 j = i + 1; j < indices.size(); ++j)
        {
            uint32 obj_idx_j = indices[j];

            // 如果当前目标是无效的就跳过
            if (!output.is_valid(obj_idx_j))
            {
                LOG_DEFAULT_DEBUG("nms_ops: obj_idx_j = %d is invalid;", obj_idx_j);
                continue;
            }

            // 获取当前目标的边界框信息
            const float32* data_j_fp32p = output.at(obj_idx_j);

            // 如果 agnostic 为 false 不同类别之间不会进行nms
            uint32 cls_id_j = static_cast<uint32>(data_j_fp32p[ObjectOffset::cls_id]);
            if (!agnostic && (cls_id_i != cls_id_j))
            {
                continue;
            }

            // 解析框 j 的坐标 (存储的是 x_center, y_center, w, h)
            float32 jx1 = data_j_fp32p[ObjectOffset::x_center] - data_j_fp32p[ObjectOffset::width] * 0.5f;
            float32 jy1 = data_j_fp32p[ObjectOffset::y_center] - data_j_fp32p[ObjectOffset::height] * 0.5f;
            float32 jx2 = data_j_fp32p[ObjectOffset::x_center] + data_j_fp32p[ObjectOffset::width] * 0.5f;
            float32 jy2 = data_j_fp32p[ObjectOffset::y_center] + data_j_fp32p[ObjectOffset::height] * 0.5f;
            float32 area_j = data_j_fp32p[ObjectOffset::width] * data_j_fp32p[ObjectOffset::height];

            // 计算两个框的iou
            // 计算交集 (Intersection)
            float32 inter_x1 = std::max(ix1, jx1);
            float32 inter_y1 = std::max(iy1, jy1);
            float32 inter_x2 = std::min(ix2, jx2);
            float32 inter_y2 = std::min(iy2, jy2);
            // 计算交集面积
            float32 inter_w = std::max(0.0f, inter_x2 - inter_x1);
            float32 inter_h = std::max(0.0f, inter_y2 - inter_y1);
            float32 inter_area = inter_w * inter_h;

            // 计算 IoU
            float32 iou = 0.0;
            if (inter_area > 1e-4f)
            {
                iou = inter_area / (area_i + area_j - inter_area + 1e-6f);
            }

            // 如果 IoU 大于阈值，则将框 j 标记为无效
            if (iou >= iou_thr)
            {
                output.set_valid(obj_idx_j, false);
            }
        }  // for j
    }  // for i

    // 压缩物理缓存区, 将目标变得连续
    try
    {
        output.compact();
    }
    catch (const std::exception& e)
    {
        LOG_DEFAULT_ERROR("nms_ops: %s", e.what());
        return;
    }

    LOG_DEFAULT_DEBUG("after nms_ops: output.size = %d", output.get_obj_count());
}

/***
 * @description:
 * @param output ObjectBuffer& : 后处理后的输出结果,
 * @param max_det uint32 :
 * @return
 */
static inline void end2end_post(ObjectBuffer& output, uint32 max_det = 300)
{
    // 获取所有目标的个数
    uint32 count = output.get_obj_count();
    LOG_DEFAULT_DEBUG("end2end_post: output.size = %d", count);

    if (count <= max_det)
    {
        LOG_DEFAULT_DEBUG("end2end_post: output.size = %d, no need to end2end_post", count);
        return;
    }

    // 保证目标都是有效的
    if (output.get_valid_count() != output.get_obj_count())
    {
        LOG_DEFAULT_DEBUG("end2end_post: output.size = %d, valid_count = %d", output.get_obj_count(),
                          output.get_valid_count());

        // 压缩物理缓存区, 将目标变得连续
        try
        {
            output.compact();
        }
        catch (const std::exception& e)
        {
            LOG_DEFAULT_ERROR("end2end_post: %s", e.what());
            throw std::runtime_error("end2end_post: ObjectBuffer compact error");
        }
    }

    //  获取按分数降序排列的索引列表, 添加进来的有效+无效[可能存在, 置信度筛选低于阈值被置为无效]的目标个数
    std::vector<uint32> indices = output.get_sorted_indices();

    // 遍历索引列表, max_det 个目标后, 剩余的目标设置为 无效
    for (uint32 idx = max_det; idx < indices.size(); ++idx)
    {
        // 目标设置为 无效
        output.set_valid(indices[idx], false);
    }

    // 压缩物理缓存区, 将目标变得连续
    try
    {
        output.compact();
    }
    catch (const std::exception& e)
    {
        LOG_DEFAULT_ERROR("end2end_post output.compact error: %s", e.what());
    }
}

/***
 * @description: 非极大值抑制
 * @param outputs std::vector<ObjectBuffer>& : 后处理后的输出结果, 每个图片算一个 ObjectBuffer
 * !本次设计是直接在每个 ObjectBuffer 上直接进行 nms 操作, 原地修改, 所以不能使用const
 * @param iou_thr float32 : IoU 阈值
 * @param agnostic bool : 是否进行类别区分, false: 不同类别之间不会进行nms
 * @return
 */
inline void non_max_suppression(std::vector<ObjectBuffer>& outputs,  //
                                float32 iou_thr = 0.45,              //
                                bool agnostic = false,               //
                                uint32 max_det = 300,                //
                                bool end2end = false                 //
)
{
#ifdef DEBUG_MODE
    // 断言检查
    assert(0. <= iou_thr && iou_thr <= 1. && "Invalid IoU, valid values are between 0.0 and 1.0");
#else
    if (iou_thr < 0. || iou_thr > 1.)
    {
        iou_thr = 0.45;
        LOG_DEFAULT_WARN("iou_thr values are between 0.0 and 1.0, but got %.2f; Changed the value to 0.45", iou_thr);
    }
#endif

    // outputs 的个数, 即 Batch Size
    uint32 batch_size = outputs.size();

    // 开始对每个 Batch 进行处理
    for (uint32 batch_idx = 0; batch_idx < batch_size; ++batch_idx)
    {
        // 获取当前 Batch 的输出
        ObjectBuffer& output = outputs[batch_idx];
        if (output.get_obj_count() == 0)
        {
            LOG_DEFAULT_DEBUG("non_max_suppression: batch: %d, output.size = 0", batch_idx);
        }

        // 对当前 Batch 的输出进行非极大值抑制
        try
        {
            if (end2end)  // yolo26 和 yolov10 都是使用 end2end 模式
            {
                end2end_post(output, max_det);
            }
            else
            {
                nms_ops(output, iou_thr, agnostic);
            }
        }
        catch (const std::exception& e)
        {
            LOG_DEFAULT_ERROR("non_max_suppression: %s", e.what());
        }
    }
}

class BasePostProcess
{
   public:
    /***
     * @description:
     * @return
     */
    BasePostProcess() = default;

    /***
     * @description: 虚析构函数
     * @return
     */
    virtual ~BasePostProcess() = default;

    /***
     * @description: 后处理函数, 在当前函数中要实现对输出的特征图映射为具体的检测框和其他信息,
     * 根据不同类别的置信度进行筛选,
     * !不会进行nms
     * @param outputs std::vector<NetOutput> : BaseNet 的输出, 三组特征, 每个特征图shape为(b, na*no, h, w)
     * @param results std::vector<ObjectBuffer> : 最后的输出结果, 每个图片算一个vector
     * @return
     */
    virtual void run(const std::vector<NetOutput>& outputs, std::vector<ObjectBuffer>& results) = 0;

    /**
     * @description: 输出类别信息, 比如类别名称啥的
     * @return {*}
     */
    virtual std::string to_string() const
    {
        // 使用模板函数 get_class_name 获取类名
        return get_class_name(*this);
    }
};

/**
 * @description: 重载
 * @return {*}
 */
inline std::ostream& operator<<(std::ostream& os, const BasePostProcess& obj)
{
    os << obj.to_string();
    return os;
}

}  // namespace yolo
#endif  // !__BASEPOSTPROCESS__H__