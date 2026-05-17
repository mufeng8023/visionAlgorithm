/***
 * @Author       : gxs
 * @Date         : 2026-04-27 13:08:26
 * @LastEditors  : gxs
 * @LastEditTime : 2026-04-27 13:08:29
 * @FilePath     : /visionAlgorithm/lib/detector/V5DetPostProcess.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __V5DETPOSTPROCESS__H__
#define __V5DETPOSTPROCESS__H__

#include "BasePostProcess.hpp"

namespace yolo
{
class V5DetPostProcess : public BasePostProcess
{
   private:
    // 推理的batch
    uint32 batch_size = 0;
    // 类别数量
    uint32 nc = 0;

    // 反量化系数
    std::vector<float32> scale_outputs = {1.0, 1.0, 1.0};

    // 是否存在conf
    bool has_conf = true;
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

    // 每个位置anchor个数 anchors[0].size(), anchor-free默认为1;
    uint32 na = 0;
    // 每个位置输出的信息数 no
    // anchor-free: 4 + nc + ...
    // anchor-base: 4 + [1 if has_conf else 0] + nc + ...
    uint32 no = 0;
    // 输出层数 scale_outputs.size()
    uint32 nl = 0;

    // 输出特征图的 strides
    std::vector<uint32> strides = {8, 16, 32};
    // anchor 信息, 如果是空的表示是 anchor-free
    std::vector<std::vector<float32>> anchors = {};

    // 每个输出特征图的宽高
    std::vector<uint32> net_out_h = {};
    std::vector<uint32> net_out_w = {};
    // 输出的每个特征图的数据个数, batch_size * na * no * net_out_h[i] * net_out_w[i]
    std::vector<uint32> output_len = {};  // 每个特征图的输出数据大小

   public:
    V5DetPostProcess(const DetectionNetConfig& config)
    {
        // 初始化各种参数
        this->batch_size = config.batch_size;
        this->nc = config.nc;
        this->has_conf = config.has_conf;

        this->scale_outputs = config.scale_outputs;
        this->conf_thrs = config.conf_thrs;
        this->min_conf = config.min_conf;
        this->iou_thrs = config.iou_thrs;
        this->max_det = config.max_det;
        this->agnostic = config.agnostic;

        this->na = config.na;
        this->no = config.no;
        this->nl = config.nl;
        this->strides = config.strides;
        this->anchors = config.anchors;
        this->net_out_h = config.net_out_h;
        this->net_out_w = config.net_out_w;

        for (int i = 0; i < this->nl; i++)
        {
            // 计算每个特征图的输出数据大小, na默认为1是为了方便计算, 兼容性更高
            this->output_len.push_back(this->batch_size * this->na * this->no * this->net_out_h[i] *
                                       this->net_out_w[i]);
        }
    }

    // 禁止各种复制拷贝, 只引用传递
    /***
     * @description: 禁用拷贝构造函数, 防止对象被拷贝
     * @return
     */
    V5DetPostProcess(const V5DetPostProcess& other) = delete;

    /***
     * @description: 禁用赋值操作符, 防止对象被赋值
     * @return
     */
    V5DetPostProcess& operator=(const V5DetPostProcess& other) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    V5DetPostProcess(V5DetPostProcess&& other) = default;

    /***
     * @description: 禁用移动赋值操作符, 防止对象被移动赋值
     * @return
     */
    V5DetPostProcess& operator=(V5DetPostProcess&& other) = default;

    /***
     * @description: 析构函数
     * @return
     */
    ~V5DetPostProcess() = default;

    /***
     * @description:
     * @param output NetOutput& :
     * @param result ObjectBuffer& :
     * @return
     */
    /***
     * @description: 处理一层输出特征图
     * @param output NetOutput& : 输出的网络特征图
     * @param result ObjectBuffer& : 解析出来的结果
     * @param scale_outputs float32 : 反量化系数
     * @param net_out_h uint32 : 输出特征图的宽
     * @param net_out_w uint32 : 输出特征图的高
     * @param stride uint32 : 输出特征图的步长
     * @return
     */
    void process_one(const NetOutput& output,              //
                     std::vector<ObjectBuffer>& results,   //
                     const std::vector<float32>& anchors,  //
                     const float32 scale_output,           //
                     const uint32 net_out_h,               //
                     const uint32 net_out_w,               //
                     const uint32 stride)
    {
        // 遍历整个特征图, 解析出每个位置的结果
        // 开始遍历特征图 (B, na * no, h, w)
        for (uint32 batch_idx = 0; batch_idx < this->batch_size; ++batch_idx)
        {
            // 当前batch的首地址索引
            uint32 batch_addr = batch_idx * this->na * this->no * net_out_h * net_out_w;

            // 根据 batch 获取 result
            ObjectBuffer& result = results[batch_idx];

            // 遍历每个特征图的位置 (na * no, h, w)
            // 遍历anchor索引
            for (uint32 anchor_idx = 0; anchor_idx < this->na; ++anchor_idx)
            {
                // 一个通道的元素个数 height * width
                // 一组通道的元素总个数 no * height * width
                // 第a组的首地址 a * no * height * width
                // 这一组的首地址
                uint32 group_addr = batch_addr + anchor_idx * this->no * net_out_h * net_out_w;

                // 当前组的 anchor
                uint32 anchor_w = anchors[anchor_idx * 2];
                uint32 anchor_h = anchors[anchor_idx * 2 + 1];

                // 遍历每个位置 (特征图网格)
                for (uint32 grid_y = 0; grid_y < net_out_h; ++grid_y)
                {
                    for (uint32 grid_x = 0; grid_x < net_out_w; ++grid_x)
                    {
                        // 当前位置相对于当前组的偏移
                        uint32 grid_offset = grid_y * net_out_w + grid_x;

                        // 最大类别分数
                        float32 max_class_score = -1.0f;
                        // 最大分数对应的类别索引
                        uint32 max_class_idx = 0;
                        // 置信度 (box confidence)
                        float32 box_conf = 1.0f;

                        // 类别信息的起始偏移 (相对于x,y,w,h)
                        uint32 class_offset = 5;
                        if (!this->has_conf)
                        {
                            // 没有置信度通道时, 类别从第5个位置开始 (idx=4)
                            class_offset = 4;
                        }

                        // 获取当前已检测到的目标数量
                        uint32 output_idx = result.get_obj_count();
                        if (output_idx > this->max_det)
                        {
                            // 超过最大检测数, 跳过
                            continue;
                        }

                        // 当前网格特征的首地址
                        uint32 feature_addr = group_addr + grid_offset;
                        // 通道间的偏移量 (用于在C维度上跳跃)
                        uint32 channel_stride = net_out_h * net_out_w;

                        // 获取box置信度 (如果有conf通道的话)
                        if (this->has_conf)
                        {
                            box_conf = output[feature_addr + 4 * channel_stride] * scale_output;
                        }
                        // 置信度小于阈值, 跳过
                        if (box_conf < this->min_conf)
                        {
                            continue;
                        }

                        // 遍历所有类别, 找出最大分数和对应类别
                        float32 class_score = 0.0f;
                        for (uint32 class_idx = 0; class_idx < this->nc; ++class_idx)
                        {
                            class_score = output[feature_addr + (class_offset + class_idx) * channel_stride] * scale_output;
                            if (max_class_score < class_score)
                            {
                                max_class_score = class_score;
                                max_class_idx = class_idx;
                            }
                        }
                        // 最终置信度 = box_conf * max_class_score
                        box_conf *= max_class_score;

                        // 根据各类别的阈值进行过滤
                        if (box_conf < this->conf_thrs[max_class_idx])
                        {
                            continue;
                        }

                        // 扩展result缓冲区, 并标记为有效
                        result.expand_obj();
                        result.set_valid(output_idx, true);

                        // 解码边界框 (x, y, w, h)
                        float32 decoded_val = 0.0f;
                        // x坐标: (tx * 2 - 0.5 + cx) * stride
                        decoded_val = output[feature_addr + 0 * channel_stride] * scale_output * 2.0f;
                        result[output_idx][0] = (decoded_val + grid_x - 0.5f) * stride;
                        // y坐标
                        decoded_val = output[feature_addr + 1 * channel_stride] * scale_output * 2.0f;
                        result[output_idx][1] = (decoded_val + grid_y - 0.5f) * stride;
                        // w宽度: pw * (2 * tx)^2
                        decoded_val = output[feature_addr + 2 * channel_stride] * scale_output * 2.0f;
                        result[output_idx][2] = pow(decoded_val, 2.0f) * anchor_w;
                        // h高度
                        decoded_val = output[feature_addr + 3 * channel_stride] * scale_output * 2.0f;
                        result[output_idx][3] = pow(decoded_val, 2.0f) * anchor_h;
                        // 存储最终置信度和类别索引
                        result[output_idx][4] = box_conf;
                        result[output_idx][5] = static_cast<float32>(max_class_idx);

                    }  // for grid_x

                }  // for grid_y

            }  // for anchor_idx

        }  // for batch_idx
    }

    void run(const std::vector<NetOutput>& outputs, std::vector<ObjectBuffer>& results)
    {
        // 遍历每一层输出特征图, 解析并将多个特征图的结果保存到一个对象中
        for (int i = 0; i < this->nl; ++i)
        {
            this->process_one(outputs[i],              // 当前层的输出
                              results,                 // 当前层的对象结果
                              this->anchors[i],        // 当前层的anchors
                              this->scale_outputs[i],  // 当前层的scale_output
                              this->net_out_h[i],      // 当前层的net_out_h
                              this->net_out_w[i],      // 当前层的net_out_w
                              this->strides[i]         // 当前层的stride
            );
        }

        // 判断一下, 如果没有检测目标, 就直接返回, 不会进行 NMS
        non_max_suppression(results, this->iou_thrs, this->agnostic);
    }
};
}  // namespace yolo

#endif  // !__V5DETPOSTPROCESS__H__