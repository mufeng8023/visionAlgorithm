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
        for (uint32 batch_size_ = 0; batch_size_ < this->batch_size; ++batch_size_)
        {
            // 当前batch-size的首地址索引
            uint batch_addr = batch_size_ * this->na * this->no * net_out_h * net_out_w;

            // 根据 batch 获取 result
            ObjectBuffer& result = results[batch_size_];

            // 遍历每个特征图的位置 (na * no, h, w)
            // 遍历na_
            for (uint32 na_ = 0; na_ < this->na; ++na_)
            {
                // 一个通道的元素个数 height * width
                // 一组通道的元素总个数 no * height * width
                // 第a组的首地址 a * no * height * width
                // 这一组的首地址
                uint32 group_addr = batch_addr + na_ * this->no * net_out_h * net_out_w;

                // 当前组的 anchor
                uint32 anchor_w = anchors[na_ * 2];
                uint32 anchor_h = anchors[na_ * 2 + 1];

                // 遍历每个位置
                for (uint32 h = 0; h < net_out_h; ++h)
                {
                    for (uint32 w = 0; w < net_out_w; ++w)
                    {
                        // 当前位置相对于 当前组的位置偏移
                        uint32 offset_c = h * net_out_w + w;

                        // 最大分数
                        float32 max_score = -1.0;
                        // 最大分数的索引, 也就是类别id
                        float32 max_idx = 0.0;
                        // 如果有has_conf 获取conf值
                        float32 conf = 1.0;

                        // 获取所有类别的最大分数
                        uint32 cls_offset = 5;
                        if (!this->has_conf)
                        {
                            // 没有置信度
                            cls_offset = 4;
                        }

                        // 获取当前输出个数
                        uint32 output_idx = result.get_obj_count();
                        if (output_idx > this->max_det)
                        {
                            // 如果超过最大检测数, 就直接返回
                            continue;
                        }

                        // 遍历每个输出 x y w h conf nc
                        // 当前组的首地址
                        uint32 no_addr = group_addr + offset_c;
                        uint32 no_offset = net_out_h * net_out_w;

                        // 如果有置信度, 获取置信度, 否则就默认为1
                        if (this->has_conf)
                        {
                            conf = output[no_addr + 4 * no_offset] * scale_output;
                        }
                        // 如果置信度小于阈值, 就跳过
                        if (conf < this->min_conf)
                        {
                            continue;  // 直接跳过当前组
                        }

                        // 获取类别和类别分数
                        float32 value = 0.0;
                        for (uint32 nc_ = 0; nc_ < this->nc; ++nc_)
                        {
                            value = output[no_addr + (cls_offset + nc_) * no_offset] * scale_output;
                            if (max_score < value)
                            {
                                // 更新最大分数和索引
                                max_score = value;
                                max_idx = nc_;
                            }
                        }
                        // 根据分数判断是否要保留
                        // 当前是最后的预测分数
                        conf *= max_score;

                        // 根据每个类的阈值进行过滤
                        if (conf < this->conf_thrs[static_cast<uint32>(max_idx)])
                        {
                            continue;  // 跳过
                        }

                        // 获取其余信息
                        // 因为是直接将结果放到 result中, 提前扩容一个数据的位置, 方便通过索引直接访问
                        result.expand_obj();
                        result.set_valid(output_idx, true);

                        // x
                        value = output[no_addr + 0 * no_offset] * scale_output * 2.0;
                        result[output_idx][0] = (value + w - 0.5) * stride;
                        // y
                        value = output[no_addr + 1 * no_offset] * scale_output * 2.0;
                        result[output_idx][1] = (value + h - 0.5) * stride;
                        // w
                        value = output[no_addr + 2 * no_offset] * scale_output * 2.0;
                        result[output_idx][2] = pow(value, 2.0) * anchor_w;
                        // h
                        value = output[no_addr + 3 * no_offset] * scale_output * 2.0;
                        result[output_idx][3] = pow(value, 2.0) * anchor_h;
                        // 添加置信度和类别索引
                        result[output_idx][4] = conf;
                        result[output_idx][5] = max_idx;

                    }  // for w

                }  // for h

            }  // for na

        }  // for batch-size
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