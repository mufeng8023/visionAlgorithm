/***
 * @Author       : gxs
 * @Date         : 2026-05-29 10:55:15
 * @LastEditors  : gxs
 * @LastEditTime : 2026-05-29 10:55:31
 * @FilePath     : /visionAlgorithm/lib/detector/DetPostProcessV5.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __DETPOSTPROCESSV5__H__
#define __DETPOSTPROCESSV5__H__

#include "detector/BasePostProcess.hpp"

namespace yolo
{
class DetPostProcessV5 : public BasePostProcess
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
    DetPostProcessV5(const DetectionNetConfig& config)
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

        for (uint32 i = 0; i < this->nl; i++)
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
    DetPostProcessV5(const DetPostProcessV5& other) = delete;

    /***
     * @description: 禁用赋值操作符, 防止对象被赋值
     * @return
     */
    DetPostProcessV5& operator=(const DetPostProcessV5& other) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    DetPostProcessV5(DetPostProcessV5&& other) = default;

    /***
     * @description: 禁用移动赋值操作符, 防止对象被移动赋值
     * @return
     */
    DetPostProcessV5& operator=(DetPostProcessV5&& other) = default;

    /***
     * @description: 析构函数
     * @return
     */
    ~DetPostProcessV5() = default;

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
        // NOTE: 提前计算好单张特征图一个通道的面积, 避免在内层循环中重复计算乘法
        const uint32 grid_size = net_out_h * net_out_w;
        // 计算类别的偏移量
        const uint32 class_offset = this->has_conf ? 5 : 4;

        // 遍历整个特征图, 解析出每个位置的结果
        // 开始遍历特征图 (B, na * no, h, w)
        for (uint32 batch_idx = 0; batch_idx < this->batch_size; ++batch_idx)
        {
            // 遍历每个特征图的位置 (na * no, h, w)
            // 遍历anchor索引
            for (uint32 anchor_idx = 0; anchor_idx < this->na; ++anchor_idx)
            {
                // 根据 batch 获取 result
                ObjectBuffer& result = results[batch_idx];

                // 这里计算的是每个 batch 的 每个 anchor 实际索引, 用来计算偏移量
                // 将原本四维寻址展平 (batch, na * no, h, w) -> ( batch * na * no * h * w, )
                // 来看 base_ch_idx 是 指 当前 batch 第几个 组anchor 的索引
                // 实际的计算应该是: batch_idx * this->na * this->no + anchor_idx * this->no
                uint32 base_ch_idx = (batch_idx * this->na + anchor_idx) * this->no;

                // 获取当 batch 的第 anchor_idx 的首地址, 后续通过 base_output_ptr[idx] 访问数据
                const float32* base_output_ptr = output.data() + base_ch_idx * grid_size;

                // 当前组的 anchor
                const uint32& anchor_w = anchors[anchor_idx * 2];
                const uint32& anchor_h = anchors[anchor_idx * 2 + 1];

                // 核心优化: 在进入循环之前, 先将 x / y / w / h / conf / nc 的各自通道的 [绝对首地址] 指针
                // 彻底消除了原代码最内层中类似 [feature_addr + k * channel_stride] 的复杂乘法寻址
                const float32* x_ptr = base_output_ptr + 0 * grid_size;
                const float32* y_ptr = base_output_ptr + 1 * grid_size;
                const float32* w_ptr = base_output_ptr + 2 * grid_size;
                const float32* h_ptr = base_output_ptr + 3 * grid_size;
                // 要注意, conf_ptr 是可选的, 如果没有置信度通道, 则为 nullptr
                const float32* conf_ptr = this->has_conf ? base_output_ptr + 4 * grid_size : nullptr;
                // 类别首地址, 类别通道的指针定位同样利用预计算的行首, 保持 offset 的连续性
                // has_conf 为 true 时, class_offset = 4 + 1 = 5
                // has_conf 为 false 时, class_offset = 4
                const float32* class_ptr = base_output_ptr + class_offset * grid_size;

                // 遍历每个位置 (特征图网格)
                // 核心优化,将 grid_y 和 grid_x 调整至最内层
                // 这样在进行 `[offset]` 访问时, 内存是完全连续线性扫描的, 极大地提升了 CPU Cache 命中率
                for (uint32 grid_y = 0; grid_y < net_out_h; ++grid_y)
                {
                    // 提前计算 当前行首地址 相对于 grid 的首地址的偏移量
                    uint32 row_offset = grid_y * net_out_w;

                    for (uint32 grid_x = 0; grid_x < net_out_w; ++grid_x)
                    {
                        // 计算当前 像素点 在 grid 的实际 偏移量
                        uint32 grid_offset = row_offset + grid_x;

                        // 获取当前已检测到的目标数量
                        uint32 output_idx = result.get_obj_count();
                        if (output_idx > this->max_det)
                        {
                            // 超过最大检测数, 直接退出
                            break;
                        }

                        // 获取box置信度 (如果有conf通道的话)
                        float32 box_conf = 1.0f;
                        if (this->has_conf)
                        {
                            box_conf = conf_ptr[grid_offset] * scale_output;
                        }
                        // 置信度小于阈值, 跳过
                        if (box_conf < this->min_conf)
                        {
                            continue;
                        }

                        // 最大类别分数
                        float32 max_class_score = -1.0f;
                        // 最大分数对应的类别索引
                        uint32 max_class_idx = 0;

                        // 定义一个临时指针指向当前类别的通道
                        const float32* cur_class_ptr = class_ptr;
                        // 遍历所有类别, 找出最大分数和对应类别
                        for (uint32 class_idx = 0; class_idx < this->nc; ++class_idx)
                        {
                            // 类别通道的指针定位同样利用预计算的行首, 保持 offset 的连续性
                            float32 class_score = cur_class_ptr[grid_offset] * scale_output;

                            if (max_class_score < class_score)
                            {
                                max_class_score = class_score;
                                max_class_idx = class_idx;
                            }

                            // 更新类别指针 (指向下一个类别通道)
                            cur_class_ptr += grid_size;
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
                        // x坐标: (tx * 2 - 0.5 + cx) * stride
                        float32 dx = x_ptr[grid_offset] * scale_output * 2.0f;
                        result[output_idx][ObjectOffset::x_center] = (dx + grid_x - 0.5f) * stride;

                        // y坐标
                        float32 dy = y_ptr[grid_offset] * scale_output * 2.0f;
                        result[output_idx][ObjectOffset::y_center] = (dy + grid_y - 0.5f) * stride;

                        // w宽度: pw * (2 * tx)^2; 使用乘法代替 pow
                        float32 dw = w_ptr[grid_offset] * scale_output * 2.0f;
                        result[output_idx][ObjectOffset::width] = dw * dw * anchor_w;

                        // h高度: ph * (2 * ty)^2; 使用乘法代替 pow
                        float32 dh = h_ptr[grid_offset] * scale_output * 2.0f;
                        result[output_idx][ObjectOffset::height] = dh * dh * anchor_h;

                        // 存储最终置信度和类别索引
                        result[output_idx][ObjectOffset::score] = box_conf;
                        result[output_idx][ObjectOffset::cls_id] = static_cast<float32>(max_class_idx);

                    }  // for grid_x

                }  // for grid_y

            }  // for anchor_idx

        }  // for batch_idx
    }

    void run(const std::vector<NetOutput>& outputs, std::vector<ObjectBuffer>& results) override
    {
        // 记录后处理时间
        TIMER_START_DEBUG(DET_POSTPROCESS_TIME_NAME);

        // 遍历每一层输出特征图, 解析并将多个特征图的结果保存到一个对象中
        for (uint32 i = 0; i < this->nl; ++i)
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

        LOG_DEFAULT_DEBUG("%s cost time: %s", this->to_string().c_str(),
                          TIMER_ELAPSED_STR_DEBUG(DET_POSTPROCESS_TIME_NAME).c_str());

        // 记录 NMS 时间
        TIMER_START_DEBUG(DET_NMS_TIME_NAME);

        // 判断一下, 如果没有检测目标, 就直接返回, 不会进行 NMS
        non_max_suppression(results, this->iou_thrs, this->agnostic);

        // 记录 NMS 时间
        LOG_DEFAULT_DEBUG("non_max_suppression cost time: %s", TIMER_ELAPSED_STR_DEBUG(DET_NMS_TIME_NAME).c_str());
    }
};
}  // namespace yolo

#endif  // !__DETPOSTPROCESSV5__H__
