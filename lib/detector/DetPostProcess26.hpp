/***
 * @Author       : gxs
 * @Date         : 2026-05-30 16:18:36
 * @LastEditors  : gxs
 * @LastEditTime : 2026-05-30 16:18:36
 * @FilePath     : /visionAlgorithm/lib/detector/DetPostProcess26.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __DETPOSTPROCESS26__H__
#define __DETPOSTPROCESS26__H__

#include "detector/BasePostProcess.hpp"

namespace yolo
{
class DetPostProcess26 : public BasePostProcess
{
   private:
    // 推理的batch
    uint32 batch_size = 0;
    // 类别数量
    uint32 nc = 0;

    // 反量化系数
    std::vector<float32> scale_outputs = {1.0, 1.0, 1.0};

    // 是否存在conf
    bool has_conf = false;
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

   private:
    class ClassInfo
    {
       public:
        uint32 cls_id = 0;
        float32 score = 0.0;

        // 添加这个构造函数
        ClassInfo(uint32 cls_id, float32 score) : cls_id(cls_id), score(score) {}

        // 如果代码其他地方需要创建空实例, 保留默认构造函数
        ClassInfo() = default;
    };

   public:
    DetPostProcess26(const DetectionNetConfig& config)
    {
        // 初始化各种参数
        this->batch_size = config.batch_size;
        this->nc = config.nc;
        this->has_conf = false;  // NOTE: 默认没有置信度

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
        this->anchors.clear();  // NOTE: anchor-free默认为空
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
    DetPostProcess26(const DetPostProcess26& other) = delete;

    /***
     * @description: 禁用赋值操作符, 防止对象被赋值
     * @return
     */
    DetPostProcess26& operator=(const DetPostProcess26& other) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    DetPostProcess26(DetPostProcess26&& other) = default;

    /***
     * @description: 禁用移动赋值操作符, 防止对象被移动赋值
     * @return
     */
    DetPostProcess26& operator=(DetPostProcess26&& other) = default;

    /***
     * @description: 析构函数
     * @return
     */
    ~DetPostProcess26() = default;

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

                // 核心优化: 在进入循环之前, 先将 x / y / w / h / conf / nc 的各自通道的 [绝对首地址] 指针
                // 彻底消除了原代码最内层中类似 [feature_addr + k * channel_stride] 的复杂乘法寻址
                const float32* x1_ptr = base_output_ptr + 0 * grid_size;
                const float32* y1_ptr = base_output_ptr + 1 * grid_size;
                const float32* x2_ptr = base_output_ptr + 2 * grid_size;
                const float32* y2_ptr = base_output_ptr + 3 * grid_size;
                // 类别首地址, 类别通道的指针定位同样利用预计算的行首, 保持 offset 的连续性
                const float32* class_ptr = base_output_ptr + 4 * grid_size;

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

                        // typedef struct
                        // {
                        //     uint32 cls_id = 0;
                        //     float32 score = 0.0;
                        // } ClassInfo;
                        // 保存类别分数和类别索引
                        std::vector<ClassInfo> class_infos;
                        if (this->agnostic)
                        {
                            // 如果是agnostic, 每个锚点仅有一个预测结果
                            class_infos.reserve(1);
                            // 初始化一个类别信息
                            class_infos.emplace_back(0U, -1.0f);
                        }
                        else
                        {
                            // 如果不是agnostic, 每个锚点有多个预测结果
                            class_infos.reserve(this->nc);  // 预分配空间, 避免后续频繁的内存分配和释放
                        }

                        // 定义一个临时指针指向当前类别的通道
                        const float32* cur_class_ptr = class_ptr;
                        // 遍历所有类别, 找出最大分数和对应类别
                        for (uint32 class_idx = 0; class_idx < this->nc; ++class_idx)
                        {
                            // 类别通道的指针定位同样利用预计算的行首, 保持 offset 的连续性
                            float32 class_score = cur_class_ptr[grid_offset] * scale_output;

                            if (this->agnostic)
                            {
                                // 如果是agnostic, 每个锚点仅有一个预测结果
                                // 需要找一个最大分数和类别作为预测结果
                                if (class_infos[0].score < class_score)
                                {
                                    class_infos[0].score = class_score;
                                    class_infos[0].cls_id = class_idx;
                                }
                            }
                            else
                            {
                                // 如果不是agnostic, 每个锚点有多个预测结果
                                // 只要对应类别分数大于设置的阈值就认为是预测结果
                                if (class_score > this->conf_thrs[class_idx])
                                {
                                    // 记录类别分数和类别索引
                                    class_infos.emplace_back(class_idx, class_score);
                                }
                            }

                            // 更新类别指针 (指向下一个类别通道)
                            cur_class_ptr += grid_size;
                        }

                        // 如果是agnostic, 那么class_infos中只有一个元素,
                        // 还需要判断一下该位置的置信度是否大于阈值
                        // 小于阈值的时候, 跳过该位置
                        if (this->agnostic)
                        {
                            // 有且仅有一个目标
                            if (class_infos[0].score < this->conf_thrs[class_infos[0].cls_id])
                                continue;
                        }
                        else
                        {
                            // 可能存在没有预测结果的情况
                            if (class_infos.empty())
                                continue;
                        }

                        // 获取当前已检测到的目标数量
                        uint32 output_idx = result.get_obj_count();
                        // 扩展result缓冲区, 并标记为有效
                        result.expand_obj();
                        result.set_valid(output_idx, true);

                        // 解码边界框 (x1, y1, x2, y2) -> (x, y, w, h)
                        float32 dx1 = x1_ptr[grid_offset] * scale_output;
                        float32 dy1 = y1_ptr[grid_offset] * scale_output;
                        float32 dx2 = x2_ptr[grid_offset] * scale_output;
                        float32 dy2 = y2_ptr[grid_offset] * scale_output;

                        // x1坐标: (grid_x + 0.5 - value) * stride
                        // x1 = (grid_x + 0.5f - dx1) * stride;
                        // x = (x1 + x2) / 2 = (grid_x + 0.5f) * stride + (dx2 - dx1) * 0.5 * stride
                        result[output_idx][ObjectOffset::x_center] =
                            (grid_x + 0.5f) * stride + (dx2 - dx1) * 0.5f * stride;  // 中心点x

                        // y1坐标: (grid_y + 0.5 - value) * stride
                        // y1 = (grid_y + 0.5f - dy1) * stride;
                        // y = (y1 + y2) / 2 = (grid_y + 0.5f) * stride + (dy2 - dy1) * 0.5 * stride
                        result[output_idx][ObjectOffset::y_center] =
                            (grid_y + 0.5f) * stride + (dy2 - dy1) * 0.5f * stride;  // 中心点y

                        // x2坐标: (grid_x + 0.5 + value) * stride
                        // x2 = (grid_x + 0.5f + dx2) * stride;
                        // width = x2 - x1 = (grid_x + 0.5f + dx2) * stride - (grid_x + 0.5f - dx1) * stride
                        // width = (dx2 + dx1) * stride
                        result[output_idx][ObjectOffset::width] = (dx2 + dx1) * stride;  // width

                        // y2坐标: (grid_y + 0.5 + value) * stride
                        // y2 = (grid_y + 0.5f + dy2) * stride;
                        // height = y2 - y1 = (grid_y + 0.5f + dy2) * stride - (grid_y + 0.5f - dy1) * stride
                        // height = (dy2 + dy1) * stride
                        result[output_idx][ObjectOffset::height] = (dy2 + dy1) * stride;  // height

                        // 存储最终置信度和类别索引
                        result[output_idx][ObjectOffset::score] = class_infos[0].score;
                        result[output_idx][ObjectOffset::cls_id] = static_cast<float32>(class_infos[0].cls_id);

                        // 获取最后一个检测目标的数据指针
                        const float32* data = result[output_idx];

                        // 可能存在一个位置输出多个类别的情况, 所以使用 for
                        for (uint32 idx = 1; idx < class_infos.size(); ++idx)
                        {
                            // 现将最后一个目标复制一下
                            result.push_back(data);

                            // 更改目标的类别和对应的分数, 其余信息是一样的
                            // 获取当前已检测到的目标数量 - 1 是最后一个目标的索引
                            output_idx = result.get_obj_count() - 1;

                            // 复制的最后一个的目标信息, 需要修改 类别和分数
                            result[output_idx][ObjectOffset::score] = class_infos[idx].score;
                            result[output_idx][ObjectOffset::cls_id] = static_cast<float32>(class_infos[idx].cls_id);
                        }

                    }  // for grid_x

                }  // for grid_y

            }  // for anchor_idx

        }  // for batch_idx
    }

    void run(const std::vector<NetOutput>& outputs, std::vector<ObjectBuffer>& results) override
    {
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

        // 因为是端到端的 yolo26 不需要进行 NMS, 置信度大于 conf_thr 的都是最终的预测结果
        // 虽然不需要进行 nms 但是需要根据分数排序, 之后再取前 max_det 个 作为结果
        // 在 this->process_one 中不能对输出结果使用 max_det 进行拦截, 否则可能会导致输出结果丢失
        non_max_suppression(results,         //
                            this->iou_thrs,  //
                            this->agnostic,  //
                            this->max_det,   //
                            true             // end2end 标志必须是 true
        );
    }
};
}  // namespace yolo

#endif  // !__DETPOSTPROCESS26__H__