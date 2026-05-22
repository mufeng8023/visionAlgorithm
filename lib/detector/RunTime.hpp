/***
 * @Author       : gxs
 * @Date         : 2026-05-17 12:53:15
 * @LastEditors  : gxs
 * @LastEditTime : 2026-05-17 12:53:17
 * @FilePath     : /visionAlgorithm/lib/detector/RunTime.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __RUNTIME__H__
#define __RUNTIME__H__

#include "OpencvNet.hpp"
#include "V5DetPostProcess.hpp"
#include "YoloObject.h"
#include "logging.hpp"
#include "utils.hpp"

namespace yolo
{

/***
 * @description:
 * 加载 ini 配置文件, 初始化配置
 * 根据配置文件中的配置, 初始化模型, 后处理, 并初始化运行时需要使用的变量,
 */
class RunTime
{
   private:
    // 配置
    DetectionNetConfig config;
    // 网络运行类, 使用智能指针, 使用多态, 可以方便的切换模型
    std::shared_ptr<BaseNet> net;
    // 后处理类, 使用智能指针, 使用多态, 可以方便的切换模型
    std::shared_ptr<BasePostProcess> postProcess;

    // 保存网络输出的变量
    std::vector<NetOutput> net_outputs;
    // 最后输出的变量
    std::vector<ObjectBuffer> results;

   public:
    /***
     * @description:
     * @param config_path string& :
     * @return
     */
    RunTime(const std::string& config_path,           //
            const ModelPathParams& param,             //
            const std::string& net_bench = "opencv",  //
            int32 device = -1)
    {
        // 加载配置文件
        parser_ini_det_net_config(config_path, this->config);

        // 初始化网络输出变量
        for (uint32 i = 0; i < this->config.nl; ++i)
        {
            this->net_outputs.emplace_back(this->config.batch_size,            // batch_size
                                           this->config.na * this->config.no,  // 输出通道数量
                                           this->config.net_out_h[i],          // 输出高度
                                           this->config.net_out_w[i]           // 输出宽度
            );
        }

        // 初始化模型
        if (net_bench == "opencv")
        {
            this->net = std::make_shared<OpencvNet>(this->config);
            this->net->load_model(param, device);

            LOG_DEFAULT_INFO("Init OpenCVNet Success!");
        }
        // TODO: 后续实现其他框架
        else
        {
            LOG_DEFAULT_ERROR("net bench: %s not support;", net_bench.c_str());
        }

        // 初始化后处理
        // 初始化 ObjectBuffer
        uint32 extra_dim = 0;
        switch (this->config.task)
        {
            case TaskType::detection:
            {
                extra_dim = 0;  // 检测任务, 除了目标检测信息外, 没有别的信息

                switch (this->config.model_type)
                {
                    case ModelType::yolov5:
                        // 初始化V5DetPostProcess
                        this->postProcess = std::make_shared<V5DetPostProcess>(this->config);
                        LOG_DEFAULT_INFO("Init V5DetPostProcess Success!");
                        break;

                    // TODO: 后续实现 yolov8 / yolo11 / yolo26 / yolov9 / yolov10 / yolov12 等
                    default:
                        LOG_DEFAULT_ERROR("model type: %s not support;",
                                          model_type_to_string(this->config.model_type).c_str());
                        break;
                }
                break;
            }

            // TODO: 待实现的其他任务
            default:
                LOG_DEFAULT_ERROR("task: %s not support;", task_type_to_string(this->config.task).c_str());
                break;
        }

        // 开始初始化 ObjectBuffer
        for (uint32 i = 0; i < this->config.batch_size; ++i)
        {
            // 原地构造
            this->results.emplace_back(this->config.max_det, extra_dim);
            this->results[i].clear();
            LOG_DEFAULT_INFO(
                "Init ObjectBuffer Success! batch: %d, config max_det: %d, config extra_dim: %d, buffer max_count: %d, "
                "buffer stride: %d",
                i, this->config.max_det, extra_dim, this->results[i].get_max_count(), this->results[i].get_stride());
        }
    }

    // 禁止各种复制拷贝, 只引用传递
    /***
     * @description: 禁止各种复制拷贝
     * @return
     */
    RunTime(const RunTime& other) = delete;

    /***
     * @description: 禁止各种复制拷贝
     * @return
     */
    RunTime& operator=(const RunTime& other) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    RunTime(RunTime&& other) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    RunTime& operator=(RunTime&& other) = delete;

    /***
     * @description:
     * @return
     */
    ~RunTime() = default;

    /***
     * @description:
     * @param images_bgr const std::vector<cv::Mat>& :
     * @param det_results std::vector<std::vector<YoloObject>>& :
     * @return
     */
    void operator()(const std::vector<cv::Mat>& images_bgr, std::vector<std::vector<YoloObject>>& det_results)
    {
        LOG_DEFAULT_INFO("RunTime Start!");
        // 推理模型
        this->net->run(images_bgr, this->net_outputs);

        // 后处理
        this->postProcess->run(this->net_outputs, this->results);

        // this->results: std::vector<ObjectBuffer> 是在连续内存中保存的, 方便操作
        // 将 this->results 结果转移到 std::vector<std::vector<YoloObject>> 中, 方便后处理
        // 需要使用两个 for 循环 遍历 this->results 中的结果, 之后对 YoloObject 进行赋值;
        for (uint32 batch_idx = 0; batch_idx < this->config.batch_size; ++batch_idx)
        {
            // 根据 batch_idx 获取对应的 ObjectBuffer
            for (uint32 result_idx = 0; result_idx < this->results[batch_idx].get_obj_count(); ++result_idx)
            {
                // 获取 ObjectBuffer 中的结果
                // 每一组检测结果的长度
                uint32 stride = this->results[batch_idx].get_stride();

                // 每一组检测结果的首地址
                float32* result_base_addr = this->results[batch_idx].at(result_idx);
                // 获取检测结果的基础边界框信息
                float32 x = result_base_addr[ObjectOffset::x_center];
                float32 y = result_base_addr[ObjectOffset::y_center];
                float32 w = result_base_addr[ObjectOffset::width];
                float32 h = result_base_addr[ObjectOffset::height];
                float32 score = result_base_addr[ObjectOffset::score];
                uint32 cls_id = static_cast<uint32>(result_base_addr[ObjectOffset::cls_id]);

                // 创建 YoloObject 对象
                det_results[batch_idx].emplace_back(YoloObject());
                det_results[batch_idx].back().box.cls_id = cls_id;
                det_results[batch_idx].back().box.score = score;
                det_results[batch_idx].back().box.x1 = x - w / 2;
                det_results[batch_idx].back().box.y1 = y - h / 2;
                det_results[batch_idx].back().box.x2 = det_results[batch_idx].back().box.x1 + w;
                det_results[batch_idx].back().box.y2 = det_results[batch_idx].back().box.y1 + h;

                // 根据任务类型获取额外信息
                switch (this->config.task)
                {
                    case TaskType::detection:
                        det_results[batch_idx].back().type = TaskType::detection;
                        break;
                    case TaskType::pose:
                        det_results[batch_idx].back().type = TaskType::pose;
                        det_results[batch_idx].back().kpts.clear();

                        // 获取关键点信息
                        for (uint32 keypoint_idx = 0; keypoint_idx < this->config.kpt_count; ++keypoint_idx)
                        {
                            // 当前关键点起始索引
                            uint32 kpt_start_idx = ObjectOffset::extra_start + keypoint_idx * this->config.kpt_dim;
                            // 添加关键点信息
                            det_results[batch_idx].back().kpts.emplace_back(KeyPoint());
                            det_results[batch_idx].back().kpts.back().x = result_base_addr[kpt_start_idx + 0];
                            det_results[batch_idx].back().kpts.back().y = result_base_addr[kpt_start_idx + 1];

                            if (this->config.kpt_dim == 3)
                            {
                                det_results[batch_idx].back().kpts.back().score = result_base_addr[kpt_start_idx + 3];
                            }
                            else if (this->config.kpt_dim == 2)
                            {
                                det_results[batch_idx].back().kpts.back().score = 1.0;
                            }
                        }
                        break;

                    // TODO: 添加其他任务类型
                    default:
                        break;
                }
            }  // for this->results[batch_idx].get_obj_count()

            // 清空上一批的推理结果
            this->results[batch_idx].clear();
            LOG_DEFAULT_DEBUG("batch: %d, buffer clear!", batch_idx);
        }  // for this->config.batch_size
    }
};

}  // namespace yolo

#endif  // !__RUNTIME__H__