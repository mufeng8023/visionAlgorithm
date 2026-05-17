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
        // NOTE: 后续实现其他框架
        else
        {
            LOG_DEFAULT_ERROR("net bench: %s not support;", net_bench.c_str());
        }

        // 初始化后处理
        // 初始化 ObjectBuffer
        uint32 extra_dim = 0;
        if (this->config.task == "detection")
        {
            extra_dim = 0;  // 检测任务, 除了目标检测信息外, 没有别的信息
            if (this->config.model_type == "yolov5")
            {
                if (this->config.anchors.size() != 0)
                {
                    // 初始化V5DetPostProcess
                    this->postProcess = std::make_shared<V5DetPostProcess>(this->config);
                    LOG_DEFAULT_INFO("Init V5DetPostProcess Success!");
                }
                else  // anchors 为空, 说明是anchor-free的模型
                {
                    // 待实现的V8-anchor-free后处理
                }
            }
            // NOTE: 后续实现 yolov8 / yolo11 / yolo26 / yolov9 / yolov10 / yolov12 等
            else
            {
                LOG_DEFAULT_ERROR("model type: %s not support;", this->config.model_type.c_str());
            }
        }
        else if (this->config.task == "pose")
        {
            // pose模型, 除了目标检测信息还有关键点信息
            extra_dim = this->config.kpt_count * this->config.kpt_dim;
        }
        // NOTE: 待实现的其他任务
        else
        {
            LOG_DEFAULT_ERROR("task: %s not support;", this->config.task.c_str());
        }

        // 开始初始化 ObjectBuffer
        for (uint32 i = 0; i < this->config.batch_size; ++i)
        {
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
    RunTime(const RunTime&) = delete;

    /***
     * @description: 禁止各种复制拷贝
     * @return
     */
    RunTime& operator=(const RunTime&) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    RunTime(RunTime&&) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    RunTime& operator=(RunTime&&) = delete;

    /***
     * @description:
     * @return
     */
    ~RunTime() = default;

    void operator()(const std::vector<cv::Mat>& images_bgr)
    {
        LOG_DEFAULT_INFO("RunTime Start!");
        // 推理模型
        this->net->run(images_bgr, this->net_outputs);

        // 后处理
        this->postProcess->run(this->net_outputs, this->results);

        // 别的处理

        // 清空上一批的推理结果
        for (uint32 i = 0; i < this->config.batch_size; ++i)
        {
            this->results[i].clear();
            LOG_DEFAULT_DEBUG("batch: %d, buffer clear!", i);
        }
    }
};

}  // namespace yolo

#endif  // !__RUNTIME__H__