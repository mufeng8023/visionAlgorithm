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

#include "DetPostProcess26.hpp"
#include "DetPostProcessV5.hpp"
#include "DetPostProcessV8.hpp"
#include "OpencvNet.hpp"
#include "PosePostProcessV5.hpp"
#include "PosePostProcessV8.hpp"
#include "YoloObject.h"
#include "draw_result.hpp"
#include "logging.hpp"
#include "utils.hpp"

namespace yolo
{

enum class ModelBench : uint8
{
    OpenCV = 0,
    count,
};  // 如果修改了这个枚举, 需要修改 model_bench_names

constexpr std::array<std::string_view, static_cast<size_t>(ModelBench::count)> ModelBenchNames = {
    "OpenCV",
};

/***
 * @description: 枚举转字符串 (O(1) 性能)
 * @param type ModelType :
 * @return
 */
inline std::string model_bench_to_string(ModelBench type)
{
    size_t index = static_cast<size_t>(type);
    if (index < ModelBenchNames.size())
    {
        return std::string(ModelBenchNames[index]);
    }
    return "unknown";
}

/***
 * @description: 字符串转枚举 (依然需要遍历, 但代码很干净)
 * @param str string_view :
 * @return
 */
inline ModelBench model_bench_from_string(std::string_view str)
{
    for (size_t i = 0; i < ModelBenchNames.size(); ++i)
    {
        if (ModelBenchNames[i] == str)
        {
            return static_cast<ModelBench>(i);
        }
    }
    throw std::invalid_argument("Unknown ModelBench string");
}

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

    // 输入的图像, resize 之后的图像
    // 初始化时先 resize 到对应个数, 后续不再更改, 提高内存复用率
    std::vector<cv::Mat> imgs_resized;
    // 输入图像 等比例缩放的信息 ratio, dw, dh
    std::vector<std::tuple<float32, int32, int32>> resize_info;

    // 保存网络输出的变量, 初始化时分配内存之后, 后续不再释放内存, 内存直接复用
    std::vector<NetOutput> net_outputs;
    // 最后输出的变量, 初始化时分配内存之后, 后续不再释放内存, 内存直接复用
    std::vector<ObjectBuffer> results;

   public:
    /***
     * @description:
     * @param config_path string& :
     * @return
     */
    RunTime(const std::string& config_path,                    //
            const ModelPathParams& param,                      //
            const ModelBench& net_bench = ModelBench::OpenCV,  //
            int32 device = -1)
    {
        // 加载配置文件
        parser_ini_det_net_config(config_path, this->config);

        // 初始化模型
        switch (net_bench)
        {
            case ModelBench::OpenCV:
                this->net = std::make_shared<OpencvNet>(this->config);
                if (!this->net->load_model(param, device))
                {
                    LOG_DEFAULT_ERROR("Init OpenCVNet Failed! load model:%s failed!", param.onnx_path.c_str());
                    throw std::runtime_error("Init OpenCVNet Failed!");
                }

                LOG_DEFAULT_INFO("Init OpenCVNet Success!");
                break;  // case ModelBench::OpenCV

            // TODO: 后续实现其他框架
            default:
                LOG_DEFAULT_ERROR("net bench: %s not support;", model_bench_to_string(net_bench).c_str());
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
                        // 初始化 DetPostProcessV5
                        this->postProcess = std::make_shared<DetPostProcessV5>(this->config);
                        LOG_DEFAULT_INFO("Init DetPostProcessV5 Success!");
                        break;  // case ModelType::yolov5

                    // ultralytics 中实现的模型后处理都是一样的
                    case ModelType::yolov3u:
                    case ModelType::yolov5u:
                    case ModelType::yolov6u:
                    case ModelType::yolov8:
                    case ModelType::yolov9:
                    case ModelType::yolo11:
                    case ModelType::yolo12:
                        // 初始化 DetPostProcessV8
                        this->postProcess = std::make_shared<DetPostProcessV8>(this->config);
                        LOG_DEFAULT_INFO("Init DetPostProcessV8 Success!");
                        break;  // case ModelType::yolov8

                    case ModelType::yolov10:
                    case ModelType::yolo26:
                        // 初始化 DetPostProcess26
                        this->postProcess = std::make_shared<DetPostProcess26>(this->config);
                        LOG_DEFAULT_INFO("Init DetPostProcess26 Success!");
                        break;  // case ModelType::yolov26

                    // TODO: 后续实现 yolo11 / yolo26 / yolov9 / yolov10 / yolov12 等
                    default:
                        LOG_DEFAULT_ERROR("model type: %s not support;",
                                          model_type_to_string(this->config.model_type).c_str());
                        break;
                }
                break;
            }  // case TaskType::detection

            case TaskType::pose:
            {
                extra_dim = this->config.kpt_count * this->config.kpt_dim;  // 姿态估计任务, 需要保存关键点信息

                switch (this->config.model_type)
                {
                    case ModelType::yolov5:
                        // 初始化 PosePostProcessV5
                        this->postProcess = std::make_shared<PosePostProcessV5>(this->config);
                        LOG_DEFAULT_INFO("Init PosePostProcessV5 Success!");
                        break;  // case ModelType::yolov5

                    // ultralytics 中实现的模型后处理都是一样的
                    case ModelType::yolov5u:
                    case ModelType::yolov8:
                    case ModelType::yolo11:
                        // 初始化 PosePostProcessV8
                        this->postProcess = std::make_shared<PosePostProcessV8>(this->config);
                        LOG_DEFAULT_INFO("Init PosePostProcessV8 Success!");
                        break;  // case ModelType::yolov8

                    default:
                        LOG_DEFAULT_ERROR("model type: %s not support;",
                                          model_type_to_string(this->config.model_type).c_str());
                        break;
                }
                break;
            }  // case TaskType::pose

            // TODO: 待实现的其他任务
            default:
                LOG_DEFAULT_ERROR("task: %s not support;", task_type_to_string(this->config.task).c_str());
                break;
        }

        // 初始化输入的图像, 先直接 resize , 后续直接通过 索引使用
        this->imgs_resized.resize(this->config.batch_size);
        this->resize_info.resize(this->config.batch_size);

        // 初始化网络输出变量
        for (uint32 i = 0; i < this->config.nl; ++i)
        {
            this->net_outputs.emplace_back(this->config.batch_size,            // batch_size
                                           this->config.na * this->config.no,  // 输出通道数量
                                           this->config.net_out_h[i],          // 输出高度
                                           this->config.net_out_w[i]           // 输出宽度
            );
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
        }  // 初始化 ObjectBuffer
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
     * @param images_bgr const std::vector<cv::Mat>& : 没有处理的原始图片
     * @param det_results vector<vector<YoloObject>>& : 检测结果 已经映射到 images_bgr 图像中, 用完之后记得 clear
     * @return
     */
    void operator()(const std::vector<cv::Mat>& images_bgr,  //
                    std::vector<std::vector<YoloObject>>& det_results)
    {
        LOG_DEFAULT_INFO("RunTime Start!");

        // 添加图片预处理 resize, 等比例缩放到 目标shape
        for (uint32 i = 0; i < images_bgr.size(); ++i)
        {
            this->resize_info[i] = pre_process_resize_img(images_bgr[i],                                  //
                                                          static_cast<int32>(this->config.input_height),  //
                                                          static_cast<int32>(this->config.input_width),   //
                                                          this->imgs_resized[i]                           //
            );
        }

        // 推理模型
        this->net->run(this->imgs_resized, this->net_outputs);

        // 后处理
        this->postProcess->run(this->net_outputs, this->results);

        // 先对 det_results 进行初始化, 后面直接使用索引访问
        // 下面循环直接通过 索引访问并覆盖各 batch 的数据, 避免重复释放和申请外层内存
        if (det_results.size() < this->config.batch_size)
        {
            det_results.resize(this->config.batch_size);
        }
        // det_results 外层 不用 clear(), 后续会直接使用索引访问, 可以覆盖上一次的结果

        // 图片 resize 的信息, 用于后续的坐标还原
        float32 ratio = 1.0;       // 缩放比例
        int32 dw = 0;              // 宽偏移
        int32 dh = 0;              // 高偏移
        float32 ratio_inv = 1.0f;  // 缩放比例的倒数, 避免更多除法

        // this->results: std::vector<ObjectBuffer> 是在连续内存中保存的, 方便操作
        // 将 this->results 结果转移到 std::vector<std::vector<YoloObject>> 中, 方便后处理
        // 需要使用两个 for 循环 遍历 this->results 中的结果, 之后对 YoloObject 进行赋值;
        for (uint32 batch_idx = 0; batch_idx < this->config.batch_size; ++batch_idx)
        {
            // 获取图片 resize 的信息, 用于后续的坐标还原
            std::tie(ratio, dw, dh) = this->resize_info[batch_idx];
            // 在外层循环提前计算好倒数, 避免内层循环重复计算除法
            // 加上安全保护, 防止分母为 0.0, 但是理论上前处理算出来的 ratio 不会为 0.0
            ratio_inv = (ratio > 0.0f) ? (1.0f / ratio) : 1.0f;

            // 创建一个当前 batch 的引用
            ObjectBuffer& batch_results = this->results[batch_idx];
            std::vector<YoloObject>& batch_det_results = det_results[batch_idx];
            // 清空当前 batch 的检测结果, 防止出现上一次检测结果残留
            batch_det_results.clear();  // size 清空了, capacity 不变

            // 检测结果数量不够, 提前扩容
            if (batch_det_results.capacity() < this->config.max_det)
            {
                batch_det_results.reserve(this->config.max_det);
            }

            // 根据 batch_idx 获取对应的 ObjectBuffer
            for (uint32 result_idx = 0; result_idx < batch_results.get_obj_count(); ++result_idx)
            {
                // 获取 ObjectBuffer 中的结果
                // 每一组检测结果的长度
                uint32 stride = batch_results.get_stride();

                // 每一组检测结果的首地址
                float32* result_base_addr = batch_results.at(result_idx);
                // 获取检测结果的基础边界框信息
                float32 x = result_base_addr[ObjectOffset::x_center];
                float32 y = result_base_addr[ObjectOffset::y_center];
                float32 w = result_base_addr[ObjectOffset::width];
                float32 h = result_base_addr[ObjectOffset::height];
                // 还原到原始图片大小
                x = (x - dw) * ratio_inv;  // 等价于 x = (x - dw) / ratio;
                y = (y - dh) * ratio_inv;  // 等价于 y = (y - dh) / ratio;
                w = w * ratio_inv;         // 等价于 w = w / ratio;
                h = h * ratio_inv;         // 等价于 h = h / ratio;

                float32 score = result_base_addr[ObjectOffset::score];
                uint32 cls_id = static_cast<uint32>(result_base_addr[ObjectOffset::cls_id]);

                // 创建 YoloObject 对象
                batch_det_results.emplace_back(YoloObject());
                batch_det_results.back().box.cls_id = cls_id;
                batch_det_results.back().box.score = score;
                batch_det_results.back().box.x1 = x - w * 0.5;
                batch_det_results.back().box.y1 = y - h * 0.5;
                batch_det_results.back().box.x2 = batch_det_results.back().box.x1 + w;
                batch_det_results.back().box.y2 = batch_det_results.back().box.y1 + h;

                // 根据任务类型获取额外信息
                switch (this->config.task)
                {
                    case TaskType::detection:
                        batch_det_results.back().type = TaskType::detection;
                        break;  // case TaskType::detection

                    case TaskType::pose:
                    {
                        batch_det_results.back().type = TaskType::pose;

                        if (batch_det_results.back().kpts.capacity() < this->config.kpt_count)
                        {
                            // reserve 更改 capacity, 不改变 size, 所以 reserve 提前扩容, 避免内层循环重复扩容
                            batch_det_results.back().kpts.reserve(this->config.kpt_count);
                        }
                        // clear() 是为了 清空上一次保存的检测结果, 防止出现上一次检测结果残留
                        batch_det_results.back().kpts.clear();

                        // 临时变量存储关键点信息
                        float32 value = 0.0;
                        // 获取关键点信息
                        for (uint32 keypoint_idx = 0; keypoint_idx < this->config.kpt_count; ++keypoint_idx)
                        {
                            // 当前关键点起始索引
                            uint32 kpt_start_idx = ObjectOffset::extra_start + keypoint_idx * this->config.kpt_dim;

                            if (kpt_start_idx + 2 > stride)
                            {
                                // 访问越界
                                LOG_DEFAULT_ERROR("kpt_start_idx + 2 > stride");
                            }

                            // 添加关键点信息
                            batch_det_results.back().kpts.emplace_back(KeyPoint());
                            // NOTE: 关键点信息已经映射回原始图片大小
                            value = result_base_addr[kpt_start_idx + 0];  // x坐标
                            // 等价于 (value - dw) / ratio
                            batch_det_results.back().kpts.back().x = (value - dw) * ratio_inv;
                            value = result_base_addr[kpt_start_idx + 1];  // y坐标
                            // 等价于 (value - dh) / ratio
                            batch_det_results.back().kpts.back().y = (value - dh) * ratio_inv;

                            if (this->config.kpt_dim == 3)
                            {
                                batch_det_results.back().kpts.back().score = result_base_addr[kpt_start_idx + 2];
                            }
                            else if (this->config.kpt_dim == 2)
                            {
                                batch_det_results.back().kpts.back().score = 1.0;
                            }
                            else
                            {
                                LOG_DEFAULT_ERROR("kpt_dim error! kpt_dim = %d, expected 2 or 3", this->config.kpt_dim);
                            }
                        }
                        break;
                    }  // case TaskType::pose

                    // TODO: 添加其他任务类型
                    default:
                        LOG_DEFAULT_ERROR("task type error!");
                        break;
                }  // switch (this->config.task)
            }  // for batch_results.get_obj_count()

            // 清空上一批的推理结果
            batch_results.clear();
            LOG_DEFAULT_DEBUG("batch: %d, buffer clear!", batch_idx);

        }  // for this->config.batch_size
    }  // operator()

    /***
     * @description:
     * @param image_bgr Mat& :  输入图片
     * @param det_results std::vector<YoloObject>& : 检测结果
     * @return
     */
    void draw_result(std::vector<cv::Mat>& images_bgr,  //
                     const std::vector<std::vector<yolo::YoloObject>>& det_results)
    {
        // 每张图单独绘制边界框
        for (uint32 i = 0; i < images_bgr.size(); ++i)
        {
            switch (this->config.task)
            {
                case TaskType::detection:
                    draw_detection_result(images_bgr[i], det_results[i], this->config.names);
                    break;  // case TaskType::detection

                case TaskType::pose:
                    draw_pose_result(images_bgr[i], det_results[i], this->config.names);
                    break;  // case TaskType::pose

                // TODO: 添加其他任务类型
                default:
                    break;
            }  // switch (this->config.task)
        }  // for (uint32 i = 0; i < images_bgr.size(); ++i)
    }
};

}  // namespace yolo

#endif  // !__RUNTIME__H__