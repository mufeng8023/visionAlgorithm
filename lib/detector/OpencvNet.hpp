/***
 * @Author       : gxs
 * @Date         : 2026-04-27 13:11:30
 * @LastEditors  : gxs
 * @LastEditTime : 2026-04-27 13:11:31
 * @FilePath     : /visionAlgorithm/lib/detector/OpencvNet.hpp
 * @Description  :
 * !默认的情况: 输入是uint8, 输出是float32
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __OPENCVNET__H__
#define __OPENCVNET__H__

#include "BaseNet.hpp"
#include "logging.hpp"

namespace yolo
{

/***
 * @description: OpencvNet 从 onnx 中读取网络
 * @return
 */
class OpencvNet : public BaseNet
{
   private:
    // 一个 opencv 的指针
    cv::dnn::Net net;
    // batch size
    uint32 batch_size = 0;
    // 类别数量
    uint32 nc = 0;

    // 输入图片大小
    uint32 input_width = 0;
    uint32 input_height = 0;
    uint32 input_channels = 0;

    // 输出特征图的 strides
    std::vector<uint32> strides = {8, 16, 32};  // 可以扩展为任意特征图数量
    // anchor 信息, 如果是空的表示是 anchor-free
    std::vector<std::vector<float32>> anchors = {};

    // 需要自己计算的
    // 每个特征图的 anchor 数量, 如果是 anchor-free 就是 1
    uint32 na = 0;
    // 输出维度
    uint32 no = 0;
    // 输出特征图的宽高
    uint32 nl = 0;
    // 每个特征图的宽高
    std::vector<uint32> net_out_h = {};
    std::vector<uint32> net_out_w = {};
    // 输出的每个特征图的数据个数, batch_size * na * no * net_out_h[i] * net_out_w[i]
    std::vector<uint32> output_len = {};  // 每个特征图的输出数据大小

    // onnx 模型的输出名称, 加载模型时只获取一次, 后续复用
    std::vector<std::string> out_layer_names;

    // 输入的 batch 数据, 大小为 (batch_size, input_channels, input_height, input_width)
    cv::Mat inputBatch;

   public:
    /***
     * @description: 构造函数
     * @param config DetectionNetConfig& : 配置信息
     * @return
     */
    OpencvNet(const DetectionNetConfig& config)
    {
        this->batch_size = config.batch_size;
        this->nc = config.nc;
        this->input_width = config.input_width;
        this->input_height = config.input_height;
        this->input_channels = config.input_channels;
        this->strides = config.strides;
        this->anchors = config.anchors;
        this->na = config.na;
        this->no = config.no;
        this->nl = config.nl;
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
    OpencvNet(const OpencvNet& other) = delete;

    /***
     * @description: 禁用赋值操作符, 防止对象被赋值
     * @return
     */
    OpencvNet& operator=(const OpencvNet& other) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    OpencvNet(OpencvNet&& other) = default;

    /***
     * @description: 禁用移动赋值操作符, 防止对象被移动赋值
     * @return
     */
    OpencvNet& operator=(OpencvNet&& other) = default;

    /***
     * @description: 析构函数
     * @return
     */
    ~OpencvNet() = default;

    /***
     * @description: 加载模型, 从 onnx 文件中加载网络, 并设置计算设备
     * @param onnx_path string& : onnx 文件路径
     * @param device int32 : 计算设备, -1 表示 CPU, 其他表示 GPU 设备编号
     * @return
     */
    bool load_model(const ModelPathParams& param, int32 device = -1) override
    {
        try
        {
            // 加载网络
            this->net = cv::dnn::readNetFromONNX(param.onnx_path);

            // 检查神经网络模型是否为空
            if (this->net.empty())
            {
                // 记录错误日志：模型加载失败, 包含模型信息和路径信息
                LOG_DEFAULT_ERROR("%s: %s Load ONNX model failed, net is empty.", this->to_string().c_str(),
                                  param.onnx_path.c_str());
                // 抛出运行时异常, 提示模型加载失败
                throw std::runtime_error("Failed to load model from " + param.onnx_path);
            }

            // 配置计算设备
            if (device <= -1)
            {
                // 强制使用 CPU
                // 设置神经网络推理的后端为OpenCV
                this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                // 设置神经网络推理的目标设备为CPU
                this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

                LOG_DEFAULT_INFO("%s: Inference mode set to CPU.", this->to_string().c_str());
            }
            else
            {
                // 使用 GPU
                // 校验 GPU 设备可用性
                int32 gpu_count = cv::cuda::getCudaEnabledDeviceCount();

                if (gpu_count <= 0)
                {
                    // 记录警告日志, 提示未检测到CUDA GPU, 将回退到CPU模式
                    LOG_DEFAULT_WARN("%s: No CUDA GPUs detected. Falling back to CPU.", this->to_string().c_str());
                    // 使用OpenCV自身的神经网络实现作为计算后端
                    this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                    // 设置神经网络的目标计算设备为CPU
                    this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                }
                else if (device >= gpu_count)
                {
                    // 核心逻辑: 判断请求的 device 序号是否越界
                    LOG_DEFAULT_ERROR("%s: Requested GPU device %d, but only %d GPU(s) available. Load failed.",
                                      this->to_string().c_str(), device, gpu_count);
                    return false;
                }
                else
                {
                    // 正常启动 GPU 模式
                    // 设置神经网络推理时使用的计算后端为CUDA，以利用NVIDIA GPU进行加速计算
                    this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                    // 设置神经网络推理时使用的目标设备为CUDA，确保计算在GPU上执行
                    this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

                    // 设置具体的 GPU 设备编号
                    cv::cuda::setDevice(device);
                    LOG_DEFAULT_INFO("%s: Inference mode set to GPU (Device: %d).", this->to_string().c_str(), device);
                }  // GPU 设备校验
            }  // GPU/CPU 判断

            this->out_layer_names = this->net.getUnconnectedOutLayersNames();

            return true;
        }  // try

        catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            LOG_DEFAULT_ERROR("%s: %s", this->to_string().c_str(), e.what());

            return false;
        }  // catch
    }  // load_model

    /***
     * @description: 预处理函数, 将输入的 images_bgr 预处理成模型输入要求的 inputBatch 格式
     * @param images_bgr std::vector<cv::Mat>& : 输入的图像数据
     * @return
     */
    void preprocess(const std::vector<cv::Mat>& images_bgr)
    {
        // 这里是预处理的核心逻辑, 需要根据模型的输入要求进行调整
        if (images_bgr.empty())
        {
            LOG_DEFAULT_ERROR("%s: Input images are empty. Preprocessing failed.", this->to_string().c_str());
            return;
        }

        // 定义模型需要的输入尺寸 (根据你的模型修改)
        // cv::Size(width, height) - 第一个参数是宽度, 第二个参数是高度
        cv::Size input_size(this->input_width, this->input_height);
        // 通过 scalefactor 和 mean 对数据进行 归一化;
        // img * scalefactor
        float64 scalefactor = 1.0;
        // 图像均值
        cv::Scalar mean = cv::Scalar(0., 0., 0.);
        // 是否交换 BGR 和 RGB
        bool swapRB = true;
        // 是否裁剪
        bool crop = false;
        // 输出数据类型, ONNX 模型通常使用 CV_32F (float32), 但这里根据需求使用 CV_8U (uint8)
        int ddepth = CV_8U;

        // 调用 OpenCV DNN 的批量处理函数
        // 该函数返回的 blob 维度为 [N, C, H, W]
        this->inputBatch = cv::dnn::blobFromImages(images_bgr,   // 图片 (h, w, 3)
                                                   scalefactor,  // scalefactor 对图像值进行缩放
                                                   input_size,   // size 参数调整图像大小
                                                   mean,         // mean 参数指定的均值从每个像素的每个通道中减去
                                                   swapRB,       // swapRB=true，它会将 BGR 转换为 RGB
                                                   crop,         // crop: 是否裁剪
                                                   ddepth        // ddepth: CV_8U: uin8; CV_32F: float32
        );
    }

    /***
     * @description: NOTE: 所有的ONNX输入必须是 Uint8, 导出ONNX的时候添加 Cast 算子, 将输入数据类型转换为 float32
     * @param images_bgr std::vector<cv::Mat>& : 输入的图像数据, 已经resize到模型输入大小的图片
     * @param outputs std::vector<NetOutput>& : 模型输出的特征图数据, 理论上是已经申请内存
     * @return
     */
    bool run(const std::vector<cv::Mat>& images_bgr, std::vector<NetOutput>& outputs) override
    {
        // 这里是推理的核心逻辑, 需要根据模型的输入输出进行调整
        try
        {
            // 预处理输入图像
            this->preprocess(images_bgr);

            if (this->inputBatch.empty())
            {
                LOG_DEFAULT_ERROR("%s: Preprocessed input batch is empty. Inference failed.",
                                  this->to_string().c_str());
                return false;
            }

            // 设置网络输入
            this->net.setInput(this->inputBatch);

            // 前向推理, 获取输出特征图
            std::vector<cv::Mat> net_outputs;

            // !默认的情况: 输入是uint8, 输出是float32
            this->net.forward(net_outputs, this->out_layer_names);

            if (net_outputs.empty())
            {
                LOG_DEFAULT_ERROR("%s: Forward pass returned empty outputs. Inference failed.",
                                  this->to_string().c_str());
                return false;
            }
            else if (net_outputs.size() != outputs.size())
            {
                // 输出数量不匹配, 记录错误日志
                LOG_DEFAULT_ERROR(
                    "%s: Number of outputs from the network (%zu) does not match expected (%zu). Inference failed.",
                    this->to_string().c_str(), net_outputs.size(), outputs.size());

                return false;
            }

            // 使用两层 for 动态 适配
            // OpenCV 输出的特征图是按照 节点名称排序的,
            // 为了防止出现 节点名称顺序 和 特征图大小 顺序不一致的情况, 这里使用两层 for 循环来匹配
            // yolov8n-p2 输入宽640 输出四个特征图 160 -> 80 -> 40 -> 20 (4个)
            // 输出节点名称为 850 : 160, 911 : 80, 972 : 40, 1033 : 20
            // outLayerNames 顺序却是 1033, 850, 972, 911 导致出现的问题, 所以在这里替换为 动态匹配方式再赋值
            for (uint32 i = 0; i < net_outputs.size(); ++i)  // 遍历 OpenCV 输出特征图
            {
                for (uint32 j = 0; j < this->nl; ++j)  // 遍历 每一层
                {
                    // 保证 (batch_size, channel, height, width) 数量是正确的
                    if (net_outputs[i].size[0] == this->batch_size        // batch size
                        && net_outputs[i].size[1] == this->na * this->no  // channel
                        && net_outputs[i].size[2] == this->net_out_h[j]   // height
                        && net_outputs[i].size[3] == this->net_out_w[j]   // width
                    )
                    {
                        // 记录输出特征图的维度信息和数据个数, 方便调试
                        LOG_DEFAULT_DEBUG("out%d: [%d, %d, %d, %d], len=%d", j,  // 索引
                                          this->batch_size,                      // batch size
                                          this->na * this->no,                   // channel
                                          this->net_out_h[j],                    // height
                                          this->net_out_w[j],                    // width
                                          this->output_len[j]                    // 输出数据个数
                        );

                        // 数据转为 float32 类型, 并复制到 outputs[i] 中
                        float32* data_ptr = reinterpret_cast<float32*>(net_outputs[i].data);

                        if (data_ptr != nullptr                                     // 需要确保 data_ptr 不为空
                            && this->output_len[j] == outputs[j].get_buffer_size()  // 数据长度要一致
                        )
                        {
                            // 如果 data_ptr 不为空, 将数据复制到 outputs[i] 中
                            outputs[j].set_data(data_ptr, this->output_len[j]);
                        }
                        else
                        {
                            // 如果 data_ptr 为空, 记录错误日志并清空 outputs
                            LOG_DEFAULT_ERROR("%s: Output data pointer is null for output %d. Inference failed.",
                                              this->to_string().c_str(), j);
                            return false;
                        }

                        // 找到匹配的输出特征图, 跳出内层循环, 节约时间
                        break;
                    }
                }  // for 遍历 nl 动态适配特征图的不同size的结果
            }  // for 遍历 net_outputs

            return true;
        }  // try
        catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            LOG_DEFAULT_ERROR("%s: Inference failed. %s", this->to_string().c_str(), e.what());
            return false;
        }  // catch
    }  // run

};  // class OpencvNet
}  // namespace yolo

#endif  // !__OPENCVNET__H__