/***
 * @Author       : gxs
 * @Date         : 2026-04-11 16:48:55
 * @LastEditors  : gxs
 * @LastEditTime : 2026-04-11 16:48:57
 * @FilePath     : /visionAlgorithm/lib/detector/BaseNet.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __BASENET__H__
#define __BASENET__H__

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "NetConfig.h"  // 模型配置
#include "NetOutput.hpp"
#include "common.hpp"  // 打印类别名称
#include "types.hpp"   // 类型定义

namespace yolo
{
typedef struct
{
    // onnx 模型路径
    std::string onnx_path = "";
} ModelPathParams;

class BaseNet
{
   public:
    /**
     * @description: 基类的析构函数
     * @return {*}
     */
    ~BaseNet() = default;

    /***
     * @description: 加载模型
     * @param param ModelPathParams& : 模型路径参数
     * @param device int32 : GPU编号, CPU: -1
     * @return
     */
    virtual bool load_model(const ModelPathParams& param, int32 device = -1) = 0;

    /***
     * @description: 推理的入口函数,
     * 输入是一个 batch 的 images[已经resize到模型输入大小的图片],
     * 输出一个 batch 的三张特征图
     * @param images_bgr std::vector<cv::Mat>& : 输入的图像数据
     * @param outputs std::vector<NetOutput>& : 第一层 vector 是三个 hw 的输出特征层; 第二层 vector 是 (b, c, h, w)
     * 的数据
     * @return
     */
    virtual bool run(const std::vector<cv::Mat>& images_bgr, std::vector<NetOutput>& outputs) = 0;

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
inline std::ostream& operator<<(std::ostream& os, const BaseNet& obj)
{
    std::cout << obj.to_string();
    return os;
}

}  // namespace yolo

#endif  // !__BASENET__H__