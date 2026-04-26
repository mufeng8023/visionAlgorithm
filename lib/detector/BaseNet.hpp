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

#include "NetOutput.hpp"
#include "common.hpp"  // 打印类别名称
#include "types.hpp"   // 类型定义

namespace yolo
{
class BaseNet
{
   public:
    /**
     * @description: 基类的析构函数
     * @return {*}
     */
    ~BaseNet() = default;

    /**
     * @description: 推理的入口函数,
     * 输入是一个 batch 的 images[已经resize到模型输入大小的图片],
     * 输出一个 batch 的三张特征图
     * @param {vector<cv::Mat>} &images 多张image
     * @param {vector<NetOutput>} &outputs 第一层 vector 是三个 hw 的输出特征层; 第二层 vector 是 (b, c, h, w) 的数据
     * @return {*}
     */
    virtual bool run(const std::vector<cv::Mat>& images, std::vector<NetOutput>& outputs) = 0;

    /**
     * @description: 输出类别信息, 比如类别名称啥的
     * @return {*}
     */
    std::string to_string() const
    {
        // 使用模板函数 get_class_name 获取类名
        return get_class_name<BaseNet>(*this);
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