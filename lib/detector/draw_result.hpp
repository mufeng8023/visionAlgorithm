/***
 * @Author       : gxs
 * @Date         : 2026-05-24 17:27:58
 * @LastEditors  : gxs
 * @LastEditTime : 2026-05-24 17:28:00
 * @FilePath     : /visionAlgorithm/lib/detector/draw_result.hpp
 * @Description  : 检测结果可视化绘制工具
 *                 提供检测框绘制和姿态关键点绘制功能
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */

#ifndef __DRAW_RESULT__H__
#define __DRAW_RESULT__H__

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "detector/NetConfig.hpp"
#include "detector/YoloObject.h"
#include "types.hpp"

namespace yolo
{

/***
 * @description: 根据类别索引获取一个颜色 (HSL -> RGB/BGR)
 *               使用黄金比例 (137.508°) 在色环上均匀分布颜色,
 *               确保不同类别获得视觉上区分度高的颜色
 * @param {int32} index  类别索引, 不同索引生成不同色相的颜色
 * @param {bool}  is_rgb 返回颜色的类型:
 *                       true  -> (R, G, B) 顺序
 *                       false -> (B, G, R) 顺序 (OpenCV 默认 BGR)
 * @return {std::tuple<uint8, uint8, uint8>} 颜色三元组, 每个分量范围 [0, 255]
 */
static std::tuple<uint8, uint8, uint8> get_color(int32 index, bool is_rgb)
{
    // 定义一个 lambda 函数, 用于根据 HSL 转换为 RGB
    // p 和 q 是计算 RGB 时的中间值, t 是色相值, 计算公式源自 HSL 到 RGB 的转换公式
    auto hue_to_rgb = [](float32 p, float32 q, float32 t) -> float32
    {
        // 将 t 限制在 0 到 1 之间, 避免负值
        t = std::fmod(t, 1.0f);  // fmod 是求浮点数的余数
        if (t < 0)
            t += 1.0f;  // 若 t 为负, 则调整 t 为正值

        // 根据 t 的不同范围, 使用不同的公式计算 RGB 的某个分量
        if (t < 1.0f / 6)
            return p + (q - p) * 6 * t;  // 第一个色相区间, 线性插值

        if (t < 1.0f / 2)
            return q;  // 第二个色相区间, 保持 q 值

        if (t < 2.0f / 3)
            return p + (q - p) * (2.0f / 3 - t) * 6;  // 第三个色相区间, 线性插值

        return p;  // 第四个色相区间, 返回 p 值
    };

    // 计算色相 (h) , 通过 index 计算一个 0 到 1 之间的浮动值
    // 137.508 是一个系数 (黄金角度) , 用于控制色相的变化
    // fmod 用于确保结果在 360 以内, 然后除以 360 转换到 [0, 1] 范围
    float32 h = fmod(static_cast<float32>(index) * 137.508f, 360.0f) / 360.0f;

    // 设置亮度和饱和度, 亮度在 [0, 1] 之间, 饱和度也是
    float32 l = 0.6f, s = 0.95f;
    float32 r, g, b;

    // 如果饱和度为 0, 表示是灰色
    if (s == 0)
    {
        r = g = b = l;  // 将 r, g, b 都设置为亮度值
    }
    else
    {
        // 计算 q 和 p 的值, 它们是根据 HSL 转 RGB 的转换公式得出的中间值
        // q 和 p 的计算方式不同, 取决于亮度 l 的大小
        float32 q = (l < 0.5f) ? (l * (1 + s)) : (l + s - l * s);
        float32 p = 2 * l - q;

        // 使用 hue_to_rgb 函数来计算 RGB 各分量的值
        // h + 1/3 用于计算红色分量, h 用于计算绿色分量, h - 1/3 用于计算蓝色分量
        r = hue_to_rgb(p, q, h + 1.0f / 3);  // 红色分量
        g = hue_to_rgb(p, q, h);             // 绿色分量
        b = hue_to_rgb(p, q, h - 1.0f / 3);  // 蓝色分量
    }

    // 将 RGB 分量的浮点值限制在 0 到 1 范围内, 并转换为 0 到 255 的整数范围
    // 使用 static_cast<uint8> 将浮点数转换为无符号 8 位整数类型
    uint8 ub = static_cast<uint8>(std::max(0.0f, std::min(b, 1.0f)) * 255);  // 蓝色分量
    uint8 ug = static_cast<uint8>(std::max(0.0f, std::min(g, 1.0f)) * 255);  // 绿色分量
    uint8 ur = static_cast<uint8>(std::max(0.0f, std::min(r, 1.0f)) * 255);  // 红色分量

    if (is_rgb)
    {
        return std::make_tuple(ur, ug, ub);  // 返回 (R, G, B) 顺序
    }
    else
    {
        return std::make_tuple(ub, ug, ur);  // 返回 (B, G, R) 顺序 (OpenCV 默认)
    }
}

/***
 * @description: 只管绘制 Box (检测框)
 *               在图像上绘制每个检测到的目标的边界框和类别标签
 *               框线粗细和文字大小根据图像尺寸自适应调整
 *               标签位置会自动调整以避免超出图像边界
 * @param image_bgr   cv::Mat&               : 输入/输出图像 (BGR 格式), 结果直接绘制到这上面
 * @param det_results std::vector<YoloObject>& : 检测结果列表, 每个元素包含检测框坐标、类别和置信度
 * @param names       std::vector<std::string>& : 类别名称列表, 索引对应 YoloObject.box.cls_id
 * @return void
 */
void draw_detection_result(cv::Mat& image_bgr,                          //
                           const std::vector<YoloObject>& det_results,  //
                           const std::vector<std::string>& names)
{
    // 获取图像尺寸, 用于自适应计算绘制参数
    int32 img_h = image_bgr.rows;
    int32 img_w = image_bgr.cols;

    // 根据图像尺寸自适应计算框线粗细
    // 使用 (高+宽)/2 * 0.003 作为基准, 最小为 2 像素
    int32 box_thickness = std::max(static_cast<int32>(std::round((img_h + img_w) / 2.0 * 0.003)), 2);
    // 文字线条粗细比框线细 1 像素, 最小为 1 像素
    int32 text_thickness = std::max(box_thickness - 1, 1);
    // 字体大小与框线粗细成正比
    double fontScale = box_thickness / 4.0;
    // 使用 OpenCV 默认简单字体
    int32 fontFace = cv::FONT_HERSHEY_SIMPLEX;

    // 遍历每个检测结果
    for (size_t i = 0; i < det_results.size(); ++i)
    {
        const YoloObject& obj = det_results[i];

        // 从 YoloObject 中提取检测框坐标和类别信息
        // YoloObject.box 是 Box 结构体, 包含浮点坐标和类别信息
        int32 x1 = static_cast<int32>(obj.box.x1);          // 左上角 x 坐标
        int32 y1 = static_cast<int32>(obj.box.y1);          // 左上角 y 坐标
        int32 x2 = static_cast<int32>(obj.box.x2);          // 右下角 x 坐标
        int32 y2 = static_cast<int32>(obj.box.y2);          // 右下角 y 坐标
        int32 cls_id = static_cast<int32>(obj.box.cls_id);  // 类别 ID
        float32 conf = obj.box.score;                       // 置信度分数

        // 根据类别 ID 获取颜色 (BGR 格式, 用于 OpenCV 绘制)
        uint8 b, g, r;
        std::tie(b, g, r) = get_color(cls_id, false);

        // 绘制检测框 (矩形)
        cv::rectangle(image_bgr,            // 输入/输出图像
                      cv::Point(x1, y1),    // 左上角坐标
                      cv::Point(x2, y2),    // 右下角坐标
                      cv::Scalar(b, g, r),  // 颜色 (BGR 格式)
                      box_thickness,        // 线条粗细
                      cv::LINE_AA           // 线条类型 (抗锯齿)
        );

        // 构造标签文本: "类别名:置信度" (例如 "person:0.92")
        std::string text = names[cls_id] + ":" + cv::format("%.2f", conf);

        // 计算文本的尺寸, 用于后续定位
        int32 baseLine = 0;
        cv::Size textSize = cv::getTextSize(text,            // 文本内容
                                            fontFace,        // 字体类型
                                            fontScale,       // 字体大小
                                            text_thickness,  // 线条粗细
                                            &baseLine        // 基线高度
        );

        // 判断标签文本是否超出图像边界, 并据此调整文本位置
        // inside_x: 文本是否在右边界内 (预留 5 像素边距)
        bool inside_x = (img_w - x1 - textSize.width) >= 5;
        // inside_y: 文本是否在上边界内 (预留 8 像素边距)
        bool inside_y = (y1 - textSize.height) >= 8;

        // 计算文本的左下角坐标
        // 水平方向: 如果空间足够放在框的左上角, 否则放在框的右上角
        int32 text_l = inside_x ? x1 : (x1 - (img_w - x1 - textSize.width));
        // 垂直方向: 如果空间足够放在框的上方, 否则放在框的内部下方
        int32 text_b = inside_y ? (y1 - 6) : (y1 + textSize.height + 5);

        // 在文本位置绘制白色背景矩形, 提高文字可读性
        cv::rectangle(image_bgr,                                       //
                      cv::Point(text_l, text_b - textSize.height),     // 左上角坐标
                      cv::Point(text_l + textSize.width, text_b + 6),  // 右下角坐标
                      cv::Scalar(255, 255, 255),                       // 颜色 (白色)
                      -1,                                              // 线条粗细 (-1 表示填充)
                      cv::LINE_AA                                      // 线条类型 (抗锯齿)
        );

        // 在背景上绘制文本
        cv::putText(image_bgr,                  //
                    text,                       // 文本内容
                    cv::Point(text_l, text_b),  // 文本左下角坐标
                    fontFace,                   // 字体类型
                    fontScale,                  // 字体大小
                    cv::Scalar(b, g, r),        // 颜色 (BGR 颜色)
                    text_thickness,             // 线条粗细
                    cv::LINE_AA                 // 线条类型 (抗锯齿)
        );
    }
}

/***
 * @description: 只管绘制 KeyPoint (姿态关键点)
 *               内部先调用 draw_detection_result 绘制检测框,
 *               然后在每个检测到的目标上绘制姿态关键点和骨架连接线
 *               关键点置信度低于阈值时会被过滤不显示
 * @param image_bgr   cv::Mat&               : 输入/输出图像 (BGR 格式), 结果直接绘制到这上面
 * @param det_results std::vector<YoloObject>& : 检测结果列表, 每个元素包含检测框和关键点数据
 * @param names       std::vector<std::string>& : 类别名称列表, 传递给 draw_detection_result
 * @param kpt_thr     float32                : 关键点可视化的分数阈值, 默认 0.5
 *                                             只有 score >= kpt_thr 的关键点才会被绘制
 * @return void
 */
void draw_pose_result(cv::Mat& image_bgr,                          //
                      const std::vector<YoloObject>& det_results,  //
                      const std::vector<std::string>& names,       //
                      const float32 kpt_thr = 0.5f)
{
    // 先绘制检测框 (复用 draw_detection_result)
    draw_detection_result(image_bgr, det_results, names);

    // 获取图像尺寸, 用于边界检查
    int32 img_h = image_bgr.rows;
    int32 img_w = image_bgr.cols;

    // 定义默认的 skeleton 连接关系 (COCO 17 关键点格式)
    // 每个 pair 表示两个关键点索引之间的连线, 索引对应关键点数组中的位置
    // COCO 17 关键点索引:
    //   0: 鼻子    1: 左眼    2: 右眼    3: 左耳    4: 右耳
    //   5: 左肩    6: 右肩    7: 左肘    8: 右肘    9: 左腕
    //   10: 右腕   11: 左髋    12: 右髋   13: 左膝   14: 右膝
    //   15: 左踝   16: 右踝
    std::vector<std::vector<int32>> skeleton = {
        {0, 1},   {0, 2},   {1, 3},   {2, 4},            // 鼻子 -> 左/右眼 -> 左/右耳
        {5, 6},   {5, 7},   {7, 9},   {6, 8},  {8, 10},  // 左/右肩 -> 左/右肘 -> 左/右腕
        {5, 11},  {6, 12},                               // 左/右肩 -> 左/右髋
        {11, 12},                                        // 左髋 -> 右髋 (髋部连接)
        {11, 13}, {13, 15}, {12, 14}, {14, 16}           // 左/右髋 -> 左/右膝 -> 左/右踝
    };

    // 遍历每个检测结果, 绘制关键点和骨架
    for (size_t i = 0; i < det_results.size(); ++i)
    {
        const YoloObject& obj = det_results[i];

        // 只处理类型为 TaskType::pose 的目标
        if (obj.type != TaskType::pose)
        {
            continue;
        }

        // 获取当前目标的关键点数组 (KeyPoint 结构体包含 x, y, score)
        const std::vector<KeyPoint>& kpts = obj.kpts;

        // 如果关键点数组为空, 跳过当前目标
        if (kpts.empty())
        {
            continue;
        }

        // 关键点总数
        size_t kpt_count = kpts.size();

        // 绘制关键点 (圆圈)
        for (size_t j = 0; j < kpt_count; ++j)
        {
            // 获取当前关键点的置信度分数
            float32 v = kpts[j].score;

            // 如果置信度低于阈值, 跳过不绘制
            if (v < kpt_thr)
            {
                continue;
            }

            // 获取关键点的图像坐标
            int32 x = static_cast<int32>(kpts[j].x);
            int32 y = static_cast<int32>(kpts[j].y);

            // 根据关键点索引获取颜色 (BGR 格式)
            // 使用 j + 5 作为颜色索引, 避免与检测框颜色冲突
            uint8 b, g, r;
            std::tie(b, g, r) = get_color(static_cast<int32>(j + 5), false);

            // 绘制关键点 (实心圆, 半径 5 像素)
            cv::circle(image_bgr,            //
                       cv::Point(x, y),      // 圆心坐标
                       5,                    // 半径
                       cv::Scalar(b, g, r),  // 颜色 (BGR 颜色)
                       -1,                   // 线条粗细 (-1 表示填充)
                       cv::LINE_AA           // 线条类型 (抗锯齿)
            );
        }

        // 绘制骨架 (线段)
        for (size_t k = 0; k < skeleton.size(); ++k)
        {
            // 获取当前骨架线段两个端点的关键点索引
            int32 idx1 = skeleton[k][0];  // 第一个端点索引
            int32 idx2 = skeleton[k][1];  // 第二个端点索引

            // 确保索引不越界 (防止关键点数量不足时访问非法内存)
            if (idx1 >= static_cast<int32>(kpt_count)      // 第一个端点索引越界
                || idx2 >= static_cast<int32>(kpt_count))  // 第二个端点索引越界
            {
                continue;
            }

            // 获取两个端点的置信度分数
            float32 pos1_v = kpts[idx1].score;
            float32 pos2_v = kpts[idx2].score;

            // 如果任一关键点置信度低于阈值, 跳过该线段
            if (pos1_v < kpt_thr || pos2_v < kpt_thr)
            {
                continue;
            }

            // 获取两个端点的图像坐标
            int32 pos1_x = static_cast<int32>(kpts[idx1].x);
            int32 pos1_y = static_cast<int32>(kpts[idx1].y);
            int32 pos2_x = static_cast<int32>(kpts[idx2].x);
            int32 pos2_y = static_cast<int32>(kpts[idx2].y);

            // 边界检查: 如果关键点坐标在图像边界上 (模宽/高为 0) 或为负数, 跳过
            // 这通常表示该关键点无效或超出图像范围
            if (pos1_x % img_w == 0 || pos1_y % img_h == 0 || pos1_x < 0 || pos1_y < 0)
            {
                continue;
            }
            if (pos2_x % img_w == 0 || pos2_y % img_h == 0 || pos2_x < 0 || pos2_y < 0)
            {
                continue;
            }

            // 根据骨架线段索引获取颜色 (BGR 格式)
            // 使用 k + 5 作为颜色索引, 避免与检测框颜色冲突
            uint8 b, g, r;
            std::tie(b, g, r) = get_color(static_cast<int32>(k + 5), false);

            // 绘制骨架线段 (线宽 2 像素)
            cv::line(image_bgr,                  //
                     cv::Point(pos1_x, pos1_y),  // 第一个端点坐标
                     cv::Point(pos2_x, pos2_y),  // 第二个端点坐标
                     cv::Scalar(b, g, r),        // 颜色 (BGR 颜色)
                     2,                          // 线宽
                     cv::LINE_AA                 // 线条类型 (抗锯齿)
            );
        }  // 结束绘制骨架
    }  // 结束遍历每个检测结果
}

}  // namespace yolo

#endif  // !__DRAW_RESULT__H__
