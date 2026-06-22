/***
 * @Author       : gxs
 * @Date         : 2026-06-22 14:48:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 15:25:00
 * @FilePath     : /visionAlgorithm/lib/tracker/BoxObject.hpp
 * @Description  : BoxObject 结构体定义 -- 统一管理原始检测框与扩展检测框
 *
 *                 设计背景:
 *                 在目标跟踪中, 检测器 (YOLO) 输出的检测框可以直接用于跟踪匹配;
 *                 但对于小目标(如人脸, 几十像素大小), 检测框定位精度有限;
 *                 卡尔曼滤波器如果基于不精确的小框初始化, 估计会不稳定;
 *                 通过将边界框向外扩展一定比例, 可以给卡尔曼滤波器更宽松的搜索空间,
 *                 提高小目标跟踪的鲁棒性;
 *
 *                 设计原则:
 *                 1. ltwh[4]        -- 原始检测框 (left, top, width, height);
 *                    由检测器直接输出, 保存后永不修改;
 *                    外部用户读取跟踪结果时, 应使用此原始框;
 *                 2. ltwh_expand[4] -- 扩展后的检测框;
 *                    基于 ltwh + expand_box_rate 计算得到;
 *                    所有卡尔曼滤波器操作 (initiate / update / predict) 都使用此扩展框;
 *                    从卡尔曼 mean 写回时, 直接写入 ltwh_expand (无需 shrink 恢复);
 *                 3. score           -- 检测框置信度分数;
 *                 4. cls_id          -- 检测框类别 ID (即 label);
 *                 5. get_x1 / get_xyah / get_xyxy 等全部基于 ltwh_expand 计算;
 *                    确保所有匹配/门控操作都在扩展空间中一致进行;
 *
 *                 NOTE: BoxObject 仅是一个数据容器 + 扩展工具;
 *                       它不持有卡尔曼滤波器状态, 也不负责跟踪逻辑;
 *                       BoxObject 在跟踪器的检测结果处理阶段创建,
 *                       在 track->update() / track->re_activate() 中传入,
 *                       同时在创建轨迹时传入并初始化轨迹的 ltwh / ltwh_expand / cls_id / score;
 *
 *                 参考:
 *                   - /mnt/E/CodeFiles/C++/bytetracker/src/STrack.cpp
 *                     (raw_tlwh 对应 ltwh, _tlwh 对应 ltwh_expand)
 *                   - /mnt/E/CodeFiles/C++/DeepSORT/tracker/deepsort/src/track.cpp
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */

#ifndef __BOXOBJECT__H__
#define __BOXOBJECT__H__

#include <array>
#include <cmath>
#include <vector>

#include "types.hpp"  // uint8, float32, uint32 等基础类型

namespace tracker
{

/***
 * @description: BoxObject 边界框对象
 *
 * 这个结构体用来干什么的?
 *   在目标跟踪中, 每一帧检测器会输出很多检测框 (bounding box)。
 *   这些检测框需要被传递给跟踪器, 用于与已有轨迹进行匹配和更新。
 *
 *   但"给跟踪器用的框"和"检测器原始输出的框"之间, 可能存在一个"扩展"的关系:
 *   为了让小目标跟踪更稳定, 我们可以把检测框向外扩大一些 (expand),
 *   然后让卡尔曼滤波器在这个"扩大后的空间"中工作。
 *
 *   BoxObject 做的就是:
 *     同时存储 "原始框 (ltwh)" 和 "扩展框 (ltwh_expand)",
 *     以及 "置信度分数 (score)" 和 "类别 ID (cls_id)",
 *     并提供了两者之间的转换方法。
 *
 *   注意:
 *     当 expand_box_rate == 0.0 时 (默认),
 *     ltwh_expand 和 ltwh 的值完全一样,
 *     因此这个扩展机制对已有代码是"零开销抽象"。
 *
 *  BoxObject 的典型使用流程:
 *   第 1 步: 从检测器拿到原始框 + 置信度 + 类别
 *   float32 raw_ltwh[4] = {100, 200, 50, 100};
 *   float32 score = 0.85f;
 *   int32  cls_id = 0;  // 人
 *
 *   第 2 步: 创建 BoxObject, 自动计算扩展框, 保存 score 和 cls_id
 *   BoxObject det_box(raw_ltwh, score, cls_id, 0.2f);  // 扩展 20%
 *
 *   第 3 步: 传递给跟踪器, 轨迹自动更新 cls_id 和 score
 *   track->update(det_box, frame_id);
 *
 *   第 4 步: 读取跟踪结果
 *   float32 raw_result[4] = track->ltwh;           // 原始框 (不变)
 *   float32 expand_result[4] = track->ltwh_expand; // 扩展框 (卡尔曼修正后)
 *   float32 cls = track->cls_id;                   // 类别 ID
 *   float32 scr = track->score;                    // 置信度分数
 *
 *  █ 存储格式说明: ltwh = left, top, width, height
 *   ltwh[0] = l  (bounding box 左边 x1)
 *   ltwh[1] = t  (bounding box 上边 y1)
 *   ltwh[2] = w  (bounding box 宽度)
 *   ltwh[3] = h  (bounding box 高度)
 *
 *   为什么不直接存 x1y1x2y2?
 *     因为卡尔曼滤波器使用 xyah (center_x, center_y, aspect_ratio, height) 格式,
 *     而 xyah 从 ltwh 计算比从 x1y1x2y2 更方便:
 *       cx = l + w/2
 *       cy = t + h/2
 *       a  = w / h
 *       h  = h
 *
 *  █ 扩展算法 (与原始 ByteTrack 保持一致):
 *   扩展逻辑使用 int32 截断, 确保数值与 bytetracker 原版一致:
 *     add_w = int(w * rate)
 *     add_h = int(h * rate)
 *     l_new = max(l - add_w/2, 0)
 *     t_new = max(t - add_h/2, 0)
 *     w_new = w + add_w
 *     h_new = h + add_h
 *
 *   注意: 这里使用了 int32 截断, 因此扩展框的
 *         w_expand - w_orig 不一定是精确的 rate * w_orig,
 *         但与原版 ByteTrack 的行为保持 100% 一致。
 */
struct BoxObject
{
    // ================================================================
    // 公开字段
    // ================================================================

    // ltwh[4]: 原始检测框 (left, top, width, height)
    // 由检测器直接输出, 跟踪过程中永不修改;
    // 外部用户读取跟踪结果时, 应使用此原始框;
    float32 ltwh[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // ltwh_expand[4]: 扩展后的检测框
    // 基于 ltwh + expand_box_rate 计算得到;
    // 所有卡尔曼滤波器操作都使用此扩展框;
    // 从卡尔曼 mean 写回时, 直接写入 ltwh_expand (无需 shrink);
    float32 ltwh_expand[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // score: 检测框的置信度分数 (detection confidence)
    // 由检测器直接输出, 在跟踪过程中由 BoxObject 传递给轨迹;
    // 对应 BaseTrack::score;
    float32 score = 0.0f;

    // cls_id: 检测框的类别 ID (class / label)
    // 由检测器直接输出, 对应检测器的类别索引;
    // 对应 BaseTrack::cls_id;
    int32 cls_id = -1;

    // ================================================================
    // 构造函数
    // ================================================================

    /***
     * @description: 默认构造函数
     * 将所有字段初始化为 0;
     * 用于容器 (如 std::vector<BoxObject> 扩容);
     */
    BoxObject() = default;

    /***
     * @description: 有参构造函数 -- 从原始 ltwh + score + cls_id 创建 BoxObject
     *
     * 这是最常用的构造函数:
     *   从检测器拿到原始框、置信度、类别后, 调用此构造函数,
     *   它会自动根据 expand_box_rate 计算出 ltwh_expand,
     *   并保存 score 和 cls_id 供后续轨迹初始化使用。
     *
     * 使用示例:
     *   从 float 数组创建
     *   float32 det_ltwh[4] = {100, 200, 50, 100};
     *   BoxObject box(det_ltwh, 0.85f, 0, 0.2f);  // score=0.85, cls_id=0, 扩展 20%
     *
     *   从 vector 创建
     *   std::vector<float32> det_vec = {100, 200, 50, 100};
     *   BoxObject box(det_vec, 0.92f, 1, 0.15f);  // score=0.92, cls_id=1, 扩展 15%
     *
     *   不扩展 (默认行为)
     *   BoxObject box(det_ltwh, 0.85f, 0, 0.0f);  // ltwh_expand == ltwh
     *
     * @param ltwh_data       const float32* : 原始检测框 ltwh[l, t, w, h];
     *                                         指针需指向有效数据, 长度至少 4;
     * @param score           float32        : 检测框置信度分数;
     * @param cls_id          int32          : 检测框类别 ID (label);
     * @param expand_box_rate float32        : 边界框扩展率;
     *                                         取值范围 [0.0, 1.0], 0.0 表示不扩展;
     *                                         典型值: 0.0 (不扩展) / 0.1 (10%) / 0.2 (20%);
     */
    BoxObject(const float32* ltwh_data,  //
              float32 score,             //
              int32 cls_id,              //
              float32 expand_box_rate = 0.0f)
        : score(score), cls_id(cls_id)
    {
        // 步骤 1: 保存原始框 (ltwh);
        this->ltwh[0] = ltwh_data[0];
        this->ltwh[1] = ltwh_data[1];
        this->ltwh[2] = ltwh_data[2];
        this->ltwh[3] = ltwh_data[3];

        // 步骤 2: 计算扩展框 (ltwh_expand);
        // 先拷贝原始值, 再在原地扩展;
        this->ltwh_expand[0] = ltwh_data[0];
        this->ltwh_expand[1] = ltwh_data[1];
        this->ltwh_expand[2] = ltwh_data[2];
        this->ltwh_expand[3] = ltwh_data[3];
        this->expand_static(this->ltwh_expand, expand_box_rate);
    }

    /***
     * @description: 从 std::vector<float32> 创建 BoxObject
     *               与上面的 float* 版本功能一致, 只是为了接口方便;
     *               如果 vector 长度不足 4, 会断言失败 (但依赖于 std::vector 的 at());
     * @param ltwh_vec        const std::vector<float32>& : 原始检测框 ltwh[l, t, w, h];
     * @param score           float32                     : 检测框置信度分数;
     * @param cls_id          uint32                      : 检测框类别 ID (label);
     * @param expand_box_rate float32                     : 边界框扩展率;
     */
    BoxObject(const std::vector<float32>& ltwh_vec,  //
              float32 score,                         //
              uint32 cls_id,                         //
              float32 expand_box_rate = 0.0f)
        : score(score), cls_id(cls_id)
    {
        // 安全拷贝, 如果越界会抛出 std::out_of_range;
        for (int32 i = 0; i < 4; ++i)
        {
            this->ltwh[i] = ltwh_vec.at(i);
            this->ltwh_expand[i] = ltwh_vec.at(i);
        }
        this->expand_static(this->ltwh_expand, expand_box_rate);
    }

    /***
     * @description: 从 std::array<float32, 4> 创建 BoxObject
     *               与上面的 float* 版本功能一致;
     * @param ltwh_arr        const std::array<float32, 4>& : 原始检测框 ltwh[l, t, w, h];
     * @param score           float32                      : 检测框置信度分数;
     * @param cls_id          uint32                       : 检测框类别 ID (label);
     * @param expand_box_rate float32                      : 边界框扩展率;
     */
    BoxObject(const std::array<float32, 4>& ltwh_arr,  //
              float32 score,                           //
              uint32 cls_id,                           //
              float32 expand_box_rate = 0.0f)
        : BoxObject(ltwh_arr.data(), score, cls_id, expand_box_rate)
    {
        // 委托给 float* 构造函数;
    }

    // ================================================================
    // 设置方法
    // ================================================================

    /***
     * @description: 同时设置原始框、扩展框、score、cls_id
     *               从原始 ltwh + 扩展率重新计算 ltwh_expand;
     *
     * @param ltwh_data       const float32* : 新的原始检测框 [l, t, w, h];
     * @param score           float32        : 检测框置信度分数;
     * @param cls_id          uint32         : 检测框类别 ID (label);
     * @param expand_box_rate float32        : 边界框扩展率;
     */
    void set_ltwh(const float32* ltwh_data,  //
                  float32 score,             //
                  uint32 cls_id,             //
                  float32 expand_box_rate = 0.0f)
    {
        // 更新原始框;
        this->ltwh[0] = ltwh_data[0];
        this->ltwh[1] = ltwh_data[1];
        this->ltwh[2] = ltwh_data[2];
        this->ltwh[3] = ltwh_data[3];

        // 更新 score 和 cls_id;
        this->score = score;
        this->cls_id = cls_id;

        // 重新计算扩展框;
        this->recompute_expand(expand_box_rate);
    }

    /***
     * @description: 基于当前的 ltwh 重新计算 ltwh_expand
     *
     * 什么时候用这个方法?
     *   当 ltwh 被外部修改后 (未通过 set_ltwh 或 set_ltwh_only),
     *   需要手动调用 recompute_expand() 保持 ltwh_expand 与 ltwh 一致;
     *
     *   注意: 一般不会出现这种情况, 因为 ltwh 是"保留原始数据"的,
     *   在轨迹生命周期中, ltwh 只应该在 update() / re_activate() 时通过 set_ltwh 更新。
     *
     * @param expand_box_rate float32 : 边界框扩展率;
     */
    void recompute_expand(float32 expand_box_rate)
    {
        // 更新扩展框 = 原始框 + 扩展;
        this->ltwh_expand[0] = this->ltwh[0];
        this->ltwh_expand[1] = this->ltwh[1];
        this->ltwh_expand[2] = this->ltwh[2];
        this->ltwh_expand[3] = this->ltwh[3];
        this->expand_static(this->ltwh_expand, expand_box_rate);
    }

    // ================================================================
    // 静态工具方法
    // ================================================================

    /***
     * @description: 原地扩展 ltwh 数组 (静态方法, 不依赖实例)
     *
     * 扩展逻辑 (与原始 ByteTrack 保持一致):
     *   1. add_w = int(w * rate)   -- 宽度扩展量 (int32 截断)
     *   2. add_h = int(h * rate)   -- 高度扩展量 (int32 截断)
     *   3. l = max(l - add_w/2, 0) -- 左边界向左偏移一半扩展量, 且不低于 0
     *   4. t = max(t - add_h/2, 0) -- 上边界向上偏移一半扩展量, 且不低于 0
     *   5. w = w + add_w           -- 宽度增加
     *   6. h = h + add_h           -- 高度增加
     *
     * 为什么用 int32 截断?
     *   与原版 bytetracker 保持数值一致, 确保结果精确可复现。
     *   截断导致的累积误差很小 (最多 1 像素), 对跟踪结果几乎没有影响。
     *
     * 为什么左上角要 clamp 到 0?
     *   防止扩展后的边界框超出图像左/上边缘;
     *   虽然会损失一点偏移信息, 但实际场景中目标极少紧贴图像边缘,
     *   这个影响可以忽略。
     *
     * @param ltwh float32* : ltwh 数组 [l, t, w, h] (原地修改);
     *                        输入是原始框, 输出是扩展后的框;
     * @param rate float32   : 扩展率, <= 0 时无操作;
     *                         例如 0.2 表示各方向扩大 20%;
     */
    static void expand_static(float32* ltwh, float32 rate = 0.0f)
    {
        // 扩展率为非正数时跳过, 无操作;
        // 这确保了当 expand_box_rate == 0.0 时,
        // ltwh_expand == ltwh, 完全一致;
        if (rate <= 0.0f)
        {
            return;
        }

        // 计算宽度和高度的扩展量 (使用 int32 截断, 与原版 bytetracker 一致);
        // 例如: w = 50, rate = 0.2 -> add_w = int(10.0) = 10
        //        h = 100, rate = 0.2 -> add_h = int(20.0) = 20
        int32 add_w = static_cast<int32>(ltwh[2] * rate);
        int32 add_h = static_cast<int32>(ltwh[3] * rate);

        // 左上角向外偏移一半的扩展量;
        // 例如: add_w = 10 -> 左边界向左偏移 5 像素
        //        add_h = 20 -> 上边界向上偏移 10 像素
        ltwh[0] -= static_cast<float32>(add_w) / 2.0f;
        ltwh[1] -= static_cast<float32>(add_h) / 2.0f;

        // 防止左上角坐标变为负数 (clamp 到 0);
        // 注意: 这个 clamp 会导致恢复时只能恢复到 0, 而非原始负值;
        // 实际场景中边界框极少靠近图像边缘, 影响可忽略;
        ltwh[0] = (ltwh[0] > 0.0f ? ltwh[0] : 0.0f);
        ltwh[1] = (ltwh[1] > 0.0f ? ltwh[1] : 0.0f);

        // 宽度和高度增加;
        ltwh[2] += static_cast<float32>(add_w);
        ltwh[3] += static_cast<float32>(add_h);
    }

    /***
     * @description: 将 ltwh 格式转换为 xyah 格式 (卡尔曼标准输入)
     *
     * 格式转换:
     *   ltwh [l, t, w, h] -> xyah [cx, cy, a, h]
     *   cx = l + w / 2      -- 中心点 x
     *   cy = t + h / 2      -- 中心点 y
     *   a  = w / h          -- 宽高比 (aspect ratio)
     *   h  = h              -- 高度 (保持不变)
     *
     * 为什么需要这个函数?
     *   卡尔曼滤波器使用 xyah 格式作为观测向量:
     *     mean[0] = cx (中心 x)
     *     mean[1] = cy (中心 y)
     *     mean[2] = a  (宽高比 = w/h)
     *     mean[3] = h  (高度)
     *   所以我们需要将 ltwh 格式转为 xyah 格式才能传给卡尔曼。
     *
     * @param ltwh_data const float32* : ltwh 数据指针, 长度至少 4;
     * @return std::array<float32, 4> : xyah [cx, cy, a, h];
     *                                   使用 std::array 返回, 无动态分配;
     */
    static std::array<float32, 4> ltwh_to_xyah(const float32* ltwh_data)
    {
        // 计算中心点 x, y;
        float32 cx = ltwh_data[0] + ltwh_data[2] * 0.5f;
        float32 cy = ltwh_data[1] + ltwh_data[3] * 0.5f;

        // 计算宽高比 (防止除零);
        float32 aspect = (ltwh_data[3] < 1e-5f) ? 0.0f : (ltwh_data[2] / ltwh_data[3]);

        // 返回 xyah [cx, cy, a, h];
        return {cx, cy, aspect, ltwh_data[3]};
    }

    /***
     * @description: 将 ltwh 格式转换为 xyah 格式 (vector 版本)
     *               功能同上, 为使用 std::vector 的调用者提供便利;
     * @param ltwh_vec const std::vector<float32>& : ltwh [l, t, w, h];
     * @return std::array<float32, 4> : xyah [cx, cy, a, h];
     */
    static std::array<float32, 4> ltwh_to_xyah(const std::vector<float32>& ltwh_vec)
    {
        return ltwh_to_xyah(ltwh_vec.data());
    }

    /***
     * @description: 将 ltwh 格式转换为 xyxy 格式
     *
     * 格式转换:
     *   ltwh [l, t, w, h] -> xyxy [x1, y1, x2, y2]
     *   x1 = l
     *   y1 = t
     *   x2 = l + w
     *   y2 = t + h
     *
     * @param ltwh_data const float32* : ltwh 数据指针, 长度至少 4;
     * @return std::array<float32, 4> : xyxy [x1, y1, x2, y2];
     */
    static std::array<float32, 4> ltwh_to_xyxy(const float32* ltwh_data)
    {
        return {
            ltwh_data[0],                 //
            ltwh_data[1],                 //
            ltwh_data[0] + ltwh_data[2],  //
            ltwh_data[1] + ltwh_data[3]   //
        };
    }

    /***
     * @description: 将 ltwh 格式转换为 xyxy 格式 (vector 版本)
     * @param ltwh_vec const std::vector<float32>& : ltwh [l, t, w, h];
     * @return std::array<float32, 4> : xyxy [x1, y1, x2, y2];
     */
    static std::array<float32, 4> ltwh_to_xyxy(const std::vector<float32>& ltwh_vec)
    {
        return ltwh_to_xyxy(ltwh_vec.data());
    }
};

}  // namespace tracker

#endif  // !__BOXOBJECT__H__