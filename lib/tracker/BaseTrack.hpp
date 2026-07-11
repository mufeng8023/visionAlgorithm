/***
 * @Author       : gxs
 * @Date         : 2026-06-22 10:36:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 14:48:00
 * @FilePath     : /visionAlgorithm/lib/tracker/BaseTrack.hpp
 * @Description  : 轨迹基类定义
 *
 *                 设计说明:
 *                 BaseTrack 是所有跟踪算法轨迹的基类, 包含所有跟踪器通用的字段和方法;
 *                 边界框信息 (ltwh / ltwh_expand / cls_id / score) 直接存储在基类中;
 *
 *                 边界框存储策略 (BoxObject 架构):
 *                 - ltwh[4] 始终保存**原始 (未扩展) **的检测框坐标
 *                   ltwh = left-top-width-height: [左, 上, 宽, 高]
 *                   由检测器直接输出, 跟踪过程中永不修改;
 *                   外部用户读取跟踪结果时, 应使用此原始框;
 *
 *                 - ltwh_expand[4] 保存**扩展后的**检测框坐标
 *                   基于 ltwh + _expand_box_rate 计算得到;
 *                   所有卡尔曼滤波器操作 (initiate / update / predict) 都使用此扩展框;
 *                   从卡尔曼 mean 写回时, 直接写入 ltwh_expand (无需 shrink 恢复);
 *
 *                 - get_x1 / get_xyah / get_xyxy 等全部基于 ltwh_expand 动态计算;
 *                   确保所有匹配/门控操作都在扩展空间中一致进行;
 *
 *                 核心数据流:
 *                 det ltwh -> expand -> ltwh_expand -> xyah -> Kalman -> mean
 *                 mean -> ltwh_expand (无收缩, 因为 ltwh 从未被覆盖)
 *
 *                 !对外接口:
 *                 外部直接通过轨迹对象的公开成员获取跟踪结果:
 *                   track->track_id, track->state, track->cls_id, track->score
 *                   track->ltwh (原始框), track->ltwh_expand (扩展框)
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __BASETRACK__H__
#define __BASETRACK__H__

#include <array>
#include <vector>

#include "tracker/BoxObject.hpp"
#include "tracker/KalmanFilter.hpp"
#include "types.hpp"

namespace tracker
{

/***
 * @description: 轨迹基类 (模板类)
 *               所有跟踪算法的轨迹继承此类;
 *
 * @tparam StateType 轨迹状态枚举类型
 *   SORT:       SortState      (Confirmed, Lost)
 *   DeepSORT:   DeepSortState  (Tentative, Confirmed, Deleted)
 *   ByteTrack:  ByteTrackState (New, Tracked, Lost, Removed)
 *   OC_SORT:    OCSortState    (New, Tracked, Occluded, Lost, Removed)
 */
template <typename StateType>
class BaseTrack
{
   public:
    // track_id: 轨迹唯一标识符, 在整个跟踪过程中保持不变, 用于区分不同目标
    int32 track_id = -1;

    // state: 轨迹的当前生命周期状态, 具体枚举由 StateType 在编译期决定
    StateType state = static_cast<StateType>(0);

    // frame_id: 当前帧 ID (每次 update 时由跟踪器更新)
    int32 frame_id = 0;

    // start_frame: 轨迹开始的帧 ID (创建时记录, 后续不变)
    int32 start_frame = 0;

    // track_len: 轨迹已持续的帧数 (每次 update 时增加)
    int32 track_len = 0;

    // ================================================================
    // 检测信息字段 (每帧匹配成功后更新)
    // cls_id: 当前匹配到的检测框的类别 ID (对应检测器的类别索引)
    int32 cls_id = -1;
    // score: 当前匹配到的检测框的置信度分数
    float32 score = 0.0f;

    // 边界框字段 -- 双框存储策略 (BoxObject 架构):
    //   ltwh[4]: 原始检测框 (left-top-width-height)
    //          保存检测器原始输出, 跟踪过程中永不修改;
    //          外部用户读取跟踪结果时, 应使用此原始框;
    //   ltwh_expand[4]: 扩展后的检测框
    //          基于 ltwh + _expand_box_rate 计算;
    //          所有卡尔曼操作都使用此扩展框;
    //          从卡尔曼 mean 写回时, 直接写入 ltwh_expand (无需 shrink);
    //
    // 为什么需要两个框?
    //   小目标 (如人脸) 在低帧率情况下可能匹配不上;
    //   通过扩展框给卡尔曼更宽松的搜索空间, 提高小目标跟踪鲁棒性;
    //   但输出的跟踪结果应使用原始检测框大小, 避免框偏大影响下游任务;
    //
    // 格式说明:
    //   ltwh[0] = l  (bounding box 左边 x1)
    //   ltwh[1] = t  (bounding box 上边 y1)
    //   ltwh[2] = w  (bounding box 宽度)
    //   ltwh[3] = h  (bounding box 高度)
    // ================================================================

    // ltwh: 原始检测框 (保存后永不修改);
    float32 ltwh[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // ltwh_expand: 扩展后的检测框 (用于卡尔曼操作);
    float32 ltwh_expand[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // ================================================================
    // 卡尔曼滤波器状态 (8 维)
    //   mean:       状态均值向量 [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
    //   covariance: 状态协方差矩阵 (8x8)
    //
    // !重要: mean 中的位置分量 (cx, cy, a, h) 存储的是**扩展空间**中的状态;
    //       !ltwh_expand 与 mean 在同一个"扩展空间"中工作;
    //       !从 mean 转换到 ltwh_expand 时, 直接写入, 无需再次扩展或收缩;
    // ================================================================
    // mean: 卡尔曼滤波器状态均值 (8 维向量)
    //   [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
    //   cx, cy : 目标框中心坐标 (像素) -- 注意: 扩展空间中的坐标
    //   a      : 宽高比 (width / height)
    //   h      : 目标框高度 (像素)
    //   v_*    : 对应的速度分量 (像素/帧)
    KAL_MEAN mean;

    // covariance: 卡尔曼滤波器状态协方差 (8x8 矩阵)
    // 对角线的值越大, 说明对应分量的不确定度越大
    KAL_COVA covariance;

   protected:
    // ================================================================
    // 边界框扩展率 (_expand_box_rate)
    //
    // 背景: 为什么需要边界框扩展?
    //       小目标 (如远处的人, 几十像素大小) 在检测器中边界框定位精度有限;
    //       卡尔曼滤波器如果基于这些不精确的小框初始化, 估计会不稳定;
    //       通过将边界框向外扩展一定比例, 可以给卡尔曼滤波器更宽松的搜索空间,
    //       提高小目标跟踪的鲁棒性;
    //
    // 取值说明:
    //   0.0  : 不扩展 (默认)
    //   0.1  : 各方向扩展 10% (例如宽高各增加 10%)
    //   0.2  : 各方向扩展 20%
    //
    // 注意: 此值通过构造函数从配置传入;
    //       扩展在 BoxObject 创建时执行, 结果存储在 ltwh_expand 中;
    //       跟踪过程中全程使用 ltwh_expand, 不再需要 expand 或 shrink 操作;
    // ================================================================
    float32 _expand_box_rate = 0.0f;

    // _is_activated: 是否已完成卡尔曼初始化
    // ByteTrack: New 状态的轨迹未激活, activate() 后激活
    // DeepSORT:  创建时直接传入卡尔曼状态, 创建即激活
    bool _is_activated = false;

   public:
    /***
     * @description: 构造函数, 初始化轨迹的基本信息
     * @param track_id         int32  : 轨迹的唯一 ID (由跟踪器分配, 从 1 开始)
     * @param frame_id         int32  : 当前帧 ID (轨迹开始时的帧号)
     * @param expand_box_rate  float32 : 边界框扩展率, 从 ini 配置传入, 默认 0.0
     */
    BaseTrack(int32 track_id, int32 frame_id, float32 expand_box_rate = 0.0f)
        : track_id(track_id),     // 使用初始化列表
          frame_id(frame_id),     //
          start_frame(frame_id),  //
          _expand_box_rate(expand_box_rate)
    {
        // 构造函数体为空, 所有初始化在初始化列表中完成;
    }

    /***
     * @description: 虚析构函数, 确保通过基类指针删除子类对象时正确释放资源
     */
    virtual ~BaseTrack() = default;

    // ================================================================
    // 纯虚函数: 状态标记 (子类必须实现); 可能存在冗余, 子类可以有空实现
    // ================================================================

    /***
     * @description: 标记轨迹为丢失状态
     *   ByteTrack: state -> Lost (暂时丢失, 等待重连)
     *   DeepSORT:  调用 mark_missed() (Tentative 直接删除, Confirmed 检查超时)
     */
    virtual void mark_lost() = 0;

    /***
     * @description: 标记轨迹为已移除状态
     *   ByteTrack: state -> Removed
     *   DeepSORT:  state -> Deleted
     */
    virtual void mark_removed() = 0;

    // ================================================================
    // 状态查询
    // ================================================================

    /***
     * @description: 获取轨迹最后更新的帧 ID
     * @return int32
     */
    inline int32 end_frame() const { return this->frame_id; }

    /***
     * @description: 判断轨迹是否已被激活 (即是否已完成卡尔曼滤波器初始化)
     * @return bool
     */
    inline bool is_activated() const { return this->_is_activated; }

    /***
     * @description: 获取边界框扩展率
     * @return float32 : 当前扩展率
     */
    inline float32 get_expand_box_rate() const { return this->_expand_box_rate; }

    // ================================================================
    // 边界框动态计算方法 (全部基于 ltwh_expand[4] 计算)
    //
    // 设计原则:
    //   所有 getter 方法都使用 ltwh_expand 进行计算;
    //   因为卡尔曼滤波器在"扩展空间"中工作, 所有匹配/门控操作
    //   (IoU 计算, 马氏距离, 余弦距离) 都应在扩展空间中一致进行;
    //   外部如果需要原始框, 直接读取 track->ltwh[0..3];
    // ================================================================

    /***
     * @description: 获取 bounding box 左边 x1 (基于扩展框 ltwh_expand[0])
     *               注意: 返回的是"扩展空间"中的坐标;
     *               如果需要原始坐标, 请直接读取 track->ltwh[0];
     * @return float32
     */
    inline float32 get_x1() const { return this->ltwh_expand[0]; }

    /***
     * @description: 获取 bounding box 上边 y1 (基于扩展框 ltwh_expand[1])
     * @return float32
     */
    inline float32 get_y1() const { return this->ltwh_expand[1]; }

    /***
     * @description: 获取 bounding box 右边 x2 (基于扩展框)
     *               由 ltwh_expand[0] + ltwh_expand[2] 动态计算;
     * @return float32
     */
    inline float32 get_x2() const { return this->ltwh_expand[0] + this->ltwh_expand[2]; }

    /***
     * @description: 获取 bounding box 下边 y2 (基于扩展框)
     *               由 ltwh_expand[1] + ltwh_expand[3] 动态计算;
     * @return float32
     */
    inline float32 get_y2() const { return this->ltwh_expand[1] + this->ltwh_expand[3]; }

    /***
     * @description: 获取边界框宽度 (扩展框 ltwh_expand[2])
     * @return float32
     */
    inline float32 get_width() const { return this->ltwh_expand[2]; }

    /***
     * @description: 获取边界框高度 (扩展框 ltwh_expand[3])
     * @return float32
     */
    inline float32 get_height() const { return this->ltwh_expand[3]; }

    /***
     * @description: 获取宽高比 (aspect ratio = width / height)
     *               基于扩展框计算;
     *               卡尔曼滤波器使用此值作为状态向量的第 3 维;
     * @return float32 : 宽高比, 如果高度为 0 则返回 0
     */
    inline float32 get_aspect_ratio() const
    {
        // 防止除零;
        if (this->ltwh_expand[3] < 1e-5f)
        {
            return 0.0f;
        }
        return this->ltwh_expand[2] / this->ltwh_expand[3];
    }

    /***
     * @description: 获取边界框 中心 x 坐标 (基于扩展框)
     *               用于卡尔曼滤波器的状态向量;
     *               由 ltwh_expand[0] + ltwh_expand[2] / 2 动态计算;
     * @return float32 : 中心点 x
     */
    inline float32 get_cx() const { return this->ltwh_expand[0] + this->ltwh_expand[2] * 0.5f; }

    /***
     * @description: 获取边界框 中心 y 坐标 (基于扩展框)
     *               用于卡尔曼滤波器的状态向量;
     *               由 ltwh_expand[1] + ltwh_expand[3] / 2 动态计算;
     * @return float32 : 中心点 y
     */
    inline float32 get_cy() const { return this->ltwh_expand[1] + this->ltwh_expand[3] * 0.5f; }

    /***
     * @description: 一次性获取 x1y1x2y2 格式的完整数组 (用于 IoU 计算和可视化)
     *               基于扩展框计算;
     *               通过返回 std::array 实现值语义, 无额外开销 (RVO);
     * @return std::array<float32, 4> : {x1, y1, x2, y2}
     */
    inline std::array<float32, 4> get_xyxy() const
    {
        return {
            this->ltwh_expand[0],                         //
            this->ltwh_expand[1],                         //
            this->ltwh_expand[0] + this->ltwh_expand[2],  //
            this->ltwh_expand[1] + this->ltwh_expand[3]   //
        };
    }

    /***
     * @description: 一次性获取 xyah 格式的完整数组 (卡尔曼滤波器标准输入)
     *               基于扩展框计算;
     *               返回 {cx, cy, a, h}, 等价于 KAL_HMEAN;
     *               通过返回 std::array 实现值语义, 无额外开销 (RVO);
     *
     *               注意: 这里返回的是**扩展空间**中的 xyah;
     *               卡尔曼滤波器的 mean 和 ltwh_expand 在同一个扩展空间中,
     *               因此可以直接用此返回值进行卡尔曼的 initiate 或 update;
     *
     * @return std::array<float32, 4> : {cx, cy, a, h}
     *   cx = 中心点 x (扩展空间)
     *   cy = 中心点 y (扩展空间)
     *   a  = 宽高比 (width / height, 扩展空间)
     *   h  = 高度 (扩展空间)
     */
    inline std::array<float32, 4> get_xyah() const
    {
        float32 cx = this->ltwh_expand[0] + this->ltwh_expand[2] * 0.5f;
        float32 cy = this->ltwh_expand[1] + this->ltwh_expand[3] * 0.5f;
        float32 a = (this->ltwh_expand[3] < 1e-5f) ? 0.0f : (this->ltwh_expand[2] / this->ltwh_expand[3]);
        return {cx, cy, a, this->ltwh_expand[3]};
    }

   protected:
    // ================================================================
    // 从卡尔曼 mean 更新 ltwh_expand
    //
    // 设计原则:
    //   mean 存储的是"扩展空间"中的状态 (cx, cy, a, h);
    //   ltwh_expand 也是"扩展空间"中的框;
    //   因此 mean -> ltwh_expand 是直接映射, 无需 expand 或 shrink;
    //   而 ltwh (原始框) 在此过程中保持不变;
    // ================================================================

    /***
     * @description: 从卡尔曼 mean 更新 this->ltwh_expand (扩展框)
     *
     *               这是"卡尔曼更新 -> 写回扩展框"链条的最后一步;
     *               mean 和 ltwh_expand 在同一个扩展空间中, 因此直接写入, 无需转换;
     *
     *               转换流程:
     *               1. mean[0..3] = [cx, cy, a, h]  (扩展空间中的 xyah)
     *               2. w = a * h                    (扩展空间中的宽度)
     *               3. ltwh_expand = [cx - w/2, cy - h/2, w, h]
     *               4. 写入 this->ltwh_expand        (扩展空间, 不收缩)
     *
     *               注意: this->ltwh (原始框) 在此过程中**不会被修改**;
     *               这是和旧版本最大的区别;
     *               旧版本需要 shrink 恢复, 新版本不再需要;
     *
     *               子类如有特殊逻辑 (如 ByteTrack 的 New 状态检查),
     *               可以在自己的 update_ltwh() 覆盖方法中先判断再调用此方法;
     *
     *               示例 (ByteTrack):
     *                 void update_ltwh() override {
     *                     if (this->state == ByteTrackState::New) return;
     *                     BaseTrack::update_ltwh_from_mean();
     *                 }
     */
    void update_ltwh_from_mean()
    {
        // 从卡尔曼 mean 提取位置分量 (扩展空间);
        // mean[0] = cx: 中心点 x (扩展空间)
        // mean[1] = cy: 中心点 y (扩展空间)
        // mean[2] = a:  宽高比 (扩展空间: w_exp / h_exp)
        // mean[3] = h:  高度 (扩展空间)
        float32 cx = this->mean(0);
        float32 cy = this->mean(1);
        float32 a = this->mean(2);
        float32 h = this->mean(3);

        // 从宽高比和高度计算宽度;
        // 注意: 这里的 w 和 h 都是"扩展空间"的尺寸;
        float32 w = a * h;

        // xyah -> ltwh 转换:
        //   l = cx - w / 2  (左边 = 中心点 - 宽度/2)
        //   t = cy - h / 2  (上边 = 中心点 - 高度/2)
        //   w = w            (宽度)
        //   h = h            (高度)
        //
        // 结果写入 this->ltwh_expand (扩展空间);
        // 不需要 shrink, 因为 ltwh_expand 就是扩展空间的框;
        this->ltwh_expand[0] = cx - w / 2.0f;
        this->ltwh_expand[1] = cy - h / 2.0f;
        this->ltwh_expand[2] = w;
        this->ltwh_expand[3] = h;

        // this->ltwh (原始框) 在此过程中保持不变;
        // this->ltwh (原始框) 只有在 轨迹 update 新的目标检测框 的时候更新;
    }
};

}  // namespace tracker

#endif  // !__BASETRACK__H__