/***
 * @Author       : gxs
 * @Date         : 2026-06-22 10:58:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 14:50:00
 * @FilePath     : /visionAlgorithm/lib/tracker/bytetrack/BytetrackTrack.hpp
 * @Description  : ByteTrack 算法的轨迹类实现
 *
 *                 边界框存储策略 (BoxObject 架构):
 *                 - ltwh[4]: 原始检测框 (保存后永不修改)
 *                 - ltwh_expand[4]: 扩展后的检测框 (用于所有卡尔曼操作)
 *                 - 所有 getter 方法基于 ltwh_expand 计算
 *
 *                 与原版 bytetracker 的关键差异:
 *                 - 原版 STrack 同时存储 raw_tlwh / _tlwh(扩展) / tlwh(输出) 三种框
 *                 - 本实现使用 BoxObject 架构: ltwh (原始) + ltwh_expand (扩展)
 *                 - 原版使用 tlwh_to_xyah 转换, 本实现在基类 get_xyah() 中统一完成
 *                 - 原版在 re_activate() 中使用未扩展框, 本实现统一使用 ltwh_expand
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __BYTETRACKTRACK__H__
#define __BYTETRACKTRACK__H__

#include <vector>

#include "tracker/BaseTrack.hpp"
#include "tracker/BoxObject.hpp"
#include "tracker/KalmanFilter.hpp"
#include "tracker/TrackState.hpp"

namespace tracker
{

/***
 * @description: ByteTrack 算法的轨迹类
 *               参考 bytetracker 的 STrack 实现;
 *
 *               边界框管理 (BoxObject 架构):
 *               - this->ltwh: 原始检测框 (raw_ltwh, 不扩展, 永不修改)
 *               - this->ltwh_expand: 扩展后的检测框 (用于所有卡尔曼操作)
 *               - 基类提供 get_xyah() / get_xyxy() 等, 全部基于 ltwh_expand 计算
 *               - cls_id / score 直接存储在基类, 外部直接通过 track->cls_id 读取
 *
 * @note 与 DeepSORT Track 的区别:
 *       - ByteTrack 持有 KalmanFilter 副本, DeepSORT 用指针引用
 *       - ByteTrack 使用 BoxObject 管理扩展框, DeepSORT 直接接收 mean 和 BoxObject
 *       - ByteTrack 没有 hits/age/time_since_update, DeepSORT 有
 *       - ByteTrack 状态流是 New->Tracked->Lost->Removed
 *       - DeepSORT 状态流是 Tentative->Confirmed->Deleted
 */
class BytetrackTrack : public BaseTrack<ByteTrackState>
{
   private:
    // _kalman_filter: 卡尔曼滤波器对象 (每个轨迹持有自己的副本)
    // 参考 bytetracker STrack::kalman_filter; 与 DeepSORT 共享全局 KF 不同;
    KalmanFilter _kalman_filter;

   public:
    /***
     * @description: 有参构造函数 (从 std::vector<float32> 创建)
     *
     *               参考 bytetracker STrack(vector<float> tlwh_, float score, int label)
     *               创建时保存原始框到 this->ltwh, 同时计算扩展框到 this->ltwh_expand;
     *               卡尔曼初始化在 activate() 中完成;
     *
     * @param ltwh             const std::vector<float32>& : 检测框 (ltwh 格式, 原始坐标)
     * @param frame_id         int32 : 创建时的帧 ID
     * @param track_id         int32 : 轨迹 ID (由跟踪器预分配, 通常为 -1, activate() 时正式分配)
     * @param cls_id           uint32 : 类别 ID, 从配置传入, 默认 -1
     * @param score            float32 : 检测框得分, 从配置传入, 默认 0.0
     * @param expand_box_rate  float32 : 边界框扩展率, 从配置传入, 默认 0.0
     */
    BytetrackTrack(const std::vector<float32>& ltwh,  //
                   int32 frame_id,                    //
                   int32 track_id,                    //
                   uint32 cls_id = -1,                //
                   float32 score = 0.0f,              //
                   float32 expand_box_rate = 0.0f)
        : BaseTrack(track_id, frame_id, expand_box_rate)
    {
        // ---- 保存原始检测框 (ltwh) ----
        // this->ltwh 始终保持检测器的原始输出, 永不修改;
        // 参考 bytetracker STrack 构造函数中对 raw_tlwh 的初始化;
        this->ltwh[0] = ltwh[0];
        this->ltwh[1] = ltwh[1];
        this->ltwh[2] = ltwh[2];
        this->ltwh[3] = ltwh[3];

        // ---- 初始化扩展框 (ltwh_expand) ----
        // 基于原始框 + _expand_box_rate 计算;
        // 使用 BoxObject 的静态工具方法进行扩展, 确保与原版 ByteTrack 行为一致;
        this->ltwh_expand[0] = ltwh[0];
        this->ltwh_expand[1] = ltwh[1];
        this->ltwh_expand[2] = ltwh[2];
        this->ltwh_expand[3] = ltwh[3];
        BoxObject::expand_static(this->ltwh_expand, this->_expand_box_rate);

        //  score 和 cls_id;
        this->score = score;
        this->cls_id = cls_id;

        // ---- 初始化状态 ----
        this->state = ByteTrackState::New;
        this->track_len = 0;
    }

    /***
     * @description: 有参构造函数 (从 BoxObject 创建)
     *
     *               当外部已经创建好 BoxObject 时, 直接使用其 ltwh / ltwh_expand / score / cls_id;
     *               避免了重复扩展计算, 同时初始化检测信息;
     *
     * @param box              const BoxObject& : 检测框信息 (包含 ltwh, ltwh_expand, score, cls_id)
     * @param frame_id         int32 : 创建时的帧 ID
     * @param track_id         int32 : 轨迹 ID (由跟踪器预分配)
     * @param expand_box_rate  float32 : 边界框扩展率, 从配置传入, 默认 0.0
     */
    BytetrackTrack(const BoxObject& box,  //
                   int32 frame_id,        //
                   int32 track_id,        //
                   float32 expand_box_rate = 0.0f)
        : BaseTrack(track_id, frame_id, expand_box_rate)
    {
        // 使用 BoxObject 中的原始框和扩展框;
        this->ltwh[0] = box.ltwh[0];
        this->ltwh[1] = box.ltwh[1];
        this->ltwh[2] = box.ltwh[2];
        this->ltwh[3] = box.ltwh[3];

        this->ltwh_expand[0] = box.ltwh_expand[0];
        this->ltwh_expand[1] = box.ltwh_expand[1];
        this->ltwh_expand[2] = box.ltwh_expand[2];
        this->ltwh_expand[3] = box.ltwh_expand[3];

        // 从 BoxObject 中读取 score 和 cls_id;
        // 与 ByteTrack 原版 STrack(label, score) 构造函数行为一致;
        this->score = box.score;
        this->cls_id = box.cls_id;

        this->state = ByteTrackState::New;
        this->track_len = 0;
    }

    /***
     * @description: 默认构造函数 (用于容器, 如 std::vector 扩容)
     */
    BytetrackTrack() : BaseTrack(-1, 0, 0.0f) {}

    /***
     * @description: 析构函数
     */
    virtual ~BytetrackTrack() = default;

    // ================================================================
    // 边界框动态计算方法
    // ================================================================

    /***
     * @description: 获取 xyxy 格式 [x1, y1, x2, y2], 用于 IoU 计算
     *               参考 bytetracker STrack::xyxy
     *
     *               NOTE: IoU 匹配使用扩展框的 xyxy, 因为扩展框与卡尔曼状态
     *               处于同一空间, 匹配一致性更好;
     *               如果需要原始框的 xyxy, 可直接读取 track->ltwh 自行计算;
     *
     * @return std::array<float32, 4> : xyxy 格式 [x1, y1, x2, y2];
     *                                   使用 std::array 返回, 无动态分配;
     */
    std::array<float32, 4> get_xyxy() const
    {
        return {
            this->ltwh_expand[0],                         //
            this->ltwh_expand[1],                         //
            this->ltwh_expand[0] + this->ltwh_expand[2],  //
            this->ltwh_expand[1] + this->ltwh_expand[3]   //
        };
    }

    /***
     * @description: 从卡尔曼 mean 更新 this->ltwh_expand (扩展框)
     *
     *               mean 存储的是扩展空间中的 xyah 状态;
     *               通过基类的 update_ltwh_from_mean() 转换为 ltwh_expand;
     *
     *               New 状态保持原始框不变 (因为 mean 尚未赋予实际值);
     *               参考 bytetracker STrack::static_tlwh()
     */
    void update_ltwh()
    {
        // New 状态的轨迹尚未完成卡尔曼初始化, mean 中无有效数据;
        // 保持 ltwh_expand 不变 (仍为创建时设置的扩展框);
        // 参考 bytetracker STrack::static_tlwh() 中 state == New 的处理;
        if (this->state == ByteTrackState::New)
        {
            return;
        }

        // 从 mean 转换 ltwh_expand (扩展空间, 不收缩);
        // 基类的 update_ltwh_from_mean() 内部执行:
        //   1. mean[0..3] = [cx, cy, a, h]  (扩展空间)
        //   2. w = a * h
        //   3. ltwh_expand = [cx - w/2, cy - h/2, w, h]
        //   4. 写入 this->ltwh_expand (不修改 this->ltwh)
        this->update_ltwh_from_mean();
    }

    // ================================================================
    // 核心方法 (参考 bytetracker STrack)
    // ================================================================

    /***
     * @description: 激活轨迹, 使用卡尔曼滤波器初始化状态
     *               首次将 New 状态的轨迹转为 Tracked;
     *
     *               卡尔曼初始化流程:
     *               1. 从 ltwh_expand (扩展框) 获取 xyah
     *               2. 用扩展后的 xyah 初始化卡尔曼滤波器
     *               3. 从 mean 更新 ltwh_expand
     *               4. 更新状态为 Tracked
     *
     *               参考 bytetracker STrack::activate()
     * @param kalman_filter KalmanFilter& : 卡尔曼滤波器对象 (拷贝到本地副本)
     * @param frame_id      int32 : 当前帧 ID
     */
    void activate(KalmanFilter& kalman_filter, int32 frame_id)
    {
        // 拷贝卡尔曼滤波器 (每个轨迹持有独立副本, 与 DeepSORT 共享全局 KF 不同);
        this->_kalman_filter = kalman_filter;

        // 分配新的全局唯一 track_id;
        this->track_id = this->next_id();
        this->frame_id = frame_id;
        this->start_frame = frame_id;

        // 从 ltwh_expand (扩展框) 获取 xyah, 用于卡尔曼初始化;
        // 基类的 get_xyah() 基于 ltwh_expand 计算, 返回 [cx, cy, a, h];
        // 扩展框给卡尔曼更宽松的搜索空间, 提高小目标跟踪鲁棒性;
        std::array<float32, 4> xyah_arr = this->get_xyah();
        KAL_HMEAN xyah;
        xyah << xyah_arr[0], xyah_arr[1], xyah_arr[2], xyah_arr[3];

        // 卡尔曼初始化: 创建初始状态 [cx, cy, a, h, 0, 0, 0, 0];
        // mean 中存储的是扩展空间中的状态 (与 ltwh_expand 一致);
        KAL_DATA init_data = this->_kalman_filter.initiate(xyah);
        this->mean = init_data.first;
        this->covariance = init_data.second;

        // 从 mean 更新 ltwh_expand (扩展空间, 不收缩);
        this->update_ltwh();

        // 更新轨迹元信息;
        this->track_len = 0;
        this->state = ByteTrackState::Tracked;
        this->_is_activated = true;
    }

    /***
     * @description: 重新激活轨迹 (丢失后再次匹配时调用)
     *               与 activate() 不同: 不是初始化新的卡尔曼状态, 而是用新检测框更新现有卡尔曼;
     *
     *               重新激活流程:
     *               1. 从检测框的 ltwh 更新 this->ltwh (原始框)
     *               2. 从检测框的 ltwh_expand 更新 this->ltwh_expand (扩展框)
     *               3. 基于 ltwh_expand 计算 xyah, 进行卡尔曼更新
     *               4. 从 mean 更新 ltwh_expand
     *
     *               参考 bytetracker STrack::re_activate()
     * @param det_box const BoxObject& : 当前帧匹配到的检测框信息
     *                包含 ltwh (原始框) 和 ltwh_expand (扩展框);
     * @param frame_id  int32 : 当前帧 ID
     * @param new_id    bool   : 是否分配新的轨迹 ID (默认 false)
     */
    void re_activate(const BoxObject& det_box, int32 frame_id, bool new_id = false)
    {
        // ---- 第 1 步: 更新原始框 ----
        // 保持检测器原始输出, 确保外部读取 track->ltwh 时是原始坐标;
        this->ltwh[0] = det_box.ltwh[0];
        this->ltwh[1] = det_box.ltwh[1];
        this->ltwh[2] = det_box.ltwh[2];
        this->ltwh[3] = det_box.ltwh[3];

        // ---- 第 2 步: 更新扩展框 ----
        // 使用检测框的扩展版本进行卡尔曼更新;
        this->ltwh_expand[0] = det_box.ltwh_expand[0];
        this->ltwh_expand[1] = det_box.ltwh_expand[1];
        this->ltwh_expand[2] = det_box.ltwh_expand[2];
        this->ltwh_expand[3] = det_box.ltwh_expand[3];

        // ---- 第 3 步: 卡尔曼更新 ----
        // 基于 ltwh_expand 计算 xyah, 用于修正卡尔曼状态;
        std::array<float32, 4> xyah_arr = this->get_xyah();
        KAL_HMEAN xyah;
        xyah << xyah_arr[0], xyah_arr[1], xyah_arr[2], xyah_arr[3];

        // 卡尔曼更新: 用扩展后的检测框修正预测状态;
        KAL_DATA update_data = this->_kalman_filter.update(this->mean, this->covariance, xyah);
        this->mean = update_data.first;
        this->covariance = update_data.second;

        // ---- 第 4 步: 从 mean 写回 ltwh_expand ----
        this->update_ltwh();

        // 同步检测信息;
        // 从 BoxObject 中读取 score 和 cls_id, 与 re_activate 传入的检测框一致;
        this->score = det_box.score;
        this->cls_id = det_box.cls_id;

        this->track_len = 0;
        this->state = ByteTrackState::Tracked;
        this->_is_activated = true;
        this->frame_id = frame_id;

        if (new_id)
        {
            this->track_id = this->next_id();
        }
    }

    /***
     * @description: 更新轨迹 (每帧匹配成功后调用)
     *               连续跟踪中的轨迹, 使用当前帧匹配到的检测框修正卡尔曼;
     *
     *               更新流程 (参考 bytetracker STrack::update() -- STrack.cpp:168):
     *               1. 从检测框的 ltwh 更新 this->ltwh (原始框, 保持检测器原始输出)
     *               2. 从检测框的 ltwh_expand 更新 this->ltwh_expand (扩展框, 用于卡尔曼)
     *               3. 基于 ltwh_expand 计算 xyah, 进行卡尔曼更新
     *               4. 从 mean 写回 ltwh_expand
     *
     *               NOTE: 原始 ByteTrack 在 update() 中使用的是扩展后的框
     *               (new_track.tlwh 实际上是 _tlwh 扩展后的结果);
     *               本实现明确使用 det_box.ltwh_expand, 语义更清晰;
     *
     * @param det_box  const BoxObject& : 当前帧匹配到的检测框信息
     *                 包含 ltwh (原始框) 和 ltwh_expand (扩展框);
     * @param frame_id int32 : 当前帧 ID
     */
    void update(const BoxObject& det_box, int32 frame_id)
    {
        this->frame_id = frame_id;
        this->track_len++;

        // ---- 第 1 步: 更新原始框 (保持检测器原始输出) ----
        // 参考 bytetracker STrack::update() 中 raw_tlwh 的赋值;
        this->ltwh[0] = det_box.ltwh[0];
        this->ltwh[1] = det_box.ltwh[1];
        this->ltwh[2] = det_box.ltwh[2];
        this->ltwh[3] = det_box.ltwh[3];

        // ---- 第 2 步: 更新扩展框 (用于卡尔曼操作) ----
        // 参考 bytetracker STrack::update() 中 _tlwh 的赋值;
        this->ltwh_expand[0] = det_box.ltwh_expand[0];
        this->ltwh_expand[1] = det_box.ltwh_expand[1];
        this->ltwh_expand[2] = det_box.ltwh_expand[2];
        this->ltwh_expand[3] = det_box.ltwh_expand[3];

        // 同步检测信息;
        // 从 BoxObject 中读取 score 和 cls_id, 与 update 传入的检测框一致;
        this->score = det_box.score;
        this->cls_id = det_box.cls_id;

        // ---- 第 3 步: 从扩展框转 xyah 进行卡尔曼更新 ----
        // 基类的 get_xyah() 基于 ltwh_expand 计算, 返回 [cx, cy, a, h];
        std::array<float32, 4> xyah_arr = this->get_xyah();
        KAL_HMEAN xyah;
        xyah << xyah_arr[0], xyah_arr[1], xyah_arr[2], xyah_arr[3];

        // 卡尔曼更新: 用扩展后的检测框修正预测状态;
        // 参考 bytetracker STrack::update() 中 kalman_filter.update 的调用;
        KAL_DATA update_data = this->_kalman_filter.update(this->mean, this->covariance, xyah);
        this->mean = update_data.first;
        this->covariance = update_data.second;

        // ---- 第 4 步: 从 mean 写回 ltwh_expand ----
        // mean 经过卡尔曼修正后, 是"后验估计"的扩展空间状态;
        // 直接写入 ltwh_expand, 无需收缩;
        this->update_ltwh();

        this->state = ByteTrackState::Tracked;
        this->_is_activated = true;
    }

    // ================================================================
    // 状态标记 (继承自 BaseTrack)
    // ================================================================

    /***
     * @description: 标记轨迹为丢失状态
     *               参考 bytetracker STrack::mark_lost()
     */
    virtual void mark_lost() override { this->state = ByteTrackState::Lost; }

    /***
     * @description: 标记轨迹为已移除状态
     *               参考 bytetracker STrack::mark_removed()
     */
    virtual void mark_removed() override { this->state = ByteTrackState::Removed; }

    // ================================================================
    // 静态工具方法
    // ================================================================

    /***
     * @description: 批量预测所有轨迹
     *               每帧在所有检测结果出来之前调用一次;
     *               对非 Tracked 状态的轨迹将速度分量 v_h 清零 (防止时间间隔过长导致速度异常);
     *               参考 bytetracker STrack::multi_predict()
     * @param stracks       std::vector<BytetrackTrack*>& : 所有活跃轨迹的指针列表
     * @param kalman_filter KalmanFilter& : 卡尔曼滤波器对象 (所有轨迹共用)
     */
    static void multi_predict(std::vector<BytetrackTrack*>& stracks, KalmanFilter& kalman_filter)
    {
        for (size_t i = 0; i < stracks.size(); i++)
        {
            // 对于非 Tracked 状态 (New/Lost) 的轨迹, 将高度变化率清零;
            // 原因: Lost 轨迹可能已经丢失多帧, 速度信息不可靠;
            //       清零速度可以让预测位置停留在上一帧位置, 等待重连;
            if (stracks[i]->state != ByteTrackState::Tracked)
            {
                stracks[i]->mean(7) = 0.0f;
            }
            // 使用共享的卡尔曼滤波器进行预测 (公式: mean' = F*mean, cov' = F*cov*F^T + Q);
            kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
        }
    }

    /***
     * @description: 获取下一个全局唯一的轨迹 ID
     *               ID 从 1 开始递增, 每次分配自动 +1;
     *               传入负数时递增, 传入正数时重置计数器;
     *               参考 bytetracker STrack::next_id()
     * @param id int32 : 负数 = 自动递增; 正数 = 设置当前计数为此值
     * @return int32 : 新分配的轨迹 ID
     */
    static int32 next_id(int32 id = -1)
    {
        // 静态计数器, 程序运行期间持续递增;
        static int32 _count = 0;
        if (id < 0)
        {
            _count++;
        }
        else
        {
            _count = id;
        }
        return _count;
    }

    /***
     * @description: 将 ltwh 转为 xyah 格式 (卡尔曼标准输入)
     *               ltwh = [l, t, w, h] -> xyah = [cx, cy, a, h]
     *               cx = l + w/2,  cy = t + h/2,  a = w/h;
     *
     *               参考 bytetracker STrack::tlwh_to_xyah()
     *               此方法已移至 BoxObject::ltwh_to_xyah(), 此处保留为兼容旧调用;
     *
     * @param ltwh_data const float32* : ltwh 数据指针, 长度至少 4
     * @return std::array<float32, 4> : xyah [cx, cy, a, h];
     *                                   使用 std::array 返回, 无动态分配;
     */
    static std::array<float32, 4> ltwh_to_xyah(const float32* ltwh_data)
    {
        // 委托给 BoxObject 的静态方法;
        return BoxObject::ltwh_to_xyah(ltwh_data);
    }

    /***
     * @description: 将 ltwh 转为 xyah 格式 (std::array 重载)
     *               参考 bytetracker STrack::tlwh_to_xyah()
     * @param ltwh_arr const std::array<float32, 4>& : ltwh [l, t, w, h]
     * @return std::array<float32, 4> : xyah [cx, cy, a, h]
     */
    static std::array<float32, 4> ltwh_to_xyah(const std::array<float32, 4>& ltwh_arr)
    {
        return BoxObject::ltwh_to_xyah(ltwh_arr.data());
    }

    /***
     * @description: 将 xyxy 转为 ltwh (原地修改)
     *               xyxy = [x1, y1, x2, y2] -> ltwh = [x1, y1, x2-x1, y2-y1];
     *               参考 bytetracker STrack::xyxy_to_tlwh()
     * @param xyxy std::vector<float32>& : xyxy 格式 (原地修改为 ltwh 格式)
     */
    static void xyxy_to_ltwh(std::vector<float32>& xyxy)
    {
        xyxy[2] -= xyxy[0];
        xyxy[3] -= xyxy[1];
    }
};

}  // namespace tracker

#endif  // !__BYTETRACKTRACK__H__