/***
 * @Author       : gxs
 * @Date         : 2026-06-22 21:38:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 21:38:00
 * @FilePath     : /visionAlgorithm/lib/tracker/deepsort/matching.hpp
 * @Description  : DeepSORT 匹配算法实现;
 *                 包括余弦距离, 马氏距离门控, 级联匹配和 IoU 匹配;
 *
 *                 参考 deepsort/linear_assignment.py 中的实现;
 *
 * =====================================================================
 * 匹配流程概览
 * =====================================================================
 *
 *  DeepSORT 的数据关联分两步:
 *
 *  Step 1 - 级联匹配 (Cascade Matching):
 *    优先匹配"最近更新"的轨迹 (time_since_update 小的优先);
 *    距离度量 = (1 - lambda) * 马氏距离 + lambda * 余弦距离;
 *    门控: 马氏距离超过卡方阈值(9.4877)的匹配被拒绝;
 *          余弦距离超过 max_cosine_distance 的匹配被拒绝;
 *
 *  Step 2 - IoU 匹配 (IoU Matching):
 *    级联匹配中未匹配的轨迹 x 未匹配的检测;
 *    仅对 time_since_update == 1 的轨迹计算 IoU;
 *    距离度量 = 1 - IoU (使用扩展框);
 *
 * 为什么用级联匹配而不是一次性关联?
 *   如果轨迹 A 刚更新过 (time_since_update=1), 卡尔曼预测很准;
 *   如果轨迹 B 已经丢失了 10 帧 (time_since_update=10), 预测很不准;
 *   级联匹配让 A 先挑检测框, B 只能挑 A 不要的框;
 *   这样避免了 B 的"大光环"覆盖掉 A 的精确匹配;
 * =====================================================================
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __DEEPSORT_MATCHING__H__
#define __DEEPSORT_MATCHING__H__

#include <cmath>   // std::sqrt
#include <vector> 

#include "tracker/BoxKalmanFilter.hpp"
#include "tracker/deepsort/DeepSortTrack.hpp"
#include "tracker/deepsort/hungarian.hpp"

namespace tracker
{
namespace deepsort
{

// 无穷大成本 (用于门控过滤);
// 当马氏距离超过卡方阈值时, 将成本矩阵对应位置设为此值, 阻止匹配;
constexpr float32 INFTY_COST_DEEP = 1e5f;

// vector2D 模板别名: 简化二维向量的类型声明;
template <typename T>
using vector2D = std::vector<std::vector<T>>;

/***
 * @description: 匹配结果结构;
 *               存储一次匹配操作的结果, 包含匹配对/未匹配轨迹/未匹配检测;
 *
 *               这个结构是 DeepSORT 匹配函数的通用输出格式,
 *               与 ByteTrack 的 ByteMatchResult 结构类似但独立;
 */
struct MatchResult
{
    // matches: 匹配对列表, first = 轨迹索引, second = 检测索引;
    std::vector<std::pair<int32, int32>> matches;

    // unmatched_tracks: 未匹配的轨迹索引列表 (将在后续步骤中标记为丢失);
    std::vector<int32> unmatched_tracks;

    // unmatched_detections: 未匹配的检测索引列表 (将用于初始化新轨迹或参与 IoU 匹配);
    std::vector<int32> unmatched_detections;
};

/***
 * @description: 计算余弦距离矩阵;
 *               用于 ReID 外观特征的距离计算;
 *
 *               余弦距离 = 1 - 余弦相似度;
 *               余弦相似度 = dot(a, b) / (|a| * |b|);
 *
 *               范围: [0, 2], 0 表示完全相同, 2 表示完全相反;
 *               当特征向量为空时返回零矩阵 (不使用余弦距离);
 *
 *               @note 当前实现不依赖 ReID 特征;
 *               如果未配置 ReID 网络, features 和 track_features 均为空列表,
 *               余弦距离矩阵会退回全零矩阵, 级联匹配退化为纯马氏距离匹配;
 *
 * @param features      const std::vector<std::vector<float32>>& : 检测框的特征向量列表;
 *                     features[i] 是对应检测框的 ReID 特征;
 * @param track_features const std::vector<std::vector<float32>>& : 轨迹的特征向量列表;
 *                        track_features[j] 是对应轨迹的 ReID 特征;
 * @return std::vector<std::vector<float32>> : 余弦距离矩阵 [n_tracks x n_dets];
 */
inline vector2D<float32> cal_cosine_distance(const vector2D<float32>& features,        //
                                             const vector2D<float32>& track_features)  //
{
    size_t n_tracks = track_features.size();
    size_t n_dets = features.size();
    // 初始化成本矩阵为全零矩阵 (默认: 不使用余弦距离时的兜底值);
    vector2D<float32> cost_matrix(n_tracks, std::vector<float32>(n_dets, 0.0f));

    // 特征为空时返回零矩阵 (不使用余弦距离);
    if (features.empty() || track_features.empty() || features[0].empty() || track_features[0].empty())
    {
        return cost_matrix;
    }

    for (size_t i = 0; i < n_tracks; i++)
    {
        for (size_t j = 0; j < n_dets; j++)
        {
            // ---- 计算余弦相似度 ----
            // 公式: cos_sim = sum(a_k * b_k) / (sqrt(sum(a_k^2)) * sqrt(sum(b_k^2)));
            float32 dot = 0.0f;
            float32 norm_a = 0.0f;
            float32 norm_b = 0.0f;
            const std::vector<float32>& a = track_features[i];
            const std::vector<float32>& b = features[j];
            size_t dim = a.size();
            for (size_t k = 0; k < dim; k++)
            {
                dot += a[k] * b[k];     // 点积
                norm_a += a[k] * a[k];  // L2 范数平方 (a)
                norm_b += b[k] * b[k];  // L2 范数平方 (b)
            }
            norm_a = std::sqrt(norm_a);
            norm_b = std::sqrt(norm_b);
            // 余弦相似度 (避免除零);
            float32 cos_sim = (norm_a > 0.0f && norm_b > 0.0f) ? (dot / (norm_a * norm_b)) : 0.0f;
            // 余弦距离 = 1 - 余弦相似度;
            cost_matrix[i][j] = 1.0f - cos_sim;
        }
    }

    return cost_matrix;
}

/***
 * @description: 计算马氏距离矩阵 (门控);
 *               使用卡尔曼滤波器的 gating_distance() 方法计算;
 *
 *               马氏距离是"归一化后的欧氏距离", 考虑了各维度的方差和相关性;
 *               公式: d^2 = (z - z_pred)^T * S^(-1) * (z - z_pred);
 *               其中 z_pred = H * mean, S = H * P * H^T + R;
 *
 *               @note 这里的马氏距离后续会被 gating_distance() 内部的计算覆盖,
 *               当前函数主要用于构建级联匹配所需的距离矩阵;
 *
 * @param kf              KalmanFilter* : 卡尔曼滤波器指针;
 * @param tracks          std::vector<DeepSortTrack*>& : 轨迹指针列表 (用于获取 mean 和 covariance);
 * @param detections      const std::vector<BoxObject>& : 检测框列表 (使用 ltwh_expand 构建 xyah);
 * @param track_indices   const std::vector<int32>& : 参与匹配的轨迹索引;
 * @param detection_indices const std::vector<int32>& : 参与匹配的检测索引;
 * @return std::vector<std::vector<float32>> : 马氏距离矩阵 [n_tracks x n_dets];
 */
inline vector2D<float32> cal_gating_distance(KFBox* kf,                                    //
                                             std::vector<DeepSortTrack*>& tracks,          //
                                             const std::vector<BoxObject>& detections,     //
                                             const std::vector<int32>& track_indices,      //
                                             const std::vector<int32>& detection_indices)  //
{
    // 轨迹个数
    size_t n_tracks = track_indices.size();
    // 目标个数
    size_t n_dets = detection_indices.size();
    // 初始化成本矩阵为全零矩阵 (默认: 不使用马氏距离时的兜底值);
    // n_tracks 个 vector<float32>(n_dets, 0.0f);
    // n_dets 个 0.0f
    vector2D<float32> cost_matrix(n_tracks, std::vector<float32>(n_dets, 0.0f));

    // ---- 收集所有检测框的 xyah 测量值 ----
    // xyah = [cx, cy, 宽高比, 高度];
    // 使用 ltwh_expand (扩展框) 构建, 因为扩展框考虑了边框的缩放;
    std::vector<BOX_HMEAN> measurements;
    measurements.reserve(n_dets);  // 预先分配内存, 提高性能;

    // 遍历所有 候选检测框 都转为 卡尔曼观测向量mean
    for (size_t j = 0; j < n_dets; j++)
    {
        // 获取当前检测框的索引
        int32 det_idx = detection_indices[j];
        // 获取当前检测框
        const BoxObject& det = detections[det_idx];

        // ltwh_to_xyah: [left, top, width, height] -> [cx, cy, w/h, height];
        std::array<float32, 4> xyah_arr = BoxObject::ltwh_to_xyah(det.ltwh_expand);
        // 观测值矩阵
        BOX_HMEAN m;
        m << xyah_arr[0], xyah_arr[1], xyah_arr[2], xyah_arr[3];
        measurements.push_back(m);
    }

    // 遍历所有 候选轨迹
    for (size_t i = 0; i < n_tracks; i++)
    {
        // 获取当前轨迹的索引
        int32 track_idx = track_indices[i];
        // 获取当前轨迹
        DeepSortTrack* track = tracks[track_idx];

        // ---- 计算马氏距离 ----
        // gating_distance() 内部做了 Cholesky 分解和归一化;
        // 计算当前轨迹和所有检测目标的马氏距离
        Eigen::Matrix<float32, 1, Eigen::Dynamic> gating_dist =
            kf->gating_distance(track->mean, track->covariance, measurements);

        // 填充成本矩阵;
        for (int32 k = 0; k < gating_dist.cols(); k++)
        {
            cost_matrix[i][k] = gating_dist(0, k);
        }
    }

    return cost_matrix;
}

/***
 * @description: 计算融合距离矩阵 (马氏距离 + 余弦距离);
 *               DeepSORT 级联匹配中使用的组合距离;
 *
 *               公式 (DeepSORT 论文原始定义):
 *               combined_dist = (1 - lambda) * mahala_dist + lambda * cosine_dist;
 *               其中 lambda 通常取 0.98, 表示更信任外观特征 (余弦距离), 运动模型 (马氏距离) 辅助;
 *
 *               @note 当未配置 ReID 时, cosine_dist 是全零矩阵,
 *               此时 combined_dist = (1 - lambda) * mahala_dist, 退化为马氏距离匹配;
 *
 * @param mahala_dist   const std::vector<std::vector<float32>>& : 马氏距离矩阵;
 * @param cosine_dist   const std::vector<std::vector<float32>>& : 余弦距离矩阵;
 * @param lambda        float32 : 融合权重 (余弦距离的权重, 默认 0.98);
 * @return std::vector<std::vector<float32>> : 融合距离矩阵;
 */
inline vector2D<float32> cal_combined_distance(const vector2D<float32>& mahala_dist,  //
                                               const vector2D<float32>& cosine_dist,  //
                                               float32 lambda = 0.98f)                //
{
    size_t rows = mahala_dist.size();
    size_t cols = (rows > 0) ? mahala_dist[0].size() : 0;
    vector2D<float32> cost_matrix(rows, std::vector<float32>(cols, 0.0f));

    // 凸组合: combined[i][j] = (1 - lambda) * mahala[i][j] + lambda * cosine[i][j];
    float32 inv_lambda = 1.0f - lambda;
    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            cost_matrix[i][j] = inv_lambda * mahala_dist[i][j] + lambda * cosine_dist[i][j];
        }
    }

    return cost_matrix;
}

/***
 * @description: 应用门控过滤到成本矩阵;
 *               马氏距离超过卡方 95% 置信区间阈值的匹配被标记为无穷大成本;
 *
 *               为什么需要门控过滤?
 *               即使马氏距离很大, 匈牙利算法仍然可能分配这个匹配,
 *               因为它追求"全局最小成本", 而不是"局部成本是否合理".
 *               门控过滤就是在匈牙利之前, 把不合理的匹配预先排除掉.
 *
 *               门控阈值: chi2inv95[4] = 9.4877 (4 自由度, 95% 置信区间);
 *               这意味着如果马氏距离平方 > 9.4877,
 *               该检测与轨迹属于同一个目标的概率 < 5%;
 *
 * @param kf              KalmanFilter* : 卡尔曼滤波器指针;
 * @param cost_matrix     std::vector<std::vector<float32>>& : 待过滤的成本矩阵 (原地修改);
 * @param tracks          std::vector<DeepSortTrack*>& : 轨迹列表;
 * @param detections      const std::vector<BoxObject>& : 检测列表;
 * @param track_indices   const std::vector<int32>& : 轨迹索引;
 * @param detection_indices const std::vector<int32>& : 检测索引;
 * @param gated_cost      float32 : 超过门控时的替代成本值 (默认 INFTY_COST_DEEP);
 */
inline void gate_cost_matrix(KFBox* kf,                                    //
                             vector2D<float32>& cost_matrix,               //
                             std::vector<DeepSortTrack*>& tracks,          //
                             const std::vector<BoxObject>& detections,     //
                             const std::vector<int32>& track_indices,      //
                             const std::vector<int32>& detection_indices,  //
                             float32 gated_cost = INFTY_COST_DEEP)         //
{
    // 卡方 95% 置信区间阈值 (4 自由度);
    // 4 自由度对应我们的测量维度 [cx, cy, a, h];
    float64 gating_threshold = KFBox::chi2inv95[4];
    // 候选轨迹数量
    int32 n_tracks = static_cast<int32>(track_indices.size());
    // 候选目标数量
    int32 n_dets = static_cast<int32>(detection_indices.size());

    // ---- 收集所有检测框的 xyah 测量值 ----
    std::vector<BOX_HMEAN> measurements;
    measurements.reserve(static_cast<size_t>(n_dets));
    for (int32 j = 0; j < n_dets; j++)
    {
        int32 det_idx = detection_indices[j];
        const BoxObject& det = detections[det_idx];
        std::array<float32, 4> xyah_arr = BoxObject::ltwh_to_xyah(det.ltwh_expand);
        BOX_HMEAN m;
        m << xyah_arr[0], xyah_arr[1], xyah_arr[2], xyah_arr[3];
        measurements.push_back(m);
    }

    // ---- 遍历所有候选轨迹, 计算马氏距离并过滤 ----
    for (int32 i = 0; i < n_tracks; i++)
    {
        int32 track_idx = track_indices[i];
        DeepSortTrack* track = tracks[track_idx];

        // ---- 计算马氏距离并过滤 ----
        Eigen::Matrix<float32, 1, Eigen::Dynamic> gating_dist =
            kf->gating_distance(track->mean, track->covariance, measurements);

        for (int32 j = 0; j < n_dets; j++)
        {
            if (gating_dist(0, j) > static_cast<float32>(gating_threshold))
            {
                // 马氏距离超过阈值 -> 标记为无穷大, 阻止匈牙利算法选择这个匹配;
                cost_matrix[i][j] = gated_cost;
            }
        }
    }
}

/***
 * @description: 最小成本匹配 (带门控);
 *               使用匈牙利算法求解线性分配问题;
 *
 *               流程:
 *               1. 如果输入为空, 直接返回全未匹配结果;
 *               2. 使用 hungarian_solve() 求解最小成本匹配对;
 *               3. 检查每个匹配对的成本是否 <= max_distance;
 *               4. 成本超过阈值的匹配对视为未匹配;
 *
 * @param cost_matrix       const std::vector<std::vector<float32>>& : 成本矩阵;
 * @param max_distance      float32 : 最大允许距离, 超过此值的匹配被忽略;
 * @param track_indices     const std::vector<int32>& : 轨迹索引;
 * @param detection_indices const std::vector<int32>& : 检测索引;
 * @return MatchResult : 匹配结果;
 */
inline MatchResult min_cost_matching(const vector2D<float32>& cost_matrix,         //
                                     float32 max_distance,                         //
                                     const std::vector<int32>& track_indices,      //
                                     const std::vector<int32>& detection_indices)  //
{
    MatchResult res;

    int32 n_tracks = static_cast<int32>(track_indices.size());
    int32 n_dets = static_cast<int32>(detection_indices.size());

    // ---- 空输入直接返回 ----
    if (n_tracks == 0 || n_dets == 0)
    {
        res.unmatched_tracks.assign(track_indices.begin(), track_indices.end());
        res.unmatched_detections.assign(detection_indices.begin(), detection_indices.end());
        return res;
    }

    // ---- 调用 Munkres 匈牙利算法 ----
    // DeepSORT 原版使用 Munkres (非 LAPJV);
    // Munkres 与 LAPJV 都是 O(n^3) 的求解器, 但实现思路不同;
    // Munkres 适合稠密矩阵, LAPJV 在稀疏场景下更快;
    std::vector<std::pair<int32, int32>> pairs = hungarian_solve(cost_matrix);

    // ---- 构建已匹配轨迹和检测的 bool 标记 ----
    // 用于后续区分哪些是未匹配的;
    std::vector<bool> track_matched(n_tracks, false);
    std::vector<bool> det_matched(n_dets, false);

    // ---- 处理匈牙利算法的匹配结果 ----
    for (size_t p = 0; p < pairs.size(); p++)
    {
        int32 i = pairs[p].first;
        int32 j = pairs[p].second;
        // 只有成本 <= max_distance 的匹配才接受;
        if (i >= 0 && i < n_tracks && j >= 0 && j < n_dets && cost_matrix[i][j] <= max_distance)
        {
            // 注意! 这里将 track_indices 和 detection_indices 映射回原来的全局索引;
            res.matches.push_back(std::make_pair(track_indices[i], detection_indices[j]));
            track_matched[i] = true;
            det_matched[j] = true;
        }
    }

    // ---- 收集未匹配的轨迹 ----
    for (int32 i = 0; i < n_tracks; i++)
    {
        if (!track_matched[i])
        {
            res.unmatched_tracks.push_back(track_indices[i]);
        }
    }

    // ---- 收集未匹配的检测 ----
    for (int32 j = 0; j < n_dets; j++)
    {
        if (!det_matched[j])
        {
            res.unmatched_detections.push_back(detection_indices[j]);
        }
    }

    return res;
}

/***
 * @description: 级联匹配 (DeepSORT 核心);
 *
 *               这是 DeepSORT 区别于 SORT 的关键创新点;
 *               核心思想是: "优先匹配最近更新过的轨迹";
 *
 *               为什么需要级联?
 *
 *               假设场景:
 *                - 轨迹 A: time_since_update = 1 (刚刚更新过);
 *                - 轨迹 B: time_since_update = 10 (丢失了 10 帧);
 *                - 检测框 D: 恰好在 A 和 B 的预测位置中间;
 *
 *               如果不分级: 匈牙利算法可能把 D 匹配给 B,
 *               因为 B 的马氏距离可能很大 (预测不准反而覆盖范围广);
 *
 *               级联匹配: 先让 A (level=0) 在一级检测中匹配,
 *               B 只能在后面的 level 中匹配剩余的检测;
 *
 *               这样 A 优先保证了"最近确认的目标不被未确认目标抢走检测框";
 *
 *               参考 deepsort linear_assignment::matching_cascade();
 *
 * @param kf             KalmanFilter* : 卡尔曼滤波器指针;
 * @param max_distance   float32 : 最大允许距离;
 * @param cascade_depth  int32 : 级联深度 (通常为 max_age);
 * @param tracks         std::vector<DeepSortTrack*>& : 轨迹列表;
 * @param detections     const std::vector<BoxObject>& : 检测列表;
 * @param track_indices  const std::vector<int32>& : 参与匹配的轨迹索引;
 * @param detection_indices const std::vector<int32>& : 参与匹配的检测索引;
 * @return MatchResult : 匹配结果;
 */
inline MatchResult cascade_matching(KFBox* kf,                                 //
                                    float32 max_distance,                      //
                                    int32 cascade_depth,                       //
                                    std::vector<DeepSortTrack*>& tracks,       //
                                    const std::vector<BoxObject>& detections,  //
                                    const std::vector<int32>& track_indices,   //
                                    const vector2D<float32>& features,         //
                                    const vector2D<float32>& track_features,   //
                                    float32 max_cosine_distance,               //
                                    float32 lambda_cosine_weight)              //
{
    MatchResult final_res;

    // ---- 初始化: 所有检测都视为未匹配 ----
    std::vector<int32> unmatched_detections;
    for (size_t j = 0; j < detections.size(); j++)
    {
        unmatched_detections.push_back(static_cast<int32>(j));
    }

    // ---- 按 time_since_update 分层匹配 ----
    // level = 0: 匹配 time_since_update == 1 的轨迹 (最新);
    // level = 1: 匹配 time_since_update == 2 的轨迹;
    // ...
    // level = cascade_depth-1: 匹配 time_since_update == cascade_depth 的轨迹;
    for (int32 level = 0; level < cascade_depth; level++)
    {
        if (unmatched_detections.empty())
        {
            // 没有多余的检测框了, 提前结束;
            break;
        }

        // ---- 筛选出当前层的轨迹 ----
        // time_since_update == 1 + level;
        std::vector<int32> level_track_indices;
        for (size_t k = 0; k < track_indices.size(); k++)
        {
            int32 idx = track_indices[k];
            if (tracks[idx]->time_since_update == 1 + level)
            {
                level_track_indices.push_back(idx);
            }
        }

        if (level_track_indices.empty())
        {
            // 当前层没有轨迹需要匹配, 跳到下一层;
            continue;
        }

        // ---- 构建当前层的特征子集 ----
        // 因为随着 level 增加, unmatched_detections 会越来越少;
        // 只计算剩余检测框的特征, 避免无谓的计算;
        vector2D<float32> level_features;
        vector2D<float32> level_track_features;
        for (size_t j = 0; j < unmatched_detections.size(); j++)
        {
            int32 det_idx = unmatched_detections[j];
            if (static_cast<size_t>(det_idx) < features.size())
            {
                level_features.push_back(features[det_idx]);
            }
            else
            {
                // 特征索引越界时用空列表占位;
                level_features.push_back(std::vector<float32>());
            }
        }
        for (size_t i = 0; i < level_track_indices.size(); i++)
        {
            int32 track_idx = level_track_indices[i];
            if (static_cast<size_t>(track_idx) < track_features.size())
            {
                level_track_features.push_back(track_features[track_idx]);
            }
            else
            {
                level_track_features.push_back(std::vector<float32>());
            }
        }

        // ---- 计算马氏距离矩阵 ----
        vector2D<float32> maha_dist = cal_gating_distance(kf,                   //
                                                          tracks,               //
                                                          detections,           //
                                                          level_track_indices,  //
                                                          unmatched_detections);

        // ---- 计算余弦距离矩阵 ----
        vector2D<float32> cosine_dist = cal_cosine_distance(level_features, level_track_features);

        // ---- 融合距离: combined = (1 - lambda) * maha + lambda * cosine ----
        vector2D<float32> combined_dist = cal_combined_distance(maha_dist, cosine_dist, lambda_cosine_weight);

        // ---- 门控过滤: 马氏距离超过阈值 -> 无穷大 ----
        gate_cost_matrix(kf,                    //
                         combined_dist,         //
                         tracks,                //
                         detections,            //
                         level_track_indices,   //
                         unmatched_detections,  //
                         INFTY_COST_DEEP);

        // ---- 余弦距离门控: 超过 max_cosine_distance -> 无穷大 ----
        for (size_t i = 0; i < level_track_indices.size(); i++)
        {
            for (size_t j = 0; j < unmatched_detections.size(); j++)
            {
                if (cosine_dist[i][j] > max_cosine_distance)
                {
                    combined_dist[i][j] = INFTY_COST_DEEP;
                }
            }
        }

        // ---- 最小成本匹配 (匈牙利) ----
        MatchResult level_res = min_cost_matching(combined_dist,        //
                                                  max_distance,         //
                                                  level_track_indices,  //
                                                  unmatched_detections);

        // ---- 合并当前层的匹配结果到最终结果 ----
        for (size_t m = 0; m < level_res.matches.size(); m++)
        {
            final_res.matches.push_back(level_res.matches[m]);
        }
        for (size_t m = 0; m < level_res.unmatched_tracks.size(); m++)
        {
            final_res.unmatched_tracks.push_back(level_res.unmatched_tracks[m]);
        }

        // ---- 更新未匹配检测列表 ----
        // 在当前层匹配掉的检测框, 不再参与后续层的匹配;
        unmatched_detections = level_res.unmatched_detections;
    }

    // ---- 最后剩余的未匹配检测 ----
    final_res.unmatched_detections = unmatched_detections;

    return final_res;
}

/***
 * @description: 计算 IoU 距离矩阵;
 *               用于第二关联阶段 (低置信度检测 + 未匹配轨迹);
 *
 *               IoU 距离 = 1 - IoU;
 *               IoU (Intersection over Union) = 交集面积 / 并集面积;
 *
 *               使用扩展框 (ltwh_expand) 计算 IoU, 而不是原始框;
 *               扩展框通常比原始框大 10~20%, 使得 IoU 匹配更鲁棒;
 *
 * @param tracks          std::vector<DeepSortTrack*>& : 轨迹列表 (使用 ltwh_expand 计算 IoU);
 * @param detections      const std::vector<BoxObject>& : 检测列表;
 * @param track_indices   const std::vector<int32>& : 轨迹索引;
 * @param detection_indices const std::vector<int32>& : 检测索引;
 * @return std::vector<std::vector<float32>> : IoU 距离矩阵 (1 - IoU);
 */
inline vector2D<float32> iou_cost_matrix(std::vector<DeepSortTrack*>& tracks,          //
                                         const std::vector<BoxObject>& detections,     //
                                         const std::vector<int32>& track_indices,      //
                                         const std::vector<int32>& detection_indices)  //
{
    size_t n_tracks = track_indices.size();
    size_t n_dets = detection_indices.size();
    // 初始化为 1.0 (最大距离, 即 IoU = 0 时的距离);
    vector2D<float32> cost_matrix(n_tracks, std::vector<float32>(n_dets, 1.0f));

    for (size_t i = 0; i < n_tracks; i++)
    {
        int32 track_idx = track_indices[i];
        DeepSortTrack* track = tracks[track_idx];

        // ---- 获取轨迹预测框的坐标 ----
        // DeepSortTrack 的 get_x1/get_y1/get_x2/get_y2 返回卡尔曼预测后的边界框;
        float32 tx1 = track->get_x1();
        float32 ty1 = track->get_y1();
        float32 tx2 = track->get_x2();
        float32 ty2 = track->get_y2();
        float32 area_track = (tx2 - tx1) * (ty2 - ty1);

        for (size_t j = 0; j < n_dets; j++)
        {
            int32 det_idx = detection_indices[j];
            const BoxObject& det = detections[det_idx];

            // ---- 获取检测框的坐标 (ltwh -> xyxy) ----
            float32 dx1 = det.ltwh_expand[0];
            float32 dy1 = det.ltwh_expand[1];
            float32 dx2 = det.ltwh_expand[0] + det.ltwh_expand[2];
            float32 dy2 = det.ltwh_expand[1] + det.ltwh_expand[3];

            // ---- 计算交集 (Intersection) ----
            float32 ix1 = (tx1 > dx1) ? tx1 : dx1;  // max of lefts
            float32 iy1 = (ty1 > dy1) ? ty1 : dy1;  // max of tops
            float32 ix2 = (tx2 < dx2) ? tx2 : dx2;  // min of rights
            float32 iy2 = (ty2 < dy2) ? ty2 : dy2;  // min of bottoms

            float32 iw = ix2 - ix1;
            float32 ih = iy2 - iy1;
            iw = (iw > 0.0f) ? iw : 0.0f;  // 防止负宽度
            ih = (ih > 0.0f) ? ih : 0.0f;  // 防止负高度

            float32 area_intersection = iw * ih;
            float32 area_det = (dx2 - dx1) * (dy2 - dy1);
            float32 area_union = area_track + area_det - area_intersection;

            float32 iou = (area_union > 0.0f) ? (area_intersection / area_union) : 0.0f;
            cost_matrix[i][j] = 1.0f - iou;
        }
    }

    return cost_matrix;
}

/***
 * @description: IoU 匹配 (用于第二关联阶段);
 *               在级联匹配之后, 对未匹配的轨迹和检测进行 IoU 关联;
 *
 *               @note 只有 time_since_update == 1 的轨迹参与 IoU 匹配;
 *               time_since_update > 1 的轨迹直接视为未匹配 (跳过);
 *               原因: 丢失多帧的轨迹预测框不可靠, IoU 计算无意义;
 *
 * @param tracks          std::vector<DeepSortTrack*>& : 轨迹指针列表;
 * @param detections      const std::vector<BoxObject>& : 检测列表;
 * @param track_indices   const std::vector<int32>& : 轨迹索引;
 * @param detection_indices const std::vector<int32>& : 检测索引;
 * @param max_iou_distance float32 : 最大 IoU 距离阈值;
 * @return MatchResult : 匹配结果;
 */
inline MatchResult iou_matching(std::vector<DeepSortTrack*>& tracks,          //
                                const std::vector<BoxObject>& detections,     //
                                const std::vector<int32>& track_indices,      //
                                const std::vector<int32>& detection_indices,  //
                                float32 max_iou_distance)                     //
{
    // ---- 构建 IoU 成本矩阵 ----
    // 对 time_since_update > 1 的轨迹, 距离直接设为无穷大 (跳过);
    size_t n_tracks = track_indices.size();
    size_t n_dets = detection_indices.size();
    vector2D<float32> cost_matrix(n_tracks, std::vector<float32>(n_dets, INFTY_COST_DEEP));

    for (size_t i = 0; i < n_tracks; i++)
    {
        int32 track_idx = track_indices[i];
        // 跳过长时间未更新的轨迹 (time_since_update > 1);
        if (tracks[track_idx]->time_since_update > 1)
        {
            continue;
        }
        // ---- 计算 IoU 距离 ----
        DeepSortTrack* track = tracks[track_idx];
        float32 tx1 = track->get_x1();
        float32 ty1 = track->get_y1();
        float32 tx2 = track->get_x2();
        float32 ty2 = track->get_y2();
        float32 area_track = (tx2 - tx1) * (ty2 - ty1);

        for (size_t j = 0; j < n_dets; j++)
        {
            int32 det_idx = detection_indices[j];
            const BoxObject& det = detections[det_idx];

            float32 dx1 = det.ltwh_expand[0];
            float32 dy1 = det.ltwh_expand[1];
            float32 dx2 = det.ltwh_expand[0] + det.ltwh_expand[2];
            float32 dy2 = det.ltwh_expand[1] + det.ltwh_expand[3];

            float32 ix1 = (tx1 > dx1) ? tx1 : dx1;
            float32 iy1 = (ty1 > dy1) ? ty1 : dy1;
            float32 ix2 = (tx2 < dx2) ? tx2 : dx2;
            float32 iy2 = (ty2 < dy2) ? ty2 : dy2;

            float32 iw = ix2 - ix1;
            float32 ih = iy2 - iy1;
            iw = (iw > 0.0f) ? iw : 0.0f;
            ih = (ih > 0.0f) ? ih : 0.0f;

            float32 area_intersection = iw * ih;
            float32 area_det = (dx2 - dx1) * (dy2 - dy1);
            float32 area_union = area_track + area_det - area_intersection;

            float32 iou = (area_union > 0.0f) ? (area_intersection / area_union) : 0.0f;
            cost_matrix[i][j] = 1.0f - iou;
        }
    }

    // ---- 用匈牙利算法求解 IoU 匹配 ----
    return min_cost_matching(cost_matrix, max_iou_distance, track_indices, detection_indices);
}

}  // namespace deepsort
}  // namespace tracker

#endif  // !__DEEPSORT_MATCHING__H__