/***
 * @Author       : gxs
 * @Date         : 2026-07-19 14:00:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-19 14:00:00
 * @FilePath     : /visionAlgorithm/lib/tracker/deepsort/NNMetric.hpp
 * @Description  : DeepSORT 近邻特征度量库实现;
 *
 *                 ============================================================
 *                 NNMetric 设计思想
 *                 ============================================================
 *
 *                 DeepSORT 的 ReID 匹配需要维护一个"特征库":
 *                 - 每个轨迹保存最近 budget 帧的 ReID 特征向量;
 *                 - 匹配时, 计算检测框特征与轨迹历史特征的"最近邻余弦距离";
 *                 - 最近邻余弦距离 = 检测框特征 与 轨迹历史特征列表中最相近的特征 之间的余弦距离;
 *
 *                 滚动窗口设计:
 *                 - 每帧最多保存 budget 个特征向量 (超出时删除最旧的);
 *                 - 新特征追加到列表尾部, 最旧特征从头部弹出;
 *                 - 这样每个轨迹的特征库始终反映最近的外观信息;
 *
 *                 partial_fit 更新策略:
 *                 - 每帧匹配后, 将已匹配确认轨迹的新特征写入特征库;
 *                 - 删除非活跃轨迹 (已删除轨迹) 的特征, 避免内存无限增长;
 *                 ============================================================
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __NNMETRIC__H__
#define __NNMETRIC__H__

#include <cmath>
#include <map>
#include <vector>

#include "types.hpp"  // float32, int32

namespace tracker
{
namespace deepsort
{

/***
 * @description: 近邻特征度量库 (NearNeighbor Distance Metric);
 *               维护每个轨迹的 ReID 外观特征历史, 计算最近邻余弦距离;
 *
 *               核心数据结构:
 *               - samples: 特征库, key=track_id, value=历史特征向量列表;
 *               - budget: 每个轨迹最多保留的特征数量 (滚动窗口);
 *               - mating_threshold: 余弦距离阈值, 超过此值的匹配被拒绝;
 *
 *               两个核心接口:
 *               - distance(): 计算目标轨迹与检测框之间的最小余弦距离矩阵;
 *               - partial_fit(): 更新特征库并清理非活跃轨迹的特征;
 */
class NNMetric
{
   public:
    // mating_threshold: 余弦距离匹配阈值 (0~1, 越小越严格);
    // 超过此阈值的 track-det 对将被拒绝匹配;
    // 参考 DeepSORT 原版默认值: 0.4 (用于 MOT 数据集) 或 0.2 (用于人脸场景);
    float32 mating_threshold;

    // budget: 每个轨迹最多保留的特征向量数量 (滚动窗口大小);
    // 超过此数量时, 最旧的特征向量将被弹出;
    // 参考 DeepSORT 原版默认值: 100;
    int32 budget;

    // samples: 特征库, key = track_id, value = 该轨迹的历史特征向量列表;
    // 使用 map 方便按 track_id 索引, 不需要关心轨迹的向量位置;
    std::map<int32, std::vector<std::vector<float32>>> samples;

   public:
    /***
     * @description: 默认构造函数;
     *               使用 DeepSORT 论文中推荐的默认参数;
     */
    NNMetric() : mating_threshold(0.4f), budget(100) {}

    /***
     * @description: 有参构造函数;
     * @param thresh float32 : 余弦距离阈值;
     * @param bgt    int32   : 滚动窗口大小;
     */
    NNMetric(float32 thresh, int32 bgt) : mating_threshold(thresh), budget(bgt) {}

    /***
     * @description: 计算目标轨迹与检测框之间的最小余弦距离矩阵;
     *               对每个目标轨迹, 在其历史特征列表中寻找与检测框最相近的特征;
     *               返回矩阵 [n_targets x n_detections];
     *
     *               "最近邻"的含义:
     *               对轨迹 i 和检测框 j, cost[i][j] = min over k ( cosine_dist(stored_feat_k, det_feat_j) );
     *               即: 轨迹的所有历史特征中, 距离检测框最近的那个距离值;
     *
     *               当轨迹没有历史特征时, 返回最大距离 1.0, 拒绝该匹配;
     *
     * @param features std::vector<std::vector<float32>> : 检测框的 ReID 特征列表 [n_dets x feat_dim];
     * @param targets  std::vector<int32> : 目标轨迹的 track_id 列表 [n_targets];
     * @return std::vector<std::vector<float32>> : 最小余弦距离矩阵 [n_targets x n_dets];
     */
    std::vector<std::vector<float32>> distance(const std::vector<std::vector<float32>>& features,  //
                                               const std::vector<int32>& targets)
    {
        size_t n_targets = targets.size();
        size_t n_dets = features.size();
        // 初始化为 1.0 (最大距离, 表示没有存储特征时默认拒绝);
        std::vector<std::vector<float32>> cost(n_targets, std::vector<float32>(n_dets, 1.0f));

        for (size_t i = 0; i < n_targets; i++)
        {
            int32 tid = targets[i];
            std::map<int32, std::vector<std::vector<float32>>>::const_iterator it = this->samples.find(tid);

            // 该轨迹没有历史特征时跳过 (保持默认最大距离);
            if (it == this->samples.end() || it->second.empty())
            {
                continue;
            }

            const std::vector<std::vector<float32>>& stored = it->second;
            for (size_t j = 0; j < n_dets; j++)
            {
                // 计算最近邻余弦距离: 取所有历史特征与当前检测框的最小余弦距离;
                cost[i][j] = this->_nn_cosine_distance(stored, features[j]);
            }
        }
        return cost;
    }

    /***
     * @description: 更新特征库 (partial_fit);
     *               将本帧匹配成功的轨迹特征写入特征库, 同时清理非活跃轨迹的特征;
     *
     *               参考 DeepSORT NearNeighborDisMetric::partial_fit();
     *
     *               两个操作:
     *               - 删除所有非活跃轨迹的特征 (已删除轨迹, 避免内存泄漏);
     *               - 追加新特征到各轨迹的特征列表 (滚动窗口超出时弹出最旧的);
     *
     * @param tid_feats     : (track_id, feature) 对列表, 表示本帧匹配成功的轨迹及其特征;
     * @param active_targets: 当前帧活跃的轨迹 ID 列表 (仅保留这些轨迹的特征);
     */
    void partial_fit(const std::vector<std::pair<int32, std::vector<float32>>>& tid_feats,  //
                     const std::vector<int32>& active_targets)
    {
        // ---- 清理非活跃轨迹的特征 ----
        // 遍历特征库, 删除不在 active_targets 中的轨迹的特征;
        std::map<int32, std::vector<std::vector<float32>>>::iterator it = this->samples.begin();
        while (it != this->samples.end())
        {
            bool is_active = false;
            for (size_t k = 0; k < active_targets.size(); k++)
            {
                if (active_targets[k] == it->first)
                {
                    is_active = true;
                    break;
                }
            }
            if (!is_active)
            {
                // 非活跃轨迹: 从特征库中删除;
                it = this->samples.erase(it);
            }
            else
            {
                ++it;
            }
        }

        // ---- 追加新特征到特征库 ----
        for (size_t i = 0; i < tid_feats.size(); i++)
        {
            int32 tid = tid_feats[i].first;
            const std::vector<float32>& feat = tid_feats[i].second;

            // 空特征跳过 (未使用 ReID 时 feature 为空);
            if (feat.empty())
            {
                continue;
            }

            // 追加特征到对应轨迹的列表末尾;
            this->samples[tid].push_back(feat);

            // 超出滚动窗口时, 弹出最旧的特征 (FIFO 队列策略);
            while (static_cast<int32>(this->samples[tid].size()) > this->budget)
            {
                this->samples[tid].erase(this->samples[tid].begin());
            }
        }
    }

    /***
     * @description: 获取指定轨迹的平均特征向量;
     *               用于向 cascade_matching 提供 track_features 参数;
     *               对所有历史特征取均值, 作为轨迹的"代表特征";
     *
     *               为什么用均值而非最近邻?
     *               cascade_matching 接受的是单一特征向量 (不是特征列表);
     *               用均值可以将多帧特征信息浓缩到一个向量, 计算效率更高;
     *               真正的最近邻匹配可通过 distance() 方法实现;
     *
     * @param track_id int32 : 轨迹 ID;
     * @return std::vector<float32> : 平均特征向量, 若无历史特征则返回空向量;
     */
    std::vector<float32> get_mean_feature(int32 track_id) const
    {
        std::map<int32, std::vector<std::vector<float32>>>::const_iterator it = this->samples.find(track_id);
        if (it == this->samples.end() || it->second.empty())
        {
            // 没有历史特征时返回空向量;
            return std::vector<float32>();
        }

        const std::vector<std::vector<float32>>& feats = it->second;
        size_t dim = feats[0].size();
        std::vector<float32> mean(dim, 0.0f);

        // 累加所有历史特征;
        for (size_t i = 0; i < feats.size(); i++)
        {
            for (size_t d = 0; d < dim; d++)
            {
                mean[d] += feats[i][d];
            }
        }

        // 除以特征数量得到均值;
        float32 n = static_cast<float32>(feats.size());
        for (size_t d = 0; d < dim; d++)
        {
            mean[d] /= n;
        }
        return mean;
    }

   private:
    /***
     * @description: 计算两个特征向量之间的余弦距离;
     *               余弦距离 = 1 - 余弦相似度;
     *               余弦相似度 = dot(a, b) / (||a|| * ||b||);
     *               范围: [0, 2], 0 表示完全相同, 2 表示完全相反;
     *
     * @param a const std::vector<float32>& : 特征向量 a;
     * @param b const std::vector<float32>& : 特征向量 b;
     * @return float32 : 余弦距离;
     */
    float32 _cosine_distance(const std::vector<float32>& a,  //
                             const std::vector<float32>& b) const
    {
        float32 dot = 0.0f;
        float32 na = 0.0f;
        float32 nb = 0.0f;
        size_t dim = a.size();
        for (size_t k = 0; k < dim; k++)
        {
            dot += a[k] * b[k];
            na += a[k] * a[k];
            nb += b[k] * b[k];
        }
        na = std::sqrt(na);
        nb = std::sqrt(nb);

        // 向量模为零时返回最大距离 1.0 (防止除零);
        if (na < 1e-6f || nb < 1e-6f)
        {
            return 1.0f;
        }

        float32 cos_sim = dot / (na * nb);
        // 钳位到 [-1, 1] 避免浮点误差导致 acos 失败;
        if (cos_sim > 1.0f)
            cos_sim = 1.0f;
        if (cos_sim < -1.0f)
            cos_sim = -1.0f;

        return 1.0f - cos_sim;
    }

    /***
     * @description: 计算特征列表 x 与查询向量 y 之间的最小余弦距离;
     *               即: min over i ( cosine_dist(x[i], y) );
     *               这是"最近邻"的核心计算逻辑;
     *
     *               参考 DeepSORT nn_matching.py::_nn_cosine_distance();
     *
     * @param x const std::vector<std::vector<float32>>& : 轨迹的历史特征列表;
     * @param y const std::vector<float32>& : 检测框的特征向量 (查询);
     * @return float32 : 最小余弦距离;
     */
    float32 _nn_cosine_distance(const std::vector<std::vector<float32>>& x,  //
                                const std::vector<float32>& y) const
    {
        float32 min_dist = 1.0f;
        for (size_t i = 0; i < x.size(); i++)
        {
            float32 d = this->_cosine_distance(x[i], y);
            if (d < min_dist)
            {
                min_dist = d;
            }
        }
        return min_dist;
    }
};

}  // namespace deepsort
}  // namespace tracker

#endif  // !__NNMETRIC__H__
