/***
 * @Author       : gxs
 * @Date         : 2026-06-22 21:35:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-07-20 19:50:00
 * @FilePath     : /visionAlgorithm/lib/tracker/bytetrack/lapjv.hpp
 * @Description  : Jonker-Volgenant 线性分配算法 (LAPJV) 实现;
 *                 用于目标跟踪中的数据关联 (匈牙利匹配);
 *
 *                 ============================================================
 *                 一. LAPJV 算法背景
 *                 ============================================================
 *
 *                 LAPJV (Jonker-Volgenant Algorithm for Linear Assignment Problem)
 *                 是一种专门用于解决线性分配问题 (LAP) 的经典算法;
 *
 *                 线性分配问题 (LAP) 定义:
 *                   给定 n 个工人和 n 个任务, 每个工人执行每个任务有一个成本 C[i][j];
 *                   求一种一一对应的分配方案, 使得总成本 sum(C[i][x[i]]) 最小;
 *                   约束: 每个工人恰好分配一个任务, 每个任务恰好分配一个工人;
 *
 *                 在目标跟踪中的应用:
 *                   - "工人" = 已有轨迹 (tracks);
 *                   - "任务" = 当前帧的检测框 (detections);
 *                   - C[i][j] = 轨迹 i 与检测 j 的 IoU 距离 (1 - IoU);
 *                   - 求解最优匹配, 使得总 IoU 距离最小 (即总 IoU 最大);
 *
 *                 为什么用 LAPJV 而不是 Munkres (匈牙利算法)?
 *                   - 两者都是 O(n^3) 复杂度;
 *                   - LAPJV 在稠密矩阵上实测更快 (常数因子更小);
 *                   - ByteTrack 的 Python 原版就使用 lap.lapjv 库;
 *                   - C++ 移植保持 API 和行为一致;
 *
 *                 ============================================================
 *                 二. LAPJV 算法原理 (对偶理论)
 *                 ============================================================
 *
 *                 LAPJV 基于 LAP 的对偶理论;
 *
 *                 原始问题 (Primal):
 *                   min sum(C[i][x[i]])
 *                   s.t. x 是一个排列 (行到列的双射);
 *
 *                 对偶问题 (Dual):
 *                   引入对偶变量 u[i] (行价格) 和 v[j] (列价格);
 *                   对偶可行性条件: u[i] + v[j] <= C[i][j] 对所有 (i,j) 成立;
 *                   max sum(u[i]) + sum(v[j]);
 *
 *                 互补松弛条件 (Complementary Slackness):
 *                   如果 x[i] = j 是最优分配, 则 u[i] + v[j] = C[i][j];
 *                   即: 被选中的匹配对必须满足 "约简成本 = 0";
 *                   约简成本定义: c_bar[i][j] = C[i][j] - u[i] - v[j] >= 0;
 *
 *                 LAPJV 的核心思路:
 *                   从一组初始对偶变量 (u, v) 出发;
 *                   通过三个阶段逐步改善对偶变量和部分分配;
 *                   直到找到一个完全分配, 使得所有分配的约简成本都为 0;
 *                   此时根据强对偶性, 这个分配就是最优解;
 *
 *                 ============================================================
 *                 三. LAPJV 三阶段详解
 *                 ============================================================
 *
 *                 ---- 阶段 1: 列约简与行转移 (Column Reduction & Row Transfer) ----
 *                 函数: _ccrrt_dense()
 *
 *                 目标: 初始化列对偶变量 v[j], 并尝试做初步分配;
 *
 *                 步骤:
 *                 (1) 对每列 j, 找到该列的最小成本行:
 *                       v[j] = min_i(C[i][j])
 *                       y[j] = argmin_i(C[i][j])
 *                     这意味着列 j "标价"为它最便宜的成本;
 *
 *                 (2) 对于只被一列选中的行 (unique), 建立正式分配 x[i]=j, y[j]=i;
 *                     如果一行被多列选中, 则保留第一次分配, 后来的列 j 设为 y[j]=-1;
 *
 *                 (3) 对于已经被唯一分配的行 i (分配到列 j),
 *                     计算 v[j] 的额外约简:
 *                       v[j] -= min_{j2!=j}(C[i][j2] - v[j2])
 *                     这是为了收紧对偶变量, 使后续阶段更高效;
 *
 *                 (4) 收集所有未被分配的行 (free_rows), 传递给下一阶段;
 *
 *                 输出:
 *                   - x[i]: 行 i 分配到的列 (-1 表示未分配);
 *                   - y[j]: 列 j 分配到的行 (-1 表示未分配);
 *                   - v[j]: 列对偶变量;
 *                   - free_rows: 未分配行的索引列表;
 *                   - 返回值: 未分配行的数量;
 *
 *                 ---- 阶段 2: 增广行约简 (Augmenting Row Reduction) ----
 *                 函数: _carr_dense()
 *
 *                 目标: 对每个自由行, 通过局部交换改善分配;
 *                 最多执行 2 轮 (经验值, 足以处理大多数冲突);
 *
 *                 对每个自由行 free_i:
 *                 (1) 找到约简成本最小的列 j1 和次小的列 j2:
 *                       v1 = C[free_i][j1] - v[j1]  (最小约简成本)
 *                       v2 = C[free_i][j2] - v[j2]  (次小约简成本)
 *
 *                 (2) 检查是否可以"降价":
 *                       v1_new = v[j1] - (v2 - v1)
 *                     如果 v1_new < v[j1], 则降低 v[j1];
 *                     这意味着列 j1 "更便宜"了, 从而可能将原来占据 j1 的行挤出;
 *
 *                 (3) 分配策略:
 *                     - 如果 j1 当前未被分配 (y[j1] < 0):
 *                       直接将 free_i 分配到 j1;
 *                     - 如果 j1 已被行 i0 占据:
 *                       a) 若降价成功: 将 free_i 分配到 j1, 将 i0 "踢"回自由行列表
 *                          (放到当前位置前面, 让它在本轮中立即重新处理);
 *                       b) 若降价失败但 j2 可用: 改为将 free_i 分配到 j2,
 *                          将 i0 加入下一轮的自由行列表;
 *
 *                 (4) 使用 rr_cnt 计数器防止无限循环:
 *                     如果 rr_cnt >= current * n, 退化为简单的"将被挤出行加入下一轮";
 *
 *                 输出: 新的自由行列表和数量;
 *
 *                 ---- 阶段 3: 增广 (Augmentation) ----
 *                 函数: _ca_dense()
 *
 *                 目标: 对阶段 2 后仍然未分配的行, 执行完整的最短增广路径搜索;
 *                 这是算法的"兜底"步骤, 保证所有行都能找到分配;
 *
 *                 对每个剩余的自由行 free_i:
 *                 (1) 调用 find_path_dense() 执行 Dijkstra 式最短路径搜索:
 *                     - 从 free_i 出发, 在约简成本图上搜索增广路径;
 *                     - 增广路径: free_i -> j1 -> i1 -> j2 -> i2 -> ... -> jk (free column);
 *                     - 路径终点是一个尚未被分配的列 jk;
 *
 *                 (2) 沿增广路径"翻转"分配:
 *                     while (i != free_i):
 *                       i = pred[j]        // j 的前驱行;
 *                       y[j] = i           // 将列 j 分配给行 i;
 *                       swap(j, x[i])      // 将行 i 从旧列交换到新列;
 *                     最终 free_i 获得了一个列的分配, 路径上的其他行也重新调整;
 *
 *                 find_path_dense() 内部机制:
 *                   使用类似 Dijkstra 的 SCAN/TODO 列表管理:
 *                   - READY: 已确定最短距离的列 (d[j] 不会再变);
 *                   - SCAN:  正在扫描的列 (d[j] 已确定, 但还没通过它们松弛其他列);
 *                   - TODO:  尚未确定最短距离的列 (d[j] 可能还会减小);
 *
 *                   列序列 cols[0..n-1] 被分为三段:
 *                     [0, n_ready)  = READY 列
 *                     [lo, hi)      = SCAN 列
 *                     [hi, n)       = TODO 列
 *
 *                   算法循环:
 *                     a) _find_dense(): 从 TODO 中找到 d[j] 最小的列, 移入 SCAN;
 *                     b) _scan_dense(): 遍历 SCAN 列, 对每个 SCAN 列 j (已分配行 i=y[j]),
 *                        尝试通过行 i 松弛所有 TODO 列:
 *                          cred_ij = C[i][j'] - v[j'] - h
 *                          if cred_ij < d[j']: d[j'] = cred_ij, pred[j'] = i;
 *                        如果某个 TODO 列 j' 的 d[j'] 降到了当前最小值且 y[j'] < 0,
 *                        则找到了增广路径终点, 立即返回 j';
 *                     c) 重复直到找到一个未分配的列;
 *
 *                   路径找到后, 调整对偶变量:
 *                     v[j] += d[j] - mind  (对所有 READY 列);
 *                   这保证了互补松弛条件在新分配下仍然成立;
 *
 *                 ============================================================
 *                 四. 非方阵处理与成本阈值 (lapjv 包装函数)
 *                 ============================================================
 *
 *                 LAPJV 核心算法要求输入必须是 n x n 方阵;
 *                 但在目标跟踪中, 轨迹数 (n_rows) 通常不等于检测数 (n_cols);
 *                 因此需要将非方阵扩展为方阵;
 *
 *                 扩展策略 (来自参考项目 bytetracker/src/utils.cpp):
 *                   将 n_rows x n_cols 矩阵扩展为 N x N 方阵,
 *                   其中 N = n_rows + n_cols;
 *
 *                   扩展后矩阵的分区:
 *
 *                     ┌─────────────────────┬─────────────────────┐
 *                     │  左上角              │  右上角              │
 *                     │  (n_rows x n_cols)   │  (n_rows x n_rows)  │
 *                     │  原始成本 C[i][j]    │  cost_limit / 2.0   │
 *                     ├─────────────────────┼─────────────────────┤
 *                     │  左下角              │  右下角              │
 *                     │  (n_cols x n_cols)   │  (n_cols x n_rows)  │
 *                     │  cost_limit / 2.0    │  0.0                │
 *                     └─────────────────────┴─────────────────────┘
 *
 *                   填充值含义:
 *                   - cost_limit / 2.0: "惩罚区域";
 *                     如果某个真实行被分配到虚拟列 (右上角), 成本为 cost_limit/2;
 *                     这比阈值以内的真实匹配成本高, 所以算法会优先选择真实匹配;
 *                     但比 LARGE (无穷大) 小, 所以算法仍然能"勉强"做出分配;
 *
 *                   - 0.0 (右下角): 虚拟行与虚拟列之间零成本匹配;
 *                     这些虚拟对虚拟的匹配不消耗实际成本;
 *                     它们存在的目的是让方阵中的每一行都能找到一列;
 *
 *                   隐式阈值过滤机制:
 *                     假设某个真实匹配 C[i][j] > cost_limit (超过阈值);
 *                     那么 C[i][j] > cost_limit/2 (惩罚区填充值);
 *                     算法会倾向于将行 i 分配到惩罚区的虚拟列 (成本更低);
 *                     结果 rowsol[i] 指向一个虚拟列 (>= n_cols);
 *                     后处理中, 虚拟列被映射为 -1, 表示"未匹配";
 *
 *                     反之, 如果 C[i][j] < cost_limit/2 (好的匹配);
 *                     算法会选择真实匹配 (成本更低);
 *
 *                     注意: 实际阈值效果约为 cost_limit/2, 而非精确的 cost_limit;
 *                     这是 ByteTrack 原版设计的特性, 保持了与参考实现一致;
 *
 *                 ============================================================
 *                 五. 数据流总览
 *                 ============================================================
 *
 *                 调用链:
 *                   ByteTracker::update()
 *                     -> matching::linear_assignment(cost_matrix, thresh)
 *                       -> lapjv(cost_matrix, rowsol, colsol, true, thresh)
 *                         -> 扩展矩阵为 (n_rows+n_cols) x (n_rows+n_cols) 方阵
 *                         -> lapjv_internal(n, cost_ptr, x, y)
 *                           -> _ccrrt_dense()  // 阶段 1: 列约简
 *                           -> _carr_dense()   // 阶段 2: 行约简 (最多 2 次)
 *                           -> _ca_dense()     // 阶段 3: 完整增广
 *                         -> 后处理: 虚拟列映射为 -1
 *                       -> 根据 rowsol/colsol 构建匹配结果
 *
 *                 输出:
 *                   - rowsol[i] >= 0: 行 i 匹配到列 rowsol[i];
 *                   - rowsol[i] == -1: 行 i 未匹配 (成本超过阈值);
 *                   - colsol[j] >= 0: 列 j 匹配到行 colsol[j];
 *                   - colsol[j] == -1: 列 j 未匹配;
 *
 *                 ============================================================
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __LAPJV__H__
#define __LAPJV__H__

#include <cfloat>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "types.hpp"

// ---- 大数值常量 ----
// 用于成本矩阵初始化和对偶变量初始化;
// 表示"不允许匹配"或"无穷大成本";
// 在 _ccrrt_dense 中用作 v[j] 和 v2 的初始值;
#define LARGE 1000000

// ---- 布尔常量 ----
// 用于 _ccrrt_dense 中的 unique 数组;
// unique[i] = TRUE 表示行 i 只被一列选中 (可以安全分配);
// unique[i] = FALSE 表示行 i 被多列选中 (存在冲突);
#if !defined TRUE
#define TRUE 1
#endif
#if !defined FALSE
#define FALSE 0
#endif

namespace tracker
{
namespace bytetrack
{

// ---- 安全分配内存的宏 ----
// 如果 malloc 失败, 直接返回 -1 (错误码);
// 用于在栈外分配临时工作数组 (如 free_rows, v, pred, cols, d 等);
#define LAP_NEW(x, t, n)                        \
    if ((x = (t*)malloc(sizeof(t) * (n))) == 0) \
    {                                           \
        return -1;                              \
    }

// ---- 安全释放内存的宏 ----
// 释放后将指针置零, 防止 double-free;
#define LAP_FREE(x) \
    if (x != 0)     \
    {               \
        free(x);    \
        x = 0;      \
    }

// ---- 交换两个索引值的宏 ----
// 用于增广路径中的分配翻转: swap(j, x[i]);
// 将行 i 的旧分配列与新分配列交换;
#define LAP_SWAP_INDICES(a, b) \
    {                          \
        int_t _temp_index = a; \
        a = b;                 \
        b = _temp_index;       \
    }

// ---- 类型别名 (与 lapjv 原版 C 代码保持一致) ----
// int_t:   有符号整数, 用于行/列索引 (允许 -1 表示未分配);
// uint_t:  无符号整数, 用于矩阵维度 n 和循环计数器;
// cost_t:  双精度浮点, 用于成本值和对偶变量 (避免单精度累积误差);
// boolean: char 类型布尔, 用于 unique 数组;
typedef signed int int_t;
typedef unsigned int uint_t;
typedef double cost_t;
typedef char boolean;

// =====================================================================
// 匿名命名空间: 封装 LAPJV 内部辅助函数;
// 这些函数不应被外部直接调用, 仅供 lapjv_internal() 使用;
// =====================================================================
namespace
{

/***
 * @description: 阶段 1 - 列约简和行转移 (Column Reduction and Row Transfer);
 *               LAPJV 算法的第一阶段;
 *               参考 bytetracker lapjv.cpp _ccrrt_dense;
 *
 *               算法步骤:
 *               (1) 对每列 j, 扫描所有行, 找到成本最小的行 i:
 *                     v[j] = min_i(cost[i][j])
 *                     y[j] = i (该列的最佳候选行)
 *
 *               (2) 检查每列的候选行是否唯一:
 *                   - 如果行 i 只被一列 j 选中 (unique), 则建立分配 x[i]=j;
 *                   - 如果行 i 被多列选中 (冲突), 只保留第一个分配,
 *                     其他列的 y[j] 设为 -1 (取消候选);
 *
 *               (3) 对已唯一分配的行 i (分配到列 j), 收紧列价格:
 *                     v[j] -= min_{j2!=j}(cost[i][j2] - v[j2])
 *                   这是为了让 v[j] 尽量大, 提升对偶目标值;
 *                   直觉: 行 i 被分配到列 j, 列 j 的"溢价"等于
 *                   行 i 选择次优列的额外成本;
 *
 *               (4) 收集所有 x[i] == -1 的行作为自由行;
 *
 * @param n         uint_t   : 方阵维度;
 * @param cost      cost_t** : 成本矩阵 (n x n);
 * @param free_rows int_t*   : 输出; 存储未分配行的索引数组;
 * @param x         int_t*   : 输出; 行分配: x[i] = 分配给行 i 的列 (-1 表示未分配);
 * @param y         int_t*   : 输出; 列分配: y[j] = 分配给列 j 的行 (-1 表示未分配);
 * @param v         cost_t*  : 输出; 列对偶变量 (列价格);
 * @return int_t : 未分配的行数 (传递给阶段 2);
 */
inline int_t _ccrrt_dense(const uint_t n, cost_t* cost[], int_t* free_rows, int_t* x, int_t* y, cost_t* v)
{
    int_t n_free_rows;
    boolean* unique;

    // ---- 初始化所有行为未分配, 所有列价格为 LARGE (将被更新为实际最小值) ----
    for (uint_t i = 0; i < n; i++)
    {
        x[i] = -1;     // 行 i 尚未分配到任何列;
        v[i] = LARGE;  // 列 i 的价格初始化为极大值 (后续被 min 替换);
        y[i] = 0;      // 列 i 的候选行初始化为 0 (后续被 argmin 替换);
    }

    // ---- 步骤 (1): 对每列找最小成本行, 建立列价格 v[j] 和候选行 y[j] ----
    // 遍历所有 (i, j), 如果 cost[i][j] < v[j], 则更新:
    //   v[j] = cost[i][j]  (该列的最低成本)
    //   y[j] = i           (提供最低成本的行)
    for (uint_t i = 0; i < n; i++)
    {
        for (uint_t j = 0; j < n; j++)
        {
            const cost_t c = cost[i][j];
            if (c < v[j])
            {
                v[j] = c;
                y[j] = static_cast<int_t>(i);
            }
        }
    }

    // ---- 步骤 (2): 检查唯一性并建立初步分配 ----
    // unique[i] 标记行 i 是否只被一列选中;
    // 从最后一列向前遍历, 为每个列的候选行建立分配;
    // 如果行 i 已经被之前的列分配, 则标记 unique[i]=FALSE, 取消当前列的候选;
    LAP_NEW(unique, boolean, n);
    memset(unique, TRUE, n);  // 初始假设所有行都是唯一的;
    {
        int_t j = static_cast<int_t>(n);
        do
        {
            j--;
            const int_t i = y[j];  // 列 j 的候选行;
            if (x[i] < 0)
            {
                // 行 i 尚未被分配, 建立分配 x[i]=j;
                x[i] = j;
            }
            else
            {
                // 行 i 已经被其他列占据, 存在冲突;
                // 标记行 i 为非唯一 (多列竞争同一行);
                unique[i] = FALSE;
                // 取消列 j 的候选 (该列需要在后续阶段重新分配);
                y[j] = -1;
            }
        } while (j > 0);
    }

    // ---- 步骤 (3)+(4): 收集自由行, 并对唯一分配的行收紧列价格 ----
    n_free_rows = 0;
    for (uint_t i = 0; i < n; i++)
    {
        if (x[i] < 0)
        {
            // 行 i 完全未被任何列选中, 加入自由行列表;
            free_rows[n_free_rows++] = static_cast<int_t>(i);
        }
        else if (unique[i])
        {
            // 行 i 被唯一分配到列 j;
            // 收紧列 j 的价格: v[j] -= min_{j2!=j}(cost[i][j2] - v[j2]);
            // 这等价于: 列 j 的"独占溢价" = 行 i 的次优选择与最优选择的成本差;
            const int_t j = x[i];
            cost_t min = LARGE;
            for (uint_t j2 = 0; j2 < n; j2++)
            {
                if (j2 == static_cast<uint_t>(j))
                    continue;
                const cost_t c = cost[i][j2] - v[j2];
                if (c < min)
                    min = c;
            }
            v[j] -= min;
        }
    }

    LAP_FREE(unique);
    return n_free_rows;
}

/***
 * @description: 阶段 2 - 增广行约简 (Augmenting Row Reduction);
 *               LAPJV 算法的第二阶段;
 *               参考 bytetracker lapjv.cpp _carr_dense;
 *
 *               这个阶段尝试通过局部的"交换操作"来分配自由行;
 *               不需要完整的 Dijkstra 搜索, 速度较快;
 *               通常在 1-2 轮后就能分配大部分自由行;
 *
 *               核心逻辑:
 *               对每个自由行 free_i:
 *               (1) 找到约简成本最小的列 j1 (最佳) 和 j2 (次佳):
 *                     v1 = cost[free_i][j1] - v[j1]
 *                     v2 = cost[free_i][j2] - v[j2]
 *
 *               (2) 计算新的列价格:
 *                     v1_new = v[j1] - (v2 - v1)
 *                   如果 v1_new < v[j1] (降价成功):
 *                     - 更新 v[j1] = v1_new;
 *                     - 将 free_i 分配到 j1;
 *                     - 如果 j1 原来已分配给 i0, 则 i0 变成新的自由行;
 *                       (i0 被放回当前队列前端, 在本轮立即重新处理);
 *
 *               (3) 如果降价失败 (v1_new >= v[j1]):
 *                     - 说明 j1 的价格不需要降低;
 *                     - 如果 j2 可用且 j1 已被 i0 占据:
 *                       改为将 free_i 分配到 j2, i0 加入下一轮自由行;
 *
 *               (4) rr_cnt 防死循环机制:
 *                     如果 rr_cnt >= current * n, 停止尝试交换;
 *                     直接将被挤出的行加入下一轮 (退化为简单模式);
 *
 * @param n            uint_t   : 方阵维度;
 * @param cost         cost_t** : 成本矩阵 (n x n);
 * @param n_free_rows  uint_t   : 输入的自由行数量;
 * @param free_rows    int_t*   : 输入/输出的自由行索引数组;
 * @param x            int_t*   : 行分配 (输入/输出);
 * @param y            int_t*   : 列分配 (输入/输出);
 * @param v            cost_t*  : 列对偶变量 (输入/输出);
 * @return int_t : 新的自由行数量 (传递给阶段 3);
 */
inline int_t _carr_dense(const uint_t n, cost_t* cost[], const uint_t n_free_rows, int_t* free_rows, int_t* x, int_t* y,
                         cost_t* v)
{
    // current: 当前正在处理的自由行在 free_rows 数组中的位置;
    // new_free_rows: 本轮处理后剩余的自由行数量 (存入 free_rows 前部);
    // rr_cnt: Row Reduction 操作计数器, 用于防死循环;
    uint_t current = 0;
    int_t new_free_rows = 0;
    uint_t rr_cnt = 0;

    while (current < n_free_rows)
    {
        int_t i0;           // 列 j1 的当前占据者 (可能被挤出);
        int_t j1, j2;       // j1: 最佳列, j2: 次佳列;
        cost_t v1, v2;      // v1: 最小约简成本, v2: 次小约简成本;
        cost_t v1_new;      // v1_new: 列 j1 的新价格;
        boolean v1_lowers;  // 是否降价成功;

        rr_cnt++;
        const int_t free_i = free_rows[current++];

        // ---- 对自由行 free_i, 找约简成本最小 (j1) 和次小 (j2) 的列 ----
        // 约简成本 = cost[free_i][j] - v[j];
        // 直觉: 列价格 v[j] 越高, 约简成本越低, 该列对 free_i "越有吸引力";
        j1 = 0;
        v1 = cost[free_i][0] - v[0];
        j2 = -1;
        v2 = LARGE;
        for (uint_t j = 1; j < n; j++)
        {
            const cost_t c = cost[free_i][j] - v[j];
            if (c < v2)
            {
                if (c >= v1)
                {
                    // c 介于 v1 和 v2 之间, 更新次小;
                    v2 = c;
                    j2 = static_cast<int_t>(j);
                }
                else
                {
                    // c 比 v1 还小, v1 变成次小, c 变成最小;
                    v2 = v1;
                    v1 = c;
                    j2 = j1;
                    j1 = static_cast<int_t>(j);
                }
            }
        }

        // ---- 尝试将 free_i 分配到 j1 ----
        // i0: 列 j1 当前被分配的行 (如果 y[j1] < 0 则 j1 为空闲列);
        i0 = y[j1];

        // 计算 j1 的新价格: 降低量 = 次优与最优的差;
        // 如果 v2 - v1 > 0, 说明 j1 对 free_i 有明显优势, 可以降价;
        v1_new = v[j1] - (v2 - v1);
        v1_lowers = v1_new < v[j1];

        // ---- 根据 rr_cnt 决定是否尝试交换 ----
        // rr_cnt < current * n: 正常模式, 尝试降价和交换;
        // rr_cnt >= current * n: 简单模式, 直接将被挤出行加入下一轮 (防死循环);
        if (rr_cnt < current * n)
        {
            if (v1_lowers)
            {
                // 降价成功: 更新列 j1 的价格;
                v[j1] = v1_new;
            }
            else if (i0 >= 0 && j2 >= 0)
            {
                // 降价失败, 但 j1 已被占据且 j2 可用:
                // 改为将 free_i 分配到 j2 (避免无意义地挤出 i0);
                j1 = j2;
                i0 = y[j2];
            }
            if (i0 >= 0)
            {
                if (v1_lowers)
                {
                    // 降价成功时, 被挤出的 i0 放回队列前端;
                    // (--current 使得 i0 在下一轮循环中立即被处理);
                    // 这样 i0 可以利用刚刚更新的 v[j1] 找到更好的分配;
                    free_rows[--current] = i0;
                }
                else
                {
                    // 降价失败时, 被挤出的 i0 加入"下一轮"自由行列表;
                    // (存入 free_rows 前部, 等待 _carr_dense 的下一次调用);
                    free_rows[new_free_rows++] = i0;
                }
            }
        }
        else
        {
            // ---- 简单模式 (防死循环) ----
            // 不尝试降价, 直接分配 free_i 到 j1;
            // 被挤出的 i0 无条件加入下一轮;
            if (i0 >= 0)
            {
                free_rows[new_free_rows++] = i0;
            }
        }

        // ---- 最终: 将 free_i 分配到 j1 ----
        x[free_i] = j1;
        y[j1] = free_i;
    }

    return new_free_rows;
}

/***
 * @description: 辅助函数 - 从 TODO 列中找到最小 d 值的列, 移入 SCAN 列表;
 *               参考 bytetracker lapjv.cpp _find_dense;
 *
 *               列序列 cols[] 的分区:
 *                 [0, lo)   = READY 列 (已完成扫描, d 值不会再变)
 *                 [lo, hi)  = SCAN  列 (d 值已确定, 待扫描)
 *                 [hi, n)   = TODO  列 (d 值可能还会减小)
 *
 *               本函数从 TODO 区间 [lo+1, n) 扫描, 找到 d 值最小的列;
 *               将所有 d 值等于最小值的列移入 SCAN 区间;
 *               (多个列可能有相同的最小 d 值, 全部移入 SCAN);
 *
 * @param n    uint_t  : 方阵维度;
 * @param lo   uint_t  : 当前 SCAN 列表的起始位置;
 * @param d    cost_t* : 距离数组 (d[j] = 从起始行到列 j 的最短约简成本);
 * @param cols int_t*  : 列序列 (按 READY/SCAN/TODO 分区排列);
 * @param y    int_t*  : 列分配 (未使用, 保留接口一致性);
 * @return uint_t : SCAN 列表的新上界 hi;
 */
inline uint_t _find_dense(const uint_t n, uint_t lo, cost_t* d, int_t* cols, int_t* y)
{
    // hi: SCAN 区间的上界, 初始化为 lo+1 (至少包含 cols[lo] 自身);
    uint_t hi = lo + 1;
    // mind: 当前已知的最小 d 值;
    cost_t mind = d[cols[lo]];

    // 遍历 TODO 区间 [hi, n), 找到 d 值最小的列;
    for (uint_t k = hi; k < n; k++)
    {
        int_t j = cols[k];
        if (d[j] <= mind)
        {
            if (d[j] < mind)
            {
                // 发现更小的 d 值, 重置 SCAN 区间;
                // hi = lo 表示清空之前的 SCAN 列 (它们的 d 值不够小);
                hi = lo;
                mind = d[j];
            }
            // 将列 j 交换到 SCAN 区间末尾;
            // (与 cols[hi] 交换位置, 然后 hi++);
            cols[k] = cols[hi];
            cols[hi++] = j;
        }
    }
    return hi;
}

/***
 * @description: 辅助函数 - 扫描 SCAN 列, 尝试松弛 TODO 列的距离;
 *               参考 bytetracker lapjv.cpp _scan_dense;
 *
 *               这是 Dijkstra 最短路径的核心松弛步骤:
 *               对 SCAN 区间 [lo, hi) 中的每个列 j:
 *                 - 获取列 j 的当前占据行 i = y[j];
 *                 - 计算从行 i 出发到所有 TODO 列 j' 的约简成本;
 *                 - 如果发现更短的路径, 更新 d[j'] 和 pred[j'];
 *                 - 如果某个 TODO 列 j' 是空闲的 (y[j'] < 0) 且 d[j'] 达到最小,
 *                   说明找到了增广路径终点, 立即返回 j';
 *
 *               松弛公式:
 *                 h = cost[i][j] - v[j] - d[j]
 *                   (h 是"通过列 j 的行 i"到达行 i 时的额外偏移量)
 *                 cred_ij = cost[i][j'] - v[j'] - h
 *                   (从起始行经过 ...-> j -> i -> j' 的总约简成本)
 *                 if cred_ij < d[j']:
 *                   d[j'] = cred_ij, pred[j'] = i;
 *
 * @param n    uint_t   : 方阵维度;
 * @param cost cost_t** : 成本矩阵 (n x n);
 * @param plo  uint_t*  : 输入/输出: SCAN 区间下界指针;
 * @param phi  uint_t*  : 输入/输出: SCAN 区间上界指针;
 * @param d    cost_t*  : 输入/输出: 距离数组;
 * @param cols int_t*   : 输入/输出: 列序列;
 * @param pred int_t*   : 输出: 前驱行数组 (pred[j] = 到达列 j 的前驱行);
 * @param y    int_t*   : 列分配 (y[j] = 占据列 j 的行, -1 表示空闲);
 * @param v    cost_t*  : 列对偶变量;
 * @return int_t : 找到的空闲列索引 (增广路径终点), -1 表示本轮未找到;
 */
inline int_t _scan_dense(const uint_t n, cost_t* cost[], uint_t* plo, uint_t* phi, cost_t* d, int_t* cols, int_t* pred,
                         int_t* y, cost_t* v)
{
    uint_t lo = *plo;
    uint_t hi = *phi;
    cost_t h, cred_ij;

    // ---- 遍历 SCAN 区间中的每个列 ----
    while (lo != hi)
    {
        // 取出 SCAN 区间的下一列 j;
        // lo++ 表示该列从 SCAN 移入 READY (处理完毕);
        int_t j = cols[lo++];
        // 获取列 j 的当前占据行 i;
        const int_t i = y[j];
        // mind: 当前列 j 的最短距离;
        const cost_t mind = d[j];
        // h: "经由列 j 的行 i 到达"所需的额外偏移量;
        // h = cost[i][j] - v[j] - mind;
        // 这个值在松弛所有 TODO 列时保持不变 (对行 i 而言);
        h = cost[i][j] - v[j] - mind;

        // ---- 尝试通过行 i 松弛所有 TODO 列 ----
        for (uint_t k = hi; k < n; k++)
        {
            j = cols[k];
            // cred_ij: 从起始行经由增广路径到达列 j 的约简成本;
            cred_ij = cost[i][j] - v[j] - h;
            if (cred_ij < d[j])
            {
                // 发现更短的路径到列 j, 更新距离和前驱;
                d[j] = cred_ij;
                pred[j] = i;
                if (cred_ij == mind)
                {
                    // d[j] 降到了与当前最小值相同;
                    if (y[j] < 0)
                    {
                        // 列 j 是空闲的 (未被任何行分配);
                        // 找到增广路径终点! 直接返回;
                        return j;
                    }
                    // 列 j 已被占据, 将其移入 SCAN 区间;
                    // (下一轮循环会通过 y[j] 这一行继续松弛);
                    cols[k] = cols[hi];
                    cols[hi++] = j;
                }
            }
        }
    }

    // 本轮 SCAN 全部处理完毕, 但未找到空闲列;
    // 更新 plo 和 phi, 让调用者知道 SCAN 区间已耗尽;
    *plo = lo;
    *phi = hi;
    return -1;
}

/***
 * @description: 查找增广路径 - 密集矩阵版本的 Dijkstra 最短路径算法;
 *               参考 bytetracker lapjv.cpp find_path_dense;
 *
 *               从自由行 start_i 出发, 在约简成本图上搜索一条增广路径;
 *               增广路径: start_i -> j1 -> i1 -> j2 -> i2 -> ... -> jk;
 *               其中 jk 是一个尚未被分配的列 (y[jk] < 0);
 *
 *               算法步骤:
 *               (1) 初始化: 对所有列 j, 设 d[j] = cost[start_i][j] - v[j];
 *                   (从 start_i 直接到列 j 的约简成本);
 *                   pred[j] = start_i (前驱行都是起始行);
 *
 *               (2) 循环:
 *                   a) 如果 SCAN 区间为空 (lo == hi), 调用 _find_dense()
 *                      从 TODO 中找到最小 d 值的列, 移入 SCAN;
 *                      同时检查这些列中是否有空闲列;
 *
 *                   b) 如果没有空闲列, 调用 _scan_dense()
 *                      通过 SCAN 列的占据行松弛 TODO 列;
 *                      如果松弛过程中发现空闲列, 返回该列;
 *
 *                   c) 重复直到找到空闲列 final_j;
 *
 *               (3) 调整对偶变量:
 *                   对所有 READY 列 k, v[k] += d[k] - mind;
 *                   这保证了新分配下互补松弛条件仍然成立;
 *
 * @param n       uint_t   : 方阵维度;
 * @param cost    cost_t** : 成本矩阵 (n x n);
 * @param start_i int_t    : 起始自由行索引;
 * @param y       int_t*   : 列分配 (y[j] = 占据列 j 的行);
 * @param v       cost_t*  : 列对偶变量 (输入/输出, 会被调整);
 * @param pred    int_t*   : 输出: 前驱行数组 (用于回溯增广路径);
 * @return int_t : 增广路径终点 (空闲列索引);
 */
inline int_t find_path_dense(const uint_t n, cost_t* cost[], const int_t start_i, int_t* y, cost_t* v, int_t* pred)
{
    // lo, hi: SCAN 区间的边界 [lo, hi);
    // final_j: 增广路径终点, -1 表示尚未找到;
    // n_ready: READY 列的数量 (在 _find_dense 时记录);
    uint_t lo = 0, hi = 0;
    int_t final_j = -1;
    uint_t n_ready = 0;
    int_t* cols;
    cost_t* d;

    LAP_NEW(cols, int_t, n);
    LAP_NEW(d, cost_t, n);

    // ---- 初始化: 设置列序列、距离和前驱 ----
    // cols[i] = i: 初始时所有列都在 TODO 区间;
    // d[i] = cost[start_i][i] - v[i]: 从 start_i 直接到列 i 的约简成本;
    // pred[i] = start_i: 所有列的前驱都是起始行;
    for (uint_t i = 0; i < n; i++)
    {
        cols[i] = static_cast<int_t>(i);
        pred[i] = start_i;
        d[i] = cost[start_i][i] - v[i];
    }

    // ---- 主循环: 反复查找和扫描, 直到找到空闲列 ----
    // 安全机制: 每次 _find_dense 至少将一列从 TODO 移入 SCAN,
    // 因此外层循环最多执行 n 次; 超过 n 次说明状态异常, 强制退出;
    while (final_j == -1)
    {
        // SCAN 区间为空时, 需要从 TODO 中补充新列;
        if (lo == hi)
        {
            // 安全检查: 如果所有列都已处理 (lo >= n), 不再有 TODO 列;
            // 防止 _find_dense 访问 cols[n] 导致越界 (未定义行为);
            if (lo >= n)
            {
                break;
            }

            // 记录当前 READY 列数量 (用于后续对偶变量调整);
            n_ready = lo;
            // _find_dense: 从 TODO 中找到最小 d 值的列, 移入 SCAN;
            // 返回新的 hi (SCAN 区间上界);
            hi = _find_dense(n, lo, d, cols, y);

            // 检查新加入 SCAN 的列中是否有空闲列;
            for (uint_t k = lo; k < hi; k++)
            {
                const int_t j = cols[k];
                if (y[j] < 0)
                {
                    // 找到空闲列, 增广路径终点确定;
                    final_j = j;
                }
            }
        }

        // SCAN 区间非空且尚未找到空闲列, 执行松弛;
        if (final_j == -1)
        {
            // _scan_dense: 通过 SCAN 列的占据行松弛 TODO 列;
            // 如果找到空闲列, 返回其索引; 否则返回 -1;
            final_j = _scan_dense(n, cost, &lo, &hi, d, cols, pred, y, v);
        }
    }

    // ---- 兜底搜索: 如果 Dijkstra 未找到空闲列, 暴力扫描 ----
    // 理论上不应触发 (n×n 问题中自由行必对应自由列);
    // 但扩展矩阵 + 浮点精度可能导致 Dijkstra 遗漏;
    if (final_j == -1)
    {
        for (uint_t k = 0; k < n; k++)
        {
            if (y[k] < 0)
            {
                final_j = static_cast<int_t>(k);
                break;
            }
        }
    }

    // ---- 调整对偶变量 (保证互补松弛条件) ----
    // 对所有 READY 列: v[j] += d[j] - mind;
    // mind 是 SCAN 区间第一列的 d 值 (即当前最短路径的长度);
    // 调整后, 新的对偶可行性和互补松弛条件在更新后的分配下仍然成立;
    // 安全检查: 仅当 lo < n 时执行 (兜底搜索退出时 lo 可能 >= n);
    if (lo < n)
    {
        const cost_t mind = d[cols[lo]];
        for (uint_t k = 0; k < n_ready; k++)
        {
            const int_t j = cols[k];
            v[j] += d[j] - mind;
        }
    }

    LAP_FREE(cols);
    LAP_FREE(d);

    return final_j;
}

/***
 * @description: 阶段 3 - 增广 (Augmentation);
 *               LAPJV 算法的最后阶段;
 *               参考 bytetracker lapjv.cpp _ca_dense;
 *
 *               对阶段 2 后仍未分配的自由行, 执行完整的 Dijkstra 式增广路径搜索;
 *               这是算法的"兜底"步骤, 保证所有自由行都能找到分配;
 *
 *               对每个自由行 free_i:
 *               (1) 调用 find_path_dense() 找到一条从 free_i 到某个空闲列 j 的增广路径;
 *                   增广路径通过 pred[] 数组编码:
 *                     pred[j] = i 表示"列 j 是从行 i 转移过来的";
 *
 *               (2) 沿增广路径回溯, 翻转分配:
 *                     j = 增广路径终点 (空闲列);
 *                     while (i != free_i):
 *                       i = pred[j]       // 列 j 的前驱行;
 *                       y[j] = i          // 将列 j 分配给行 i;
 *                       swap(j, x[i])     // 行 i 的旧列与新列交换;
 *                     最终: free_i 获得一个列, 路径上的其他行也重新调整;
 *
 *               (3) 安全机制: 如果回溯步数 k >= n, 强制终止;
 *                   (理论上不应该发生, 但防止数据异常导致的无限循环);
 *
 * @param n            uint_t   : 方阵维度;
 * @param cost         cost_t** : 成本矩阵 (n x n);
 * @param n_free_rows  uint_t   : 自由行数量;
 * @param free_rows    int_t*   : 自由行索引数组;
 * @param x            int_t*   : 行分配 (输入/输出);
 * @param y            int_t*   : 列分配 (输入/输出);
 * @param v            cost_t*  : 列对偶变量 (输入/输出);
 * @return int_t : 0 表示成功;
 */
inline int_t _ca_dense(const uint_t n, cost_t* cost[], const uint_t n_free_rows, int_t* free_rows, int_t* x, int_t* y,
                       cost_t* v)
{
    // pred[j]: 前驱行数组, pred[j] = i 表示增广路径中列 j 是从行 i 转移过来的;
    int_t* pred;

    LAP_NEW(pred, int_t, n);

    // ---- 处理每个自由行 ----
    // pfree_i 遍历 free_rows 数组;
    for (int_t* pfree_i = free_rows; pfree_i < free_rows + n_free_rows; pfree_i++)
    {
        int_t i = -1, j;
        uint_t k = 0;  // 回溯步数计数器 (安全机制);

        // ---- 查找增广路径 ----
        // find_path_dense 返回增广路径的终点列 j (一个空闲列);
        // 同时 pred[] 数组记录了路径信息;
        j = find_path_dense(n, cost, *pfree_i, y, v, pred);

        // ---- 沿增广路径回溯, 翻转分配 ----
        // 从终点列 j 开始, 沿 pred 回溯到起始行 *pfree_i;
        // 每一步: 将列 j 的分配从旧行转移到新行;
        //
        // 示例: 假设路径为 free_i -> j1 -> i1 -> j2 -> i2 -> j3 (空闲)
        //   第 1 步: i=pred[j3]=i2, y[j3]=i2, swap(j3, x[i2]) => j=x[i2] 的旧值=j2
        //   第 2 步: i=pred[j2]=i1, y[j2]=i1, swap(j2, x[i1]) => j=x[i1] 的旧值=j1
        //   第 3 步: i=pred[j1]=free_i, y[j1]=free_i, swap(j1, x[free_i])
        //   此时 i == free_i, 循环结束;
        while (i != *pfree_i)
        {
            i = pred[j];                // 获取列 j 的前驱行;
            y[j] = i;                   // 将列 j 分配给行 i;
            LAP_SWAP_INDICES(j, x[i]);  // 交换: 行 i 的旧列成为下一个要处理的列;
            k++;
            if (k >= n)
            {
                // 安全终止: 正常情况下路径长度不超过 n;
                break;
            }
        }
    }

    LAP_FREE(pred);
    return 0;
}

}  // namespace

/***
 * @description: LAPJV 核心算法入口 (稠密矩阵版本);
 *               解决 n x n 线性分配问题 (Linear Assignment Problem);
 *               参考 bytetracker lapjv.cpp lapjv_internal;
 *
 *               算法流程:
 *               (1) 调用 _ccrrt_dense(): 阶段 1 - 列约简 + 行转移;
 *                   初始化列价格 v[j], 建立初步分配;
 *                   返回未分配的行数 ret;
 *
 *               (2) 如果 ret > 0 (还有自由行):
 *                   调用 _carr_dense(): 阶段 2 - 增广行约简;
 *                   最多执行 2 次 (经验值, 足以处理大多数冲突);
 *                   每次迭代后 ret 更新为剩余自由行数;
 *
 *               (3) 如果 ret > 0 (仍有自由行):
 *                   调用 _ca_dense(): 阶段 3 - 完整增广;
 *                   使用 Dijkstra 最短路径为每个自由行找到增广路径;
 *                   保证所有行都能分配到列;
 *
 * @param n      uint_t   : 方阵维度 (n x n);
 * @param cost   cost_t** : 成本矩阵指针数组 (cost[i][j] = 行 i 分配到列 j 的成本);
 * @param x      int_t*   : 输出: 行分配 (x[i] = 行 i 分配到的列);
 * @param y      int_t*   : 输出: 列分配 (y[j] = 列 j 分配到的行);
 * @return int_t : 0 表示成功, -1 表示内存分配失败;
 */
inline int_t lapjv_internal(const uint_t n, cost_t* cost[], int_t* x, int_t* y)
{
    int_t ret;
    int_t* free_rows = NULL;  // 自由行索引数组 (最多 n 个);
    cost_t* v = NULL;         // 列对偶变量数组 (列价格);

    LAP_NEW(free_rows, int_t, n);
    LAP_NEW(v, cost_t, n);

    // ---- 阶段 1: 列约简 + 行转移 ----
    // 返回值 ret = 未分配的行数;
    ret = _ccrrt_dense(n, cost, free_rows, x, y, v);

    // ---- 阶段 2: 增广行约简 (最多执行 2 次) ----
    // 每次 _carr_dense 返回新的自由行数;
    // 如果 ret 降为 0, 所有行已分配, 无需进入阶段 3;
    int_t i = 0;
    while (ret > 0 && i < 2)
    {
        ret = _carr_dense(n, cost, static_cast<uint_t>(ret), free_rows, x, y, v);
        i++;
    }

    // ---- 阶段 3: 完整增广 ----
    // 对剩余的自由行执行 Dijkstra 式增广路径搜索;
    // 保证所有行都获得分配;
    if (ret > 0)
    {
        ret = _ca_dense(n, cost, static_cast<uint_t>(ret), free_rows, x, y, v);
    }

    LAP_FREE(v);
    LAP_FREE(free_rows);

    return ret;
}

/***
 * @description: 线性分配求解 (对外接口, 带矩阵扩展和成本限制);
 *               参考 bytetracker utils.cpp BYTETracker::lapjv;
 *
 *               这是 LAPJV 算法的高层包装函数;
 *               处理非方阵输入, 成本阈值过滤, 以及数据类型转换;
 *
 *               ---- 矩阵扩展策略 (核心机制) ----
 *
 *               问题: lapjv_internal 只能处理 n x n 方阵;
 *               但目标跟踪中, 轨迹数 (n_rows) 通常不等于检测数 (n_cols);
 *
 *               解决方案:
 *               将 n_rows x n_cols 矩阵扩展为 N x N 方阵;
 *               其中 N = n_rows + n_cols;
 *
 *               扩展后矩阵的四个区域:
 *
 *                 ┌──────────────────────┬──────────────────────┐
 *                 │   真实区域            │   惩罚区域 (行溢出)   │
 *                 │   (n_rows x n_cols)   │   (n_rows x n_rows)  │
 *                 │   cost[i][j]          │   cost_limit / 2.0   │
 *                 ├──────────────────────┼──────────────────────┤
 *                 │   惩罚区域 (列溢出)   │   零成本区域          │
 *                 │   (n_cols x n_cols)   │   (n_cols x n_rows)  │
 *                 │   cost_limit / 2.0    │   0.0                │
 *                 └──────────────────────┴──────────────────────┘
 *
 *               各区域含义:
 *               - 左上角 (真实区域): 原始的成本矩阵, 表示真实的匹配成本;
 *               - 右上角 (行惩罚区): 真实行匹配到虚拟列的成本;
 *                 如果行 i 的所有真实匹配成本都很高, 算法会"放弃",
 *                 将行 i 分配到这里 (成本 = cost_limit/2), 表示该行未匹配;
 *               - 左下角 (列惩罚区): 虚拟行匹配到真实列的成本;
 *                 如果列 j 的所有真实匹配成本都很高, 算法会将虚拟行分配到列 j;
 *               - 右下角 (零成本区): 虚拟行与虚拟列之间的匹配;
 *                 成本为 0, 让多余的虚拟行列可以无成本地相互匹配;
 *
 *               后处理:
 *               - 如果 rowsol[i] >= n_cols, 说明行 i 被分配到了虚拟列;
 *                 将 rowsol[i] 设为 -1, 表示"未匹配";
 *               - 如果 colsol[j] >= n_rows, 说明列 j 被分配到了虚拟行;
 *                 将 colsol[j] 设为 -1, 表示"未匹配";
 *
 * @param cost        const std::vector<std::vector<float32>>& : 原始成本矩阵 (n_rows x n_cols);
 * @param rowsol      std::vector<int32>& : 输出: 行解 (rowsol[i] = 行 i 匹配的列, -1 = 未匹配);
 * @param colsol      std::vector<int32>& : 输出: 列解 (colsol[j] = 列 j 匹配的行, -1 = 未匹配);
 * @param extend_cost bool : 是否强制扩展为方阵 (非方阵时自动启用);
 * @param cost_limit  float32 : 成本上限阈值 (用于隐式过滤高成本匹配);
 * @param return_cost bool : 是否返回总匹配成本 (默认 true);
 * @return float64 : 总匹配成本 (仅当 return_cost=true 时有意义);
 */
inline float64 lapjv(const std::vector<std::vector<float32>>& cost,        //
                     std::vector<int32>& rowsol,                           //
                     std::vector<int32>& colsol,                           //
                     bool extend_cost = false,                             //
                     float32 cost_limit = static_cast<float32>(LONG_MAX),  //
                     bool return_cost = true)
{
    // ---- 获取成本矩阵的尺寸 ----
    int32 n_rows = static_cast<int32>(cost.size());
    if (n_rows == 0)
    {
        rowsol.clear();
        colsol.clear();
        return 0.0;
    }
    int32 n_cols = static_cast<int32>(cost[0].size());
    if (n_cols == 0)
    {
        rowsol.clear();
        colsol.clear();
        return 0.0;
    }

    // 调整解向量大小;
    rowsol.resize(n_rows);
    colsol.resize(n_cols);

    // n: 算法内部使用的方阵大小;
    // 如果 n_rows == n_cols 且不需要扩展, 直接使用 n_rows;
    // 否则需要扩展为 (n_rows + n_cols);
    int32 n = 0;
    if (n_rows == n_cols)
    {
        n = n_rows;
    }
    else
    {
        // 非方阵: 强制启用扩展, 无论 extend_cost 参数的值;
        if (!extend_cost)
        {
            extend_cost = true;
        }
    }

    // 创建成本矩阵的工作副本 (避免修改原始数据);
    std::vector<std::vector<float32>> cost_c;
    cost_c.assign(cost.begin(), cost.end());

    // ---- 矩阵扩展处理 ----
    // 触发条件: extend_cost=true 或 cost_limit < LONG_MAX;
    // 两种情况都需要将矩阵扩展为 (n_rows+n_cols) x (n_rows+n_cols) 方阵;
    std::vector<std::vector<float32>> cost_c_extended;

    if (extend_cost || cost_limit < static_cast<float32>(LONG_MAX))
    {
        // 扩展为 (n_rows + n_cols) x (n_rows + n_cols) 方阵;
        n = n_rows + n_cols;
        cost_c_extended.resize(n);
        for (int32 i = 0; i < n; i++)
        {
            cost_c_extended[i].resize(n);
        }

        // ---- 根据 cost_limit 选择填充策略 ----
        if (cost_limit < static_cast<float32>(LONG_MAX))
        {
            // 有成本阈值: 整个扩展矩阵先填充 cost_limit / 2.0;
            // 这实现了隐式阈值过滤:
            //   真实匹配成本 < cost_limit/2 => 优于惩罚区, 算法选择真实匹配;
            //   真实匹配成本 > cost_limit/2 => 劣于惩罚区, 算法选择虚拟匹配 (即"不匹配");
            for (int32 i = 0; i < n; i++)
            {
                for (int32 j = 0; j < n; j++)
                {
                    cost_c_extended[i][j] = cost_limit / 2.0f;
                }
            }
        }
        else
        {
            // 无成本阈值: 使用原始矩阵最大值 + 1 作为填充;
            // 确保虚拟匹配的成本总是高于任何真实匹配;
            float32 cost_max = -1.0f;
            for (int32 i = 0; i < static_cast<int32>(cost_c.size()); i++)
            {
                for (int32 j = 0; j < static_cast<int32>(cost_c[i].size()); j++)
                {
                    if (cost_c[i][j] > cost_max)
                    {
                        cost_max = cost_c[i][j];
                    }
                }
            }
            for (int32 i = 0; i < n; i++)
            {
                for (int32 j = 0; j < n; j++)
                {
                    cost_c_extended[i][j] = cost_max + 1.0f;
                }
            }
        }

        // ---- 右下角 (虚拟行 x 虚拟列) 设为 0 ----
        // 允许多余的虚拟行与虚拟列之间进行零成本匹配;
        // 这些匹配不影响实际的跟踪结果;
        for (int32 i = n_rows; i < n; i++)
        {
            for (int32 j = n_cols; j < n; j++)
            {
                cost_c_extended[i][j] = 0.0f;
            }
        }

        // ---- 左上角: 将原始成本矩阵复制到扩展矩阵 ----
        // 覆盖左上角区域的填充值, 恢复真实成本;
        for (int32 i = 0; i < n_rows; i++)
        {
            for (int32 j = 0; j < n_cols; j++)
            {
                cost_c_extended[i][j] = cost_c[i][j];
            }
        }

        // 用扩展后的矩阵替换工作副本;
        cost_c.clear();
        cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
    }

    // ---- 数据类型转换: vector<vector<float>> -> double** ----
    // lapjv_internal 使用 C 风格指针数组 (cost_t** = double**);
    // 需要将 C++ vector 转换为原始指针;
    cost_t** cost_ptr = new cost_t*[n];
    for (int32 i = 0; i < n; i++)
    {
        cost_ptr[i] = new cost_t[n];
    }
    for (int32 i = 0; i < n; i++)
    {
        for (int32 j = 0; j < n; j++)
        {
            cost_ptr[i][j] = static_cast<cost_t>(cost_c[i][j]);
        }
    }

    // ---- 分配解向量内存 (C 风格) ----
    // x_c[i]: 行 i 分配到的列;
    // y_c[j]: 列 j 分配到的行;
    int_t* x_c = new int_t[n];
    int_t* y_c = new int_t[n];

    // ---- 调用 LAPJV 核心算法 ----
    int_t ret = lapjv_internal(static_cast<uint_t>(n), cost_ptr, x_c, y_c);
    (void)ret;  // 忽略返回值 (失败时结果不可靠, 但不会崩溃);

    // 总匹配成本 (仅当 return_cost=true 时计算);
    float64 opt = 0.0;

    // ---- 后处理: 将扩展矩阵的结果映射回原始尺寸 ----
    if (n != n_rows)
    {
        // 使用了扩展矩阵, 需要过滤虚拟匹配;

        // 将虚拟列/虚拟行的匹配标记为 -1 (未匹配);
        // x_c[i] >= n_cols: 行 i 被分配到虚拟列 => 未匹配;
        // y_c[j] >= n_rows: 列 j 被分配到虚拟行 => 未匹配;
        for (int32 i = 0; i < n; i++)
        {
            if (x_c[i] >= n_cols)
                x_c[i] = -1;
            if (y_c[i] >= n_rows)
                y_c[i] = -1;
        }

        // 将结果复制到输出参数 (只取真实行和真实列的部分);
        for (int32 i = 0; i < n_rows; i++)
        {
            rowsol[i] = static_cast<int32>(x_c[i]);
        }
        for (int32 i = 0; i < n_cols; i++)
        {
            colsol[i] = static_cast<int32>(y_c[i]);
        }

        // 计算总匹配成本 (只计算真实匹配的成本);
        if (return_cost)
        {
            for (int32 i = 0; i < n_rows; i++)
            {
                if (rowsol[i] != -1)
                {
                    opt += cost_ptr[i][rowsol[i]];
                }
            }
        }
    }
    else if (return_cost)
    {
        // 未使用扩展矩阵 (方阵): 直接复制结果并计算成本;
        for (int32 i = 0; i < n_rows; i++)
        {
            rowsol[i] = static_cast<int32>(x_c[i]);
        }
        for (int32 i = 0; i < n_cols; i++)
        {
            colsol[i] = static_cast<int32>(y_c[i]);
        }
        for (int32 i = 0; i < n_rows; i++)
        {
            opt += cost_ptr[i][rowsol[i]];
        }
    }
    else
    {
        // 未使用扩展矩阵且不需要成本: 只复制结果;
        for (int32 i = 0; i < n_rows; i++)
        {
            rowsol[i] = static_cast<int32>(x_c[i]);
        }
        for (int32 i = 0; i < n_cols; i++)
        {
            colsol[i] = static_cast<int32>(y_c[i]);
        }
    }

    // ---- 释放所有动态分配的内存 ----
    for (int32 i = 0; i < n; i++)
    {
        delete[] cost_ptr[i];
    }
    delete[] cost_ptr;
    delete[] x_c;
    delete[] y_c;

    return opt;
}

// ---- 清理 lapjv 内部使用的宏定义 ----
// 这些宏仅在本文件内使用, 防止污染外部命名空间;
#undef LAP_NEW
#undef LAP_FREE
#undef LAP_SWAP_INDICES

}  // namespace bytetrack
}  // namespace tracker

#endif  // !__LAPJV__H__
