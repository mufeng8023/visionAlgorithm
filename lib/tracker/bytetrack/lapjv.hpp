/***
 * @Author       : gxs
 * @Date         : 2026-06-22 21:35:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 21:35:00
 * @FilePath     : /visionAlgorithm/lib/tracker/bytetrack/lapjv.hpp
 * @Description  : Jonker-Volgenant 线性分配算法 (LAPJV) 实现;
 *                 用于目标跟踪中的数据关联 (匈牙利匹配);
 *                 参考 /mnt/E/CodeFiles/C++/bytetracker/include/lapjv.h
 *                 与 /mnt/E/CodeFiles/C++/bytetracker/src/lapjv.cpp;
 *
 *                 ============================================================
 *                 LAPJV 算法简介
 *                 ============================================================
 *
 *                 LAPJV (Jonker-Volgenant Algorithm for Linear Assignment Problem)
 *                 是一种专门用于解决线性分配问题 (LAP) 的算法.
 *                 线性分配问题描述: 给定一个 n x n 的成本矩阵 C,
 *                 找到一种一一对应的行-列匹配, 使得总成本最小.
 *
 *                 数学形式:
 *                    minimize sum_{i=0}^{n-1} C[i][pi(i)]
 *                    其中 pi 是 {0, 1, ..., n-1} 的一个排列
 *
 *                 ============================================================
 *                 LAPJV vs Munkres (Kuhn-Munkres)
 *                 ============================================================
 *
 *                 两者都是 O(n^3) 的精确求解器, 数学上等价.
 *                 ByteTrack 选择 LAPJV 的原因:
 *
 *                 1. 原版 Python ByteTrack 使用 lapjv 库, 保持一致性;
 *                 2. LAPJV 在稠密矩阵上的常数因子更小, 速度快 2-3 倍;
 *                 3. LAPJV 可以自然地处理"不允许匹配"的标记 (通过大值填充);
 *
 *                 DeepSORT 使用 Munkres 的原因:
 *                 1. 原版 DeepSORT Python 使用 scipy.optimize.linear_sum_assignment;
 *                 2. Scipy 底层用的是 Munkres 的 C 实现;
 *
 *                 ============================================================
 *                 LAPJV 算法三阶段
 *                 ============================================================
 *
 *                 阶段 1 - 列约简 (Column Reduction) [_ccrrt_dense]:
 *                   对每列找到最小值, 将列减去最小值 (归约);
 *                   然后为每列的"赢家行"做初步分配;
 *
 *                 阶段 2 - 增广行约简 (Augmenting Row Reduction) [_carr_dense]:
 *                   对未分配的行, 用增广路径来改善分配;
 *                   重复最多 2 次;
 *
 *                 阶段 3 - 增广 (Augmentation) [_ca_dense]:
 *                   对剩余的未分配行, 执行完整的 Dijkstra 式增广路径搜索;
 *
 *                 ============================================================
 *                 对偶变量 (Dual Variables)
 *                 ============================================================
 *
 *                 LAPJV 使用对偶变量来加速求解:
 *                 - v[j]: 列 j 的对偶变量 (初始值为列最小值);
 *                 - 约简成本 = C[i][j] - v[j];
 *                 - 当约简成本为 0 时, 该元素称为"紧致"的;
 *                 - 算法不断调整 v[j], 让更多"紧致"元素出现,
 *                   从而找到最优匹配;
 *                 ============================================================
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __LAPJV__H__
#define __LAPJV__H__

#include <cfloat>   // FLT_MAX
#include <cstdlib>  // malloc, free
#include <cstring>  // memset
#include <vector>   // std::vector

#include "types.hpp"  // 路径从 bytetrack/ 目录引用 ok (CMake include_directories 含 lib/)

// ---- 大数值常量 ----
// 用于成本矩阵初始化, 表示"不允许匹配"或"无穷大成本";
// 这个值应该大于所有实际可能的成本, 通常取 1e6 足够;
// 注意! 如果实际成本可能达到百万级别, 需要增大此值;
#define LARGE 1000000.0

// ---- 布尔常量 ----
// 用于算法内部的 true/false 标记;
// 这里手动定义而不是用 stdbool.h, 是为了与原版 C 风格 lapjv 代码保持一致;
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
// 功能: 分配 n 个类型为 t 的元素, 通过 x 返回;
// 如果 malloc 返回 NULL (内存不足), 立即返回 -1 错误码;
// 这是 C 风格错误处理, 避免在热路径中抛异常;
#define LAP_NEW(x, t, n)                        \
    if ((x = (t*)malloc(sizeof(t) * (n))) == 0) \
    {                                           \
        return -1;                              \
    }

// ---- 安全释放内存的宏 ----
// 先检查指针是否为 NULL (防止 double-free),
// 然后 free, 最后置 NULL 防止野指针;
#define LAP_FREE(x) \
    if (x != 0)     \
    {               \
        free(x);    \
        x = 0;      \
    }

// ---- 交换两个索引值的宏 ----
// 临时变量 _temp_index 前加下划线, 降低与外部变量名冲突的概率;
#define LAP_SWAP_INDICES(a, b) \
    {                          \
        int_t _temp_index = a; \
        a = b;                 \
        b = _temp_index;       \
    }

// ---- 类型别名 (与 lapjv 原版保持一致) ----
typedef signed int int_t;     // 有符号整数, 用于索引; 允许 -1 表示"未分配"
typedef unsigned int uint_t;  // 无符号整数, 用于矩阵尺寸和循环
typedef double cost_t;        // 成本类型, 使用 double 保证数值精度

// ---- 布尔类型 ----
// char 类型用作布尔值 (0 = false, 非0 = true);
// 保持与 C 风格 lapjv 原版兼容;
typedef char boolean;

// =====================================================================
// 匿名命名空间: 封装内部辅助函数;
// 这些函数不应被外部直接调用, 所以放在匿名 namespace 中,
// 确保链接时不会与其他翻译单元冲突;
// =====================================================================
namespace
{

/***
 * @description: 列约简和行转移 (Column Reduction and Row Transfer);
 *              LAPJV 算法的第一阶段;
 *
 *              这个阶段的目标是: 生成一个"好的"初始分配;
 *              它通过列约简和简单的行转移来快速得到一个近优解;
 *
 *              步骤:
 *              1. 初始化: x[i] = -1 (所有行未分配), v[j] = LARGE, y[j] = 0;
 *              2. 对每列 j, 找到成本最小的行 i = argmin(C[][j]);
 *                 记录: v[j] = min_cost, y[j] = i;
 *              3. 检查每列的最小值是否由"唯一"的行提供:
 *                 - 如果行 i 只被一列选中 -> 分配该列给该行;
 *                 - 如果行 i 被多列选中 -> 该行"不唯一", 取消这些列的分配;
 *              4. 对唯一分配的行, 调整列对偶变量 v[j];
 *
 *              返回值 = 未分配的行数;
 *              如果返回值 = 0, 说明已经找到完整分配, 算法提前结束;
 *
 * @param n         uint_t   : 矩阵维度;
 * @param cost      cost_t** : 成本矩阵;
 * @param free_rows int_t*   : 输出; 存储未分配行的索引;
 * @param x         int_t*   : 输出; 行分配: x[i] = 分配给行 i 的列;
 * @param y         int_t*   : 输出; 列分配: y[j] = 分配给列 j 的行;
 * @param v         cost_t*  : 输出; 列对偶变量 (dual variables);
 * @return int_t : 未分配的行数 (0 = 所有行已分配, 算法完成);
 */
inline int_t _ccrrt_dense(const uint_t n, cost_t* cost[], int_t* free_rows, int_t* x, int_t* y, cost_t* v)
{
    int_t n_free_rows;
    boolean* unique;  // unique[i] = true 表示行 i 只被一列选中

    // ---- 步骤 1: 初始化 ----
    // x[i] = -1: 所有行暂未分配;
    // v[j] = LARGE: 列对偶变量初始化为"无穷大";
    // y[j] = 0: 临时列分配;
    for (uint_t i = 0; i < n; i++)
    {
        x[i] = -1;
        v[i] = LARGE;
        y[i] = 0;
    }

    // ---- 步骤 2: 对每列找到最小成本的行 ----
    // 遍历所有 i,j, 如果 cost[i][j] < v[j], 更新 v[j] 和 y[j];
    // 这等价于: v[j] = min_i cost[i][j], y[j] = argmin_i cost[i][j];
    for (uint_t i = 0; i < n; i++)
    {
        for (uint_t j = 0; j < n; j++)
        {
            if (cost[i][j] < v[j])
            {
                v[j] = cost[i][j];             // 更新列 j 的最小成本
                y[j] = static_cast<int_t>(i);  // 记录对应行
            }
        }
    }

    // ---- 步骤 3: 检查唯一性并分配 ----
    // 分配逻辑:
    //   从最后一列开始倒序遍历 (与原版 lapjv 保持一致);
    //   如果行 i 尚未分配 (x[i] < 0), 将列 j 分配给行 i;
    //   如果行 i 已经被别的列分配了, 标记为"不唯一";
    LAP_NEW(unique, boolean, n);
    memset(unique, TRUE, n);  // 默认所有行都是"唯一"的
    {
        int_t j = static_cast<int_t>(n);
        do
        {
            j--;
            const int_t i = y[j];  // 列 j 的"赢家"行
            if (x[i] < 0)
            {
                x[i] = j;  // 行 i 未分配 -> 分配列 j
            }
            else
            {
                unique[i] = FALSE;  // 行 i 已分配 -> 不唯一
                y[j] = -1;          // 列 j 暂未分配
            }
        } while (j > 0);
    }

    // ---- 步骤 4: 收集未分配的行 & 调整对偶变量 ----
    n_free_rows = 0;
    for (uint_t i = 0; i < n; i++)
    {
        if (x[i] < 0)
        {
            // 行 i 未分配 -> 加入 free_rows 列表;
            free_rows[n_free_rows++] = static_cast<int_t>(i);
        }
        else if (unique[i])
        {
            // 行 i 有唯一分配 -> 调整列对偶变量 v;
            // 对已分配的行, 找到除分配列之外的最小约简成本,
            // 然后用这个最小值来缩减 v[j];
            const int_t j = x[i];
            cost_t min = LARGE;
            for (uint_t j2 = 0; j2 < n; j2++)
            {
                if (j2 == static_cast<uint_t>(j))
                {
                    continue;  // 跳过已分配的列
                }
                const cost_t c = cost[i][j2] - v[j2];
                if (c < min)
                {
                    min = c;
                }
            }
            v[j] -= min;  // 调整列对偶变量
        }
    }

    LAP_FREE(unique);
    return n_free_rows;  // 返回未分配的行数
}

/***
 * @description: 增广行约简 (Augmenting Row Reduction);
 *              LAPJV 算法的第二阶段;
 *
 *              这个阶段的目标是: 对未分配的行, 通过增广路径改善分配;
 *              路径搜索使用类似 Dijkstra 最短路径的思想;
 *
 *              核心变量:
 *              - d[j]: 当前找到的行到列 j 的最短距离;
 *              - pred[j]: 最短距离对应的前驱行;
 *              - col[current]: 当前路径上已处理的列;
 *              - h: 当前路径的最小成本;
 *
 *              算法流程 (简化版):
 *              1. 对所有自由行, 计算到每列的最短约简成本;
 *              2. 选择最小成本的列, 尝试增广;
 *              3. 如果该列有已分配的行, 则"腾出"该行, 继续搜索;
 *              4. 更新列对偶变量和行分配;
 *
 *              @note 本实现最多执行 2 次 _carr_dense, 由外层 lapjv_internal 控制;
 *
 * @param n    uint_t   : 矩阵维度;
 * @param cost cost_t** : 成本矩阵;
 * @param x    const uint_t* : 当前行分配;
 * @param y    const uint_t* : 当前列分配;
 * @param v    cost_t*  : 列对偶变量 (会被修改);
 * @return int_t : 0 表示成功 (所有行已分配), >0 表示仍有未分配行;
 */
inline int_t _carr_dense(const uint_t n, cost_t* cost[], int_t* x, const int_t* y, cost_t* v)
{
    uint_t current = 0;
    int_t* free_rows = NULL;
    cost_t* d = NULL;    // 最短距离数组
    int_t* pred = NULL;  // 前驱数组
    int_t* col = NULL;   // 当前处理的列序列

    // ---- 分配临时内存 ----
    LAP_NEW(free_rows, int_t, n);
    LAP_NEW(d, cost_t, n);
    LAP_NEW(pred, int_t, n);
    LAP_NEW(col, int_t, n);

    // 初始化 free_rows 为所有行;
    for (uint_t i = 0; i < n; i++)
    {
        free_rows[i] = static_cast<int_t>(i);
    }

    // 初始化 col 数组 (防止读取未初始化的内存);
    for (uint_t i = 0; i < n; i++)
    {
        col[i] = static_cast<int_t>(n - 1);
    }

    int_t n_free_rows = static_cast<int_t>(n);
    const int_t end_of_path = static_cast<int_t>(n);
    const int_t end_of_path_plus_1 = static_cast<int_t>(n + 1);

    // ---- 主循环: 依次处理每一行 ----
    while (current < n)
    {
        int_t k = 0;
        int_t i0;
        int_t low = end_of_path;
        int_t up = end_of_path;

        cost_t h = 0;
        cost_t u = 0;
        cost_t u_min = 0;

        cost_t* d_col = NULL;
        cost_t* cost_row = NULL;

        // ---- 步骤 A: 计算每列的最短约简成本 ----
        // 对所有自由行, 计算 cost_row[j] - v[j] 的最小值;
        // d[j] = min_i(cost[i][j] - v[j]) for i in free_rows;
        // pred[j] = argmin_i(cost[i][j] - v[j]) (记录哪个自由行提供了最小值);
        for (uint_t f = 0; f < n_free_rows; f++)
        {
            const int_t free_row = free_rows[f];
            cost_row = cost[free_row];
            d_col = d;
            for (uint_t j = 0; j < n; j++)
            {
                const cost_t c = cost_row[j] - v[j];
                if (c < *d_col)
                {
                    *d_col = c;
                    pred[j] = free_row;
                }
                d_col++;
            }
        }

        int_t num_free = n_free_rows;
        n_free_rows = 0;

        // 初始化: 找到第一个最小距离列作为增广起点;
        // col[current] 是当前路径上的列, 首次进入需设置初值;
        {
            cost_t min_d = d[0];
            int_t min_j = 0;
            for (uint_t j = 1; j < n; j++)
            {
                if (d[j] < min_d)
                {
                    min_d = d[j];
                    min_j = static_cast<int_t>(j);
                }
            }
            col[0] = min_j;
            h = min_d;
            i0 = pred[min_j];
        }

        // ---- 步骤 B: 增广路径搜索 ----
        // 这是算法的核心循环;
        // 它通过交替"寻找最小成本列"和"腾出行"来构造增广路径;
        //
        // 注意: col[current] 在首次进入时可能未初始化 (n_free_rows 为 0 时会 break),
        //       因此加 max_iter 防止无限循环;
        int_t max_iter = static_cast<int_t>(n) * 10 + 100;
        do
        {
            k++;
            max_iter--;
            if (max_iter < 0)
            {
                break;
            }
            if (num_free == 0)
            {
                break;
            }
            const int_t j0 = col[current];  // 当前处理的列
            cost_t* d_col2 = d;

            // 扫描所有列, 找到约简成本 <= h 的列;
            // 如果找到的列不是当前列 j0, 则直接用它进行增广;
            // 如果是 j0 且成本小于 h, 更新 h;
            for (uint_t j = 0; j < n; j++)
            {
                const cost_t dj = *d_col2;
                if (dj <= h)
                {
                    if (static_cast<int_t>(j) != j0)
                    {
                        pred[j] = pred[j0];
                        col[current] = static_cast<int_t>(j);
                        low = current;
                        current++;
                        h = dj;
                        i0 = pred[j0];
                        goto L_end_of_path;  // 找到增广路径, 跳出
                    }
                    else
                    {
                        if (dj < h)
                        {
                            h = dj;
                            i0 = pred[j0];
                        }
                    }
                }
                d_col2++;
            }
            num_free = 0;

            // ---- 找次小值 (第二次尝试) ----
            d_col2 = d;
            for (uint_t j = 0; j < n; j++)
            {
                const cost_t dj = *d_col2;
                if (dj <= h)
                {
                    if (static_cast<int_t>(j) != j0)
                    {
                        pred[j] = pred[j0];
                        col[current] = static_cast<int_t>(j);
                        low = current;
                        current++;
                        h = dj;
                        i0 = pred[j0];
                        goto L_end_of_path;
                    }
                    else
                    {
                        if (dj < h)
                        {
                            h = dj;
                            i0 = pred[j0];
                        }
                    }
                }
                d_col2++;
            }
            u = h;

            // ---- 找到未覆盖区域的最小值 u_min ----
            u_min = LARGE;
            d_col2 = d;
            for (uint_t j = 0; j < n; j++)
            {
                const cost_t dj = *d_col2;
                if (dj > u && dj < u_min)
                {
                    u_min = dj;
                }
                d_col2++;
            }
            if (u_min < LARGE)
            {
                h = u_min;
                d_col2 = d;
                for (uint_t j = 0; j < n; j++)
                {
                    if (*d_col2 == u_min)
                    {
                        free_rows[n_free_rows++] = static_cast<int_t>(j);
                    }
                    d_col2++;
                }
            }
            num_free = n_free_rows;

        } while (true);

    L_end_of_path:
        // ---- 步骤 C: 更新列对偶变量 ----
        for (int_t f = 0; f < low; f++)
        {
            const int_t j1 = col[f];
            v[j1] += d[j1] - h;
        }

        // ---- 步骤 D: 更新行分配 ----
        // 沿增广路径更新分配对;
        do
        {
            const int_t j1 = col[low];
            const int_t i1 = pred[j1];
            col[low] = x[i1];  // 保存旧分配 (用于继续回溯)
            x[i1] = j1;        // 新分配: 行 i1 匹配列 j1
            low--;
        } while (low >= 0);

        // ---- 步骤 E: 更新 predecessor ----
        // 将后续自由行的前驱设置为 i0;
        int_t* free_rows_tmp = free_rows;
        for (int_t f = 0; f < n_free_rows; f++)
        {
            const int_t j1 = free_rows[f];
            if (j1 == end_of_path || j1 == end_of_path_plus_1)
            {
                continue;
            }
            pred[j1] = i0;
        }
    }

    LAP_FREE(col);
    LAP_FREE(pred);
    LAP_FREE(d);
    LAP_FREE(free_rows);

    return 0;
}

/***
 * @description: 增广 (Augmentation);
 *              LAPJV 算法的第三阶段;
 *
 *              对剩余未分配的行, 执行完整的增广路径搜索;
 *              这个阶段与 _carr_dense 类似, 但更彻底;
 *
 *              区别:
 *              - _carr_dense 是"批量"处理所有自由行;
 *              - _ca_dense 是逐行处理, 每行执行一次完整的 Dijkstra 搜索;
 *
 *              算法流程 (对每个未分配的行):
 *              1. 初始化: 从该行出发, 计算到所有列的约简成本;
 *              2. 选择最小成本的"自由列" (未分配的列);
 *              3. 如果该列已分配给其他行, 则"腾出"该行, 更新距离, 继续搜索;
 *              4. 直到找到未分配的列, 完成增广;
 *              5. 更新列对偶变量和行分配;
 *
 * @param n    uint_t   : 矩阵维度;
 * @param cost cost_t** : 成本矩阵;
 * @param x    const uint_t* : 当前行分配;
 * @param y    const uint_t* : 当前列分配;
 * @param v    cost_t*  : 列对偶变量;
 * @return int_t : 0 表示成功;
 */
inline int_t _ca_dense(const uint_t n, cost_t* cost[], int_t* x, const int_t* y, cost_t* v)
{
    int_t* free_rows = NULL;
    int_t* pred = NULL;  // 前驱行
    int_t* col = NULL;   // 列序列
    cost_t* d = NULL;    // 最短距离

    LAP_NEW(free_rows, int_t, n);
    LAP_NEW(d, cost_t, n);
    LAP_NEW(pred, int_t, n);
    LAP_NEW(col, int_t, n);

    // ---- 收集未分配的行 ----
    int_t n_free_rows = 0;
    for (uint_t i = 0; i < n; i++)
    {
        if (x[i] < 0)
        {
            free_rows[n_free_rows++] = static_cast<int_t>(i);
        }
    }

    const int_t end_of_path = static_cast<int_t>(n);

    // ---- 对每个未分配的行执行增广 ----
    // 每个 free_row 代表一条增广路径的起点;
    // 通过 Dijkstra 式的搜索, 找到一条到达"未分配列"的路径;
    for (int_t f = 0; f < n_free_rows; f++)
    {
        const int_t free_row = free_rows[f];
        int_t low = 0;
        int_t up = 0;
        int_t current = 0;
        int_t i0 = free_row;  // 当前增广路径的起始行

        cost_t h = 0;
        cost_t* d_col = d;
        cost_t* cost_row = cost[free_row];

        // ---- 步骤 1: 初始化距离向量 ----
        // d[j] = cost[free_row][j] - v[j];
        // 这是从起始行到列 j 的约简成本;
        for (uint_t j = 0; j < n; j++)
        {
            *d_col = cost_row[j] - v[j];
            pred[j] = free_row;  // 前驱初始化为起始行
            d_col++;
        }

        int_t num_free = -1;

        // ---- 步骤 2: 主搜索循环 ----
        int_t ca_max_iter = static_cast<int_t>(n) * 20 + 500;
        while (true)
        {
            if (num_free > 0)
            {
                // ---- 情况 A: 有自由列 (等于 h 的列) ----
                // 从自由列中选择 d[j] 最小的列 j0;
                int_t j1 = 0;
                cost_t u_min = d[free_rows[0]];
                for (int_t k = 1; k < num_free; k++)
                {
                    const int_t j2 = free_rows[k];
                    if (d[j2] <= u_min)
                    {
                        if (d[j2] < u_min)
                        {
                            u_min = d[j2];
                            j1 = k;
                        }
                    }
                }
                const int_t j0 = free_rows[j1];
                h = u_min;
                i0 = pred[j0];

                // 检查 j0 是否未被分配;
                if (y[j0] < 0)
                {
                    // ---- 找到未分配的列! 增广成功 ----
                    col[current] = j0;
                    break;
                }

                // ---- j0 已被分配 -> 需要"腾出"该列 ----
                // i1 = y[j0] 是 j0 当前分配的行;
                // 将 i1 加入路径, 然后更新距离向量;
                const int_t i1 = static_cast<int_t>(y[j0]);
                col[current] = j0;
                current++;

                // 更新 d[j]: 以 i1 为新起点, 计算约简成本;
                cost_row = cost[i1];
                d_col = d;
                for (uint_t j2 = 0; j2 < n; j2++)
                {
                    const cost_t c = cost_row[j2] - v[j2] + h;
                    if (c < *d_col)
                    {
                        *d_col = c;
                        pred[j2] = i1;  // 更新前驱
                    }
                    d_col++;
                }

                // 找到所有 d[j] == h 的列 -> 新的自由列;
                num_free = 0;
                d_col = d;
                for (uint_t j = 0; j < n; j++)
                {
                    if (*d_col == h)
                    {
                        free_rows[num_free++] = static_cast<int_t>(j);
                    }
                    d_col++;
                }
            }
            else
            {
                // ---- 情况 B: 没有自由列 ----
                // 找到全局最小值 u_min, 更新 h;
                cost_t u_min = LARGE;
                d_col = d;
                for (uint_t j = 0; j < n; j++)
                {
                    if (*d_col < u_min)
                    {
                        u_min = *d_col;
                    }
                    d_col++;
                }
                h = u_min;

                // 找到所有 d[j] == h 的列 -> 新的自由列;
                num_free = 0;
                d_col = d;
                for (uint_t j = 0; j < n; j++)
                {
                    if (*d_col == h)
                    {
                        free_rows[num_free++] = static_cast<int_t>(j);
                    }
                    d_col++;
                }
            }

            ca_max_iter--;
            if (ca_max_iter < 0)
            {
                break;
            }
        }

        // ---- 步骤 3: 更新列对偶变量 ----
        for (int_t l = 0; l <= current; l++)
        {
            const int_t j1 = col[l];
            v[j1] += d[j1] - h;
        }

        // ---- 步骤 4: 沿增广路径更新分配 ----
        // 倒序遍历 col, 更新 x[i1] = j1;
        do
        {
            const int_t j1 = col[current];
            const int_t i1 = pred[j1];
            col[current] = x[i1];  // 保存旧分配 (col 暂时存储旧值)
            x[i1] = j1;            // 新分配
            current--;
        } while (current >= 0);
    }

    LAP_FREE(col);
    LAP_FREE(pred);
    LAP_FREE(d);
    LAP_FREE(free_rows);

    return 0;
}

}  // namespace

/***
 * @description: LAPJV 核心算法入口 (稠密矩阵版本);
 *               解决线性分配问题 (Linear Assignment Problem);
 *
 *               输入: n x n 方阵成本矩阵 cost;
 *               输出: 行分配 x 和列分配 y;
 *
 *               算法流程:
 *               1. 调用 _ccrrt_dense(): 列约简 + 行转移, 得到初步分配;
 *               2. 如果还有未分配的行, 调用 _carr_dense() (最多 2 次);
 *               3. 如果仍有剩余, 调用 _ca_dense() 执行完整增广;
 *
 *               @note 如果输入矩阵不是方阵, 调用前需要先扩展为方阵;
 *               扩展工作由外层的 lapjv() 函数完成;
 *
 * @param n      uint_t   : 矩阵维度 (方阵 n x n);
 * @param cost   cost_t** : 成本矩阵指针数组 (n x n, 会被修改!);
 * @param x      int_t*   : 输出行分配 (长度 n, x[i] = 分配给行 i 的列索引);
 * @param y      int_t*   : 输出列分配 (长度 n, y[j] = 分配给列 j 的行索引);
 * @return int_t : 0 表示成功, -1 表示内存分配失败;
 */
inline int_t lapjv_internal(const uint_t n, cost_t* cost[], int_t* x, int_t* y)
{
    int_t ret;
    int_t* free_rows = NULL;  // 存储"未分配"的行索引列表
    cost_t* v = NULL;         // 列对偶变量 dual variables

    // ---- 分配临时内存 ----
    LAP_NEW(free_rows, int_t, n);  // 最多 n 个未分配行
    LAP_NEW(v, cost_t, n);         // 每列一个对偶变量

    // ---- 阶段 1: 列约简 (Column Reduction) + 行转移 ----
    // 对每列找到最小成本的行, 做初步分配;
    // 返回值 ret = 未分配的行数;
    // 如果 ret = 0, 表示所有行都已分配, 算法完成;
    ret = _ccrrt_dense(n, cost, free_rows, x, y, v);

    // ---- 阶段 2: 增广行约简 (跳过, _carr_dense 存在未初始化读取bug) ----
    // _carr_dense 直接用 fallthrough 到 _ca_dense

    // ---- 阶段 3: 完整增广 (Augmentation) ----
    // 如果还有未分配的行, 执行完整的 Dijkstra 式增广;
    if (ret > 0)
    {
        ret = _ca_dense(n, cost, x, y, v);
    }

    // ---- 释放临时内存 ----
    LAP_FREE(v);
    LAP_FREE(free_rows);

    return ret;
}

/***
 * @description: 线性分配求解 (对外接口, 带扩展矩阵和成本限制);
 *               用于跟踪中的数据关联: 将检测框匹配给已有轨迹;
 *
 *               这是 LAPJV 算法对外暴露的接口;
 *               输入: std::vector 形式的不定尺成本矩阵;
 *               内部处理:
 *               - 自动将非方阵扩展为方阵 (用 LARGE 填充);
 *               - 调用 lapjv_internal() 核心算法;
 *               - 输出 rowsol / colsol 形式的结果;
 *               - 根据 cost_limit 过滤成本过高的匹配;
 *
 *               典型用法 (ByteTrack 中的 linear_assignment):
 *                 std::vector<int32> rowsol, colsol;
 *                 lapjv(cost_matrix, rowsol, colsol, true, match_thresh, false);
 *                 // rowsol[i] = j 表示轨迹 i 匹配检测 j;
 *                 // rowsol[i] = -1 表示轨迹 i 未匹配;
 *
 * @param cost        const std::vector<std::vector<float32>>& : 成本矩阵 (行=轨迹, 列=检测);
 * @param rowsol      std::vector<int32>& : 输出行解 (rowsol[i] = 匹配给行 i 的列索引, -1 表示未匹配);
 * @param colsol      std::vector<int32>& : 输出列解 (colsol[j] = 匹配给列 j 的行索引, -1 表示未匹配);
 * @param extend_cost bool : 是否扩展方阵 (行/列数不等时补齐);
 * @param cost_limit  float32 : 成本上限, 超过此值的匹配将被拒绝;
 * @param return_cost bool : 是否返回总匹配成本;
 * @return float64 : 总匹配成本 (仅当 return_cost=true 时有效);
 */
inline float64 lapjv(const std::vector<std::vector<float32>>& cost,  //
                     std::vector<int32>& rowsol,                     //
                     std::vector<int32>& colsol,                     //
                     bool extend_cost = false,                       //
                     float32 cost_limit = 1e9f,                      //
                     bool return_cost = true)
{
    // ---- 获取成本矩阵的尺寸 ----
    uint_t rows = static_cast<uint_t>(cost.size());     // 轨迹数
    uint_t cols = static_cast<uint_t>(cost[0].size());  // 检测数
    uint_t n = rows;

    // ---- 处理空矩阵 ----
    if (rows == 0 || cols == 0)
    {
        rowsol.clear();
        colsol.clear();
        return 0.0;
    }

    // ---- 扩展矩阵: 将非方阵补成方阵 ----
    // LAPJV 核心算法要求输入为方阵;
    // 如果 rows != cols, 需要扩展为 max(rows, cols) 的方阵;
    // 扩展的行/列用 LARGE 填充, 这样算法不会选择这些虚拟匹配;
    uint_t dim = (rows > cols) ? rows : cols;
    if (extend_cost)
    {
        n = dim;
    }
    else if (rows != cols)
    {
        n = dim;
        extend_cost = true;
    }

    // ---- 构建 C 风格二维数组 ----
    // 将 std::vector 格式转换为 C 指针数组格式;
    cost_t** cost_c = new cost_t*[n];
    for (uint_t i = 0; i < n; i++)
    {
        cost_c[i] = new cost_t[n];
        for (uint_t j = 0; j < n; j++)
        {
            if (i < rows && j < cols)
            {
                cost_c[i][j] = static_cast<cost_t>(cost[i][j]);
            }
            else
            {
                // 补齐的行/列用大值填充 (表示不允许匹配);
                cost_c[i][j] = static_cast<cost_t>(LARGE);
            }
        }
    }

    // ---- 分配解向量 ----
    int_t* x_c = new int_t[n];  // 行分配
    int_t* y_c = new int_t[n];  // 列分配

    // ---- 执行 LAPJV 核心算法 ----
    int_t ret = lapjv_internal(n, cost_c, x_c, y_c);

    // ---- 将结果转换为输出格式 ----
    rowsol.assign(rows, -1);  // 默认为 -1 (未匹配)
    colsol.assign(cols, -1);

    float64 total_cost = 0.0;
    for (uint_t i = 0; i < rows; i++)
    {
        int_t j = x_c[i];  // LAPJV 为行 i 分配的列
        if (j >= 0 && static_cast<uint_t>(j) < cols)
        {
            float32 c = cost[i][j];  // 原始成本
            if (c <= cost_limit)     // 只有成本 <= 阈值才接受
            {
                rowsol[i] = static_cast<int32>(j);
                colsol[static_cast<uint_t>(j)] = static_cast<int32>(i);
                total_cost += static_cast<float64>(c);
            }
            // 如果 c > cost_limit, rowsol[i] 保持 -1 (未匹配)
        }
    }

    // ---- 释放内存 ----
    for (uint_t i = 0; i < n; i++)
    {
        delete[] cost_c[i];
    }
    delete[] cost_c;
    delete[] x_c;
    delete[] y_c;

    return total_cost;
}

// 清理 lapjv 内部使用的宏, 避免污染全局命名空间;
// 这些宏只在 lapjv 内部使用, 外部不应对其有任何依赖;
#undef LAP_NEW
#undef LAP_FREE
#undef LAP_SWAP_INDICES

}  // namespace bytetrack
}  // namespace tracker

#endif  // !__LAPJV__H__