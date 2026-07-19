/***
 * @Author       : gxs
 * @Date         : 2026-06-22 22:30:00
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-22 22:30:00
 * @FilePath     : /visionAlgorithm/lib/tracker/deepsort/hungarian.hpp
 * @Description  : Munkres (Hungarian) 算法实现, 用于 DeepSORT 级联匹配;
 *
 *                 与 ByteTrack LAPJV 算法的区别:
 *                 - LAPJV: O(n^3) 稠密矩阵, 适合大规模 IoU 匹配 (ByteTrack);
 *                 - Munkres: O(n^3) 经典匈牙利, 适合带门控的级联匹配 (DeepSORT);
 *
 *                 两种算法数学上等价 (都解线性分配问题),
 *                 但实现细节不同, 分开实现以便各自优化;
 *
 * =====================================================================
 * Munkres 算法核心思想
 * =====================================================================
 *
 *  Munkres 算法 (也叫 Kuhn-Munkres 算法) 分 5 个步骤:
 *
 *  第 0 步 - 预处理:
 *     处理无穷大值, 行/列归约 (每行/列减去最小值);
 *
 *  第 1 步 - 标记零元素 (Star):
 *     每行每列最多选一个零元素, 标记为 STAR;
 *
 *  第 2 步 - 检查是否完成:
 *     如果已标记的 STAR 数量 == min(rows, cols), 算法结束;
 *     否则进入第 3 步;
 *
 *  第 3 步 - 找未覆盖的零 (Prime):
 *     找未覆盖的零元素, 标记为 PRIME;
 *     如果找到, 检查该行是否有 STAR;
 *       有 STAR -> 覆盖该行, 取消该 STAR 列的覆盖, 继续找;
 *       无 STAR -> 进入第 4 步;
 *     如果没找到 -> 进入第 5 步;
 *
 *  第 4 步 - 增广路径:
 *     从 PRIME 位置开始, 沿 PRIME -> STAR -> PRIME -> STAR ... 交替路径,
 *     将路径上的 PRIME 改为 STAR, STAR 改为普通零;
 *     清除所有 PRIME, 回到第 2 步;
 *
 *  第 5 步 - 调整矩阵:
 *     找到未覆盖区域的最小值 h;
 *     未覆盖行全体 +h, 已覆盖列全体 -h;
 *     回到第 3 步;
 * =====================================================================
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __DEEPSORT_HUNGARIAN__H__
#define __DEEPSORT_HUNGARIAN__H__

#include <algorithm>  // std::max, std::min
#include <cfloat>     // FLT_MAX
#include <cmath>      // std::sqrt
#include <limits>     // std::numeric_limits
#include <utility>    // std::pair
#include <vector>     // std::vector

#include "types.hpp"

namespace tracker
{
namespace deepsort
{

// 辅助宏 (与 Munkres 原版 matrix.h 保持一致);
// 用于返回两个值中的较小/较大者, 避免标准库 min/max 的模板推导问题;
#define HUN_XYZMIN(x, y) ((x) < (y) ? (x) : (y))
#define HUN_XYZMAX(x, y) ((x) > (y) ? (x) : (y))

/***
 * @description: 轻量级二维矩阵类 (header-only);
 *               用于 Munkres 算法的中间计算;
 *
 *               功能: 提供动态二维数组的基本操作 (resize, clear, 下标访问等);
 *
 *               @note 这是一个最小实现, 不包含高级矩阵运算;
 *               只在本文件的 Munkres 算法内部使用;
 *
 *               参考 deepsort/include/matrix.h;
 */
template <class T>
class Matrix
{
   private:
    // m_matrix: 二维数组指针 (T** 形式, 行优先存储);
    T** m_matrix;

    // m_rows: 矩阵行数;
    size_t m_rows;

    // m_columns: 矩阵列数;
    size_t m_columns;

   public:
    /***
     * @description: 默认构造函数; 创建空矩阵 (m_matrix = nullptr, rows/columns = 0);
     */
    Matrix() : m_rows(0), m_columns(0), m_matrix(nullptr) {}

    /***
     * @description: 带尺寸的构造函数; 创建 rows x columns 的矩阵, 所有元素初始化为 0;
     * @param rows    size_t : 行数;
     * @param columns size_t : 列数;
     */
    Matrix(const size_t rows, const size_t columns) : m_matrix(nullptr) { this->resize(rows, columns); }

    /***
     * @description: 拷贝构造函数; 深拷贝另一个 Matrix 的数据;
     * @param other const Matrix<T>& : 源矩阵;
     */
    Matrix(const Matrix<T>& other) : m_matrix(nullptr)
    {
        if (other.m_matrix != nullptr)
        {
            this->resize(other.m_rows, other.m_columns);

            for (size_t i = 0; i < this->m_rows; i++)
            {
                for (size_t j = 0; j < this->m_columns; j++)
                {
                    this->m_matrix[i][j] = other.m_matrix[i][j];
                }
            }
        }
        else
        {
            this->m_matrix = nullptr;
            this->m_rows = 0;
            this->m_columns = 0;
        }
    }

    /***
     * @description: 赋值运算符; 深拷贝另一个 Matrix 的数据, 先释放原有内存再重新分配;
     * @param other const Matrix<T>& : 源矩阵;
     * @return Matrix<T>& : 本对象的引用;
     */
    Matrix<T>& operator=(const Matrix<T>& other)
    {
        // 自赋值检查: 如果 this == &other, 直接返回;
        if (this == &other)
            return *this;

        if (other.m_matrix != nullptr)
        {
            this->resize(other.m_rows, other.m_columns);
            for (size_t i = 0; i < this->m_rows; i++)
            {
                for (size_t j = 0; j < this->m_columns; j++)
                {
                    this->m_matrix[i][j] = other.m_matrix[i][j];
                }
            }
        }
        else
        {
            // 释放原有内存;
            for (size_t i = 0; i < this->m_rows; i++)
            {
                delete[] this->m_matrix[i];
            }
            delete[] this->m_matrix;

            this->m_matrix = nullptr;
            this->m_rows = 0;
            this->m_columns = 0;
        }

        return *this;
    }

    /***
     * @description: 析构函数; 释放动态分配的二维数组内存;
     */
    ~Matrix()
    {
        if (this->m_matrix != nullptr)
        {
            for (size_t i = 0; i < this->m_rows; i++)
            {
                delete[] this->m_matrix[i];
            }
            delete[] this->m_matrix;
        }

        this->m_matrix = nullptr;
    }

    /***
     * @description: 调整矩阵大小; 保留原有数据 (新区域用 default_value 填充);
     *               @note 如果 m_matrix 为 nullptr, 相当于新建矩阵;
     * @param rows          size_t : 新行数;
     * @param columns       size_t : 新列数;
     * @param default_value T       : 新元素的默认值 (默认 0);
     */
    void resize(const size_t rows, const size_t columns, const T default_value = 0)
    {
        if (this->m_matrix == nullptr)
        {
            // ---- 新建矩阵 ----
            this->m_matrix = new T*[rows];
            for (size_t i = 0; i < rows; i++)
            {
                this->m_matrix[i] = new T[columns];
            }

            this->m_rows = rows;
            this->m_columns = columns;

            this->clear();  // 所有元素置 0;
        }
        else
        {
            // ---- 调整已有矩阵 ----
            // 创建新矩阵, 初始化所有元素为 default_value;
            T** new_matrix = new T*[rows];
            for (size_t i = 0; i < rows; i++)
            {
                new_matrix[i] = new T[columns];
                for (size_t j = 0; j < columns; j++)
                {
                    new_matrix[i][j] = default_value;
                }
            }

            // 复制旧矩阵的数据到新矩阵 (只复制重叠区域);
            size_t minrows = HUN_XYZMIN(rows, this->m_rows);
            size_t mincols = HUN_XYZMIN(columns, this->m_columns);

            for (size_t x = 0; x < minrows; x++)
            {
                for (size_t y = 0; y < mincols; y++)
                {
                    new_matrix[x][y] = this->m_matrix[x][y];
                }
            }

            // 释放旧矩阵;
            for (size_t i = 0; i < this->m_rows; i++)
            {
                delete[] this->m_matrix[i];
            }
            delete[] this->m_matrix;

            this->m_matrix = new_matrix;
        }

        this->m_rows = rows;
        this->m_columns = columns;
    }

    /***
     * @description: 将所有元素置 0;
     */
    void clear()
    {
        for (size_t i = 0; i < this->m_rows; i++)
        {
            for (size_t j = 0; j < this->m_columns; j++)
            {
                this->m_matrix[i][j] = 0;
            }
        }
    }

    /***
     * @description: 下标访问 (可修改); 返回 (x, y) 处元素的引用;
     * @param x size_t : 行索引;
     * @param y size_t : 列索引;
     * @return T& : 元素的引用;
     */
    T& operator()(const size_t x, const size_t y) { return this->m_matrix[x][y]; }

    /***
     * @description: 下标访问 (只读); 返回 (x, y) 处元素的 const 引用;
     * @param x size_t : 行索引;
     * @param y size_t : 列索引;
     * @return const T& : 元素的 const 引用;
     */
    const T& operator()(const size_t x, const size_t y) const { return this->m_matrix[x][y]; }

    /***
     * @description: 获取矩阵中的最大值;
     * @return T : 最大值;
     */
    const T mmax() const
    {
        T max_val = this->m_matrix[0][0];
        for (size_t i = 0; i < this->m_rows; i++)
        {
            for (size_t j = 0; j < this->m_columns; j++)
            {
                max_val = std::max<T>(max_val, this->m_matrix[i][j]);
            }
        }
        return max_val;
    }

    /***
     * @description: 获取矩阵的较小维度 (min(rows, columns));
     * @return size_t : min(rows, columns);
     */
    inline size_t minsize() const { return ((this->m_rows < this->m_columns) ? this->m_rows : this->m_columns); }

    /***
     * @description: 获取列数;
     * @return size_t : m_columns;
     */
    inline size_t columns() const { return this->m_columns; }

    /***
     * @description: 获取行数;
     * @return size_t : m_rows;
     */
    inline size_t rows() const { return this->m_rows; }
};

/***
 * @description: Munkres 匈牙利算法 (Kuhn-Munkres 算法);
 *               解决线性分配问题, O(n^3) 复杂度;
 *
 *               输入: n x n 成本矩阵 (非方阵会自动扩展);
 *               输出: matrix(row, col) == 0 表示(row, col)是匹配对;
 *                     matrix(row, col) == -1 表示未匹配;
 *
 *               参考 deepsort/include/munkres.h;
 *
 * =====================================================================
 * 算法内部状态标记
 * =====================================================================
 *
 *  mask_matrix 中的每个元素有三种状态:
 *   - NORMAL (0): 普通元素, 未被标记;
 *   - STAR   (1): 被"星号"标记的零元素 (即已确定的匹配);
 *   - PRIME  (2): 被"撇号"标记的零元素 (即候选匹配);
 *
 *  row_mask / col_mask: 标记哪些行/列已被"覆盖";
 *    覆盖的含义: 该行/列已确定匹配, 不再参与后续搜索;
 * =====================================================================
 */
template <typename Data>
class Munkres
{
    // ---- 内部状态常量 ----
    static const int32 NORMAL = 0;  // 普通元素
    static const int32 STAR = 1;    // 星号 (已确定匹配)
    static const int32 PRIME = 2;   // 撇号 (候选匹配)

   private:
    // mask_matrix: 与 matrix 同尺寸的标记矩阵, 记录每个元素的 STAR/PRIME/NORMAL 状态;
    Matrix<int32> mask_matrix;

    // matrix: 内部工作矩阵 (原始成本矩阵经过归约后的结果);
    Matrix<Data> matrix;

    // row_mask: 行覆盖标记; row_mask[i] = true 表示第 i 行已被覆盖;
    bool* row_mask;

    // col_mask: 列覆盖标记; col_mask[j] = true 表示第 j 列已被覆盖;
    bool* col_mask;

    // saverow, savecol: 记录第 3 步找到的未覆盖零的位置;
    size_t saverow = 0;
    size_t savecol = 0;

   public:
    /***
     * @description: 求解线性分配问题 (in-place 修改矩阵);
     *               solve() 后, matrix(row,col) == 0 表示匹配, < 0 表示未匹配;
     *
     *               输入非方阵时会自动扩展为方阵, 空位用最大值填充;
     *
     * @param m Matrix<Data>& : 成本矩阵 (将被修改为结果矩阵);
     */
    void solve(Matrix<Data>& m)
    {
        // ---- 获取矩阵尺寸 ----
        // 矩阵行
        const size_t rows = m.rows();
        // 矩阵列
        const size_t columns = m.columns();
        // 矩阵较大的尺寸
        const size_t size = HUN_XYZMAX(rows, columns);

        // ---- 复制输入矩阵到内部工作矩阵 ----
        this->matrix = m;

        // ---- 非方阵扩展为方阵 ----
        // 扩展的行/列用矩阵当前最大值填充 (这样不会干扰最优解);
        if (rows != columns)
        {
            this->matrix.resize(size, size, this->matrix.mmax());
        }

        // ---- 初始化标记矩阵 ----
        this->mask_matrix.resize(size, size);

        // ---- 初始化行/列覆盖标记 ----
        this->row_mask = new bool[size];
        this->col_mask = new bool[size];
        for (size_t i = 0; i < size; i++)
        {
            this->row_mask[i] = false;
            this->col_mask[i] = false;
        }

        // ---- 第 0 步: 处理无穷大值和行/列归约 ----
        this->replace_infinites(this->matrix);
        this->minimize_along_direction(this->matrix, rows >= columns);
        this->minimize_along_direction(this->matrix, rows < columns);

        // ---- 主循环 (5 个步骤) ----
        int32 step = 1;
        while (step)
        {
            switch (step)
            {
                case 1:
                    step = this->step1();
                    break;
                case 2:
                    step = this->step2();
                    break;
                case 3:
                    step = this->step3();
                    break;
                case 4:
                    step = this->step4();
                    break;
                case 5:
                    step = this->step5();
                    break;
            }
        }

        // ---- 将结果写入输出: STAR 位置 -> 0 (匹配), 其余 -> -1 (未匹配) ----
        for (size_t row = 0; row < size; row++)
        {
            for (size_t col = 0; col < size; col++)
            {
                if (this->mask_matrix(row, col) == STAR)
                {
                    this->matrix(row, col) = 0;
                }
                else
                {
                    this->matrix(row, col) = -1;
                }
            }
        }

        // ---- 还原为原始尺寸 ----
        this->matrix.resize(rows, columns);
        m = this->matrix;

        delete[] this->row_mask;
        delete[] this->col_mask;
    }

    /***
     * @description: 将无穷大值替换为大于矩阵最大值的有限值;
     *               这是必要的预处理: Munkres 算法不能直接处理无穷大;
     *               替换为"最大值+1"的效果等价于禁止该匹配;
     * @param mat Matrix<Data>& : 待处理的矩阵;
     */
    static void replace_infinites(Matrix<Data>& mat)
    {
        const size_t rows = mat.rows(), columns = mat.columns();
        double max_val = mat(0, 0);
        constexpr double infinity = std::numeric_limits<double>::infinity();

        // 找到矩阵中的最大值 (跳过无穷大);
        for (size_t row = 0; row < rows; row++)
        {
            for (size_t col = 0; col < columns; col++)
            {
                if (mat(row, col) != infinity)
                {
                    if (max_val == infinity)
                    {
                        max_val = mat(row, col);
                    }
                    else
                    {
                        max_val = HUN_XYZMAX(max_val, mat(row, col));
                    }
                }
            }
        }

        // 如果全矩阵都是无穷大, 用 0 代替;
        if (max_val == infinity)
        {
            max_val = 0;
        }
        else
        {
            max_val++;
        }

        // 将所有无穷大值替换为 max_val;
        for (size_t row = 0; row < rows; row++)
        {
            for (size_t col = 0; col < columns; col++)
            {
                if (mat(row, col) == infinity)
                {
                    mat(row, col) = static_cast<Data>(max_val);
                }
            }
        }
    }

    /***
     * @description: 沿行/列方向归约 (每个行/列减去该行/列的最小值);
     *               归约的目的是让每行每列至少有一个零元素;
     *
     *               如果 over_columns == true: 对每列操作 (列归约);
     *               如果 over_columns == false: 对每行操作 (行归约);
     *
     * @param mat          Matrix<Data>& : 待归约的矩阵 (原地修改);
     * @param over_columns bool : true=列归约, false=行归约;
     */
    static void minimize_along_direction(Matrix<Data>& mat, const bool over_columns)
    {
        const size_t outer_size = over_columns ? mat.columns() : mat.rows();
        const size_t inner_size = over_columns ? mat.rows() : mat.columns();

        for (size_t i = 0; i < outer_size; i++)
        {
            // 找到当前行/列的最小值;
            double min_val = over_columns ? mat(0, i) : mat(i, 0);
            for (size_t j = 1; j < inner_size && min_val > 0; j++)
            {
                min_val = HUN_XYZMIN(min_val, over_columns ? mat(j, i) : mat(i, j));
            }

            // 如果最小值 > 0, 每行/列都减去最小值;
            if (min_val > 0)
            {
                for (size_t j = 0; j < inner_size; j++)
                {
                    if (over_columns)
                    {
                        mat(j, i) -= static_cast<Data>(min_val);
                    }
                    else
                    {
                        mat(i, j) -= static_cast<Data>(min_val);
                    }
                }
            }
        }
    }

   private:
    /***
     * @description: 在矩阵中查找值等于 item 的未覆盖元素;
     *               如果找到, 通过引用的 row, col 返回位置;
     * @param item double : 要查找的值 (通常为 0);
     * @param row  size_t& : 输出, 找到的行索引;
     * @param col  size_t& : 输出, 找到的列索引;
     * @return bool : true 找到, false 未找到;
     */
    inline bool find_uncovered_in_matrix(const double item, size_t& row, size_t& col) const
    {
        const size_t rows = this->matrix.rows();
        const size_t columns = this->matrix.columns();

        // 遍历矩阵, 找到值等于 item 的未覆盖元素;
        // 遍历行
        for (row = 0; row < rows; row++)
        {
            // 如果该行已被覆盖, 跳过
            if (!this->row_mask[row])  // 0表示该行没有覆盖, 则继续执行代码;
            {
                // 遍历每一列, 找到0的时候判断一下当前列是否存在 STAR
                for (col = 0; col < columns; col++)
                {
                    // 如果该元素是零且该行/列尚未有 STAR, 标记为 STAR;
                    if (!this->col_mask[col])  // 0 表示该列没有被覆盖
                    {
                        // 找到了 行列 都没有被覆盖的区域的首个 0
                        if (this->matrix(row, col) == item)
                        {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /***
     * @description: 检查一个 pair 是否在列表中;
     * @param needle   const std::pair<size_t, size_t>& : 要查找的元素;
     * @param haystack const std::vector<std::pair<size_t, size_t>>& : 列表;
     * @return bool : true 找到, false 未找到;
     */
    bool pair_in_list(const std::pair<size_t, size_t>& needle,  //
                      const std::vector<std::pair<size_t, size_t>>& haystack)
    {
        for (size_t i = 0; i < haystack.size(); i++)
        {
            if (needle == haystack[i])
            {
                return true;
            }
        }
        return false;
    }

    /***
     * @description: 第 1 步: 标记零元素;
     *               遍历矩阵, 如果遇到零元素且该行/列尚未有 STAR, 标记为 STAR;
     *               完成后进入第 2 步;
     * @return int32 : 下一步的步骤编号 (总是 2);
     */
    int32 step1()
    {
        const size_t rows = this->matrix.rows();
        const size_t columns = this->matrix.columns();

        // 因为进行了规约,
        // 遍历行
        for (size_t row = 0; row < rows; row++)
        {
            // 在当前行遍历每一列, 找到0的时候判断一下当前列是否存在 STAR
            // 如果存在 STAR 则跳到下一列, 否则标记为 STAR
            for (size_t col = 0; col < columns; col++)
            {
                // 如果该元素是零且该行/列尚未有 STAR, 标记为 STAR;
                if (0 == this->matrix(row, col))
                {
                    // 检查该列是否已有 STAR (只检查已处理的行);
                    for (size_t nrow = 0; nrow < row; nrow++)
                    {
                        if (STAR == this->mask_matrix(nrow, col))
                        {
                            // 该列已有 STAR, 跳过;
                            goto next_column;
                        }
                    }

                    // 该列没有 STAR, 标记为 STAR;
                    this->mask_matrix(row, col) = STAR;
                    // 该行已标记 STAR, 跳到下一行;
                    goto next_row;
                }
            next_column:;
            }
        next_row:;
        }
        return 2;
    }

    /***
     * @description: 第 2 步: 检查是否完成;
     *               统计已标记 STAR 的列数, 如果 covercount >= min(rows, cols) 则完成;
     *               否则进入第 3 步;
     *               只负责统计 STAR 的列数, 不负责别的判断
     * @return int32 : 0 表示完成, 3 表示继续下一步的步骤编号;
     */
    int32 step2()
    {
        // 行数
        const size_t rows = this->matrix.rows();
        // 列数
        const size_t columns = this->matrix.columns();

        size_t covercount = 0;
        // 遍历行
        for (size_t row = 0; row < rows; row++)
        {
            // 遍历列
            for (size_t col = 0; col < columns; col++)
            {
                // 如果mask矩阵显示为 STAR
                if (STAR == this->mask_matrix(row, col))
                {
                    // 覆盖该 STAR 所在的列;
                    this->col_mask[col] = true;
                    // STAR 列的计数加 1;
                    covercount++;
                }
            }
        }
        // 如果覆盖的列数大于等于矩阵的最小维度, 则表示已经找到了最优解
        if (covercount >= this->matrix.minsize())
        {
            return 0;  // 完成!
        }
        return 3;
    }

    /***
     * @description: 第 3 步: 找未覆盖的零;
     *               如果找到, 标记为 PRIME, 并检查该行是否有 STAR;
     *               有 STAR -> 覆盖该行, 取消该 STAR 列的覆盖, 继续回去找;
     *               无 STAR -> 进入第 4 步;
     *               如果没找到未覆盖的零 -> 进入第 5 步;
     * @return int32 : 下一步的步骤编号 (3, 4, 或 5);
     */
    int32 step3()
    {
        // 从为被覆盖的区域找到一个 0
        if (this->find_uncovered_in_matrix(0, this->saverow, this->savecol))
        {
            // 找到一个未覆盖的零, 标记为 PRIME;
            this->mask_matrix(this->saverow, this->savecol) = PRIME;
        }
        else
        {
            // 没有未覆盖的零, 需要调整矩阵;
            return 5;
        }

        // 在未覆盖区域找到了一个 0, 检查该行是否有 STAR;
        for (size_t ncol = 0; ncol < this->matrix.columns(); ncol++)
        {
            // 该行有 STAR -> 覆盖该行, 取消该列的覆盖, 继续找;
            if (this->mask_matrix(this->saverow, ncol) == STAR)
            {
                // 有 STAR -> 覆盖该行, 取消该列的覆盖, 继续找;
                this->row_mask[this->saverow] = true;
                this->col_mask[ncol] = false;
                return 3;
            }
        }

        // NOTE:该行没有 STAR -> 进入第 4 步 (增广路径);
        return 4;
    }

    /***
     * @description: 第 4 步: 增广路径;
     *               从 PRIME 位置开始, 构造交替路径 (PRIME -> STAR -> PRIME -> STAR ...);
     *               将路径上的 PRIME 改为 STAR, STAR 改为普通零;
     *               清除所有 PRIME 标记, 清除所有行/列覆盖, 回到第 2 步;
     * @return int32 : 下一步的步骤编号 (总是 2);
     */
    int32 step4()
    {
        const size_t rows = this->matrix.rows();
        const size_t columns = this->matrix.columns();

        // 从 PRIME 位置开始, 构造交替路径 (PRIME -> STAR -> PRIME -> STAR ...);
        std::vector<std::pair<size_t, size_t>> seq;
        // 添加 起始的位置
        std::pair<size_t, size_t> z0(this->saverow, this->savecol);
        seq.push_back(z0);

        std::pair<size_t, size_t> z1;
        std::pair<size_t, size_t> z2n;

        size_t row = 0, col = this->savecol;
        bool made_pair = false;
        do
        {
            // 找当前列中的 STAR;
            made_pair = false;
            for (row = 0; row < rows; row++)
            {
                if (this->mask_matrix(row, col) == STAR)
                {
                    z1.first = row;
                    z1.second = col;
                    if (this->pair_in_list(z1, seq))
                    {
                        continue;
                    }
                    made_pair = true;
                    seq.push_back(z1);
                    break;
                }
            }

            if (!made_pair)
            {
                break;
            }

            // 找当前行中的 PRIME;
            made_pair = false;
            for (col = 0; col < columns; col++)
            {
                if (this->mask_matrix(row, col) == PRIME)
                {
                    z2n.first = row;
                    z2n.second = col;
                    if (this->pair_in_list(z2n, seq))
                    {
                        continue;
                    }
                    made_pair = true;
                    seq.push_back(z2n);
                    break;
                }
            }
        } while (made_pair);

        // 沿路径增广: STAR <-> PRIME 互换;
        for (size_t i = 0; i < seq.size(); i++)
        {
            if (this->mask_matrix(seq[i].first, seq[i].second) == STAR)
            {
                this->mask_matrix(seq[i].first, seq[i].second) = NORMAL;
            }
            if (this->mask_matrix(seq[i].first, seq[i].second) == PRIME)
            {
                this->mask_matrix(seq[i].first, seq[i].second) = STAR;
            }
        }

        // 清除所有 PRIME 标记;
        for (size_t r = 0; r < this->mask_matrix.rows(); r++)
        {
            for (size_t c = 0; c < this->mask_matrix.columns(); c++)
            {
                if (this->mask_matrix(r, c) == PRIME)
                {
                    this->mask_matrix(r, c) = NORMAL;
                }
            }
        }

        // 清除所有行/列覆盖;
        for (size_t i = 0; i < rows; i++)
        {
            this->row_mask[i] = false;
        }
        for (size_t i = 0; i < columns; i++)
        {
            this->col_mask[i] = false;
        }

        return 2;
    }

    /***
     * @description: 第 5 步: 调整矩阵;
     *               找到未覆盖区域的最小值 h, 然后:
     *               1. 覆盖行全体 +h;
     *               2. 未覆盖列全体 -h;
     *               这样做的效果是: 产生新的零元素到未覆盖区域;
     *               回到第 3 步;
     * @return int32 : 下一步的步骤编号 (总是 3);
     */
    int32 step5()
    {
        const size_t rows = this->matrix.rows();
        const size_t columns = this->matrix.columns();
        double h = 100000.0;

        // 找到未覆盖区域的最小值;
        for (size_t row = 0; row < rows; row++)
        {
            if (!this->row_mask[row])
            {
                for (size_t col = 0; col < columns; col++)
                {
                    if (!this->col_mask[col])
                    {
                        if (h > this->matrix(row, col) && this->matrix(row, col) != 0)
                        {
                            h = this->matrix(row, col);
                        }
                    }
                }
            }
        }

        // 覆盖行全体 +h;
        for (size_t row = 0; row < rows; row++)
        {
            if (this->row_mask[row])  // true表示被覆盖
            {
                for (size_t col = 0; col < columns; col++)
                {
                    this->matrix(row, col) += static_cast<Data>(h);
                }
            }
        }

        // 未覆盖列全体 -h;
        for (size_t col = 0; col < columns; col++)
        {
            if (!this->col_mask[col])  // false表示未被覆盖
            {
                for (size_t row = 0; row < rows; row++)
                {
                    this->matrix(row, col) -= static_cast<Data>(h);
                }
            }
        }

        return 3;
    }
};

/***
 * @description: Hungarian 算子 (Hungarian Operator);
 *               封装 Munkres 算法, 提供匹配对输出接口;
 *
 *               输入: std::vector<std::vector<float32>> 成本矩阵 (行=轨迹, 列=检测);
 *               输出: 匹配对列表 [(track_idx, det_idx), ...];
 *
 *               典型用法:
 *                 auto pairs = hungarian_solve(cost_matrix);
 *                 for (auto& p : pairs) {
 *                     int track_idx = p.first;
 *                     int det_idx = p.second;
 *                     // 处理匹配;
 *                 }
 *
 * @param cost_matrix const std::vector<std::vector<float32>>& : 成本矩阵 [n_tracks x n_dets];
 * @return std::vector<std::pair<int32, int32>> : 匹配对列表;
 */
inline std::vector<std::pair<int32, int32>> hungarian_solve(const std::vector<std::vector<float32>>& cost_matrix)
{
    std::vector<std::pair<int32, int32>> pairs;

    // ---- 获取矩阵尺寸 ----
    int32 rows = static_cast<int32>(cost_matrix.size());
    if (rows == 0)
        return pairs;
    int32 cols = static_cast<int32>(cost_matrix[0].size());
    if (cols == 0)
        return pairs;

    // ---- 构建 Munkres 矩阵 (使用 double 类型保证精度) ----
    Matrix<double> matrix(static_cast<size_t>(rows), static_cast<size_t>(cols));
    for (int32 row = 0; row < rows; row++)
    {
        for (int32 col = 0; col < cols; col++)
        {
            matrix(static_cast<size_t>(row), static_cast<size_t>(col)) = static_cast<double>(cost_matrix[row][col]);
        }
    }

    // ---- 求解 ----
    Munkres<double> m;
    m.solve(matrix);

    // ---- 收集结果: matrix(row,col) == 0 表示匹配 ----
    for (int32 row = 0; row < rows; row++)
    {
        for (int32 col = 0; col < cols; col++)
        {
            if (static_cast<int32>(matrix(static_cast<size_t>(row), static_cast<size_t>(col))) == 0)
            {
                pairs.push_back(std::make_pair(row, col));
            }
        }
    }

    return pairs;
}

// 清理辅助宏, 避免污染全局命名空间;
#undef HUN_XYZMIN
#undef HUN_XYZMAX

}  // namespace deepsort
}  // namespace tracker

#endif  // !__DEEPSORT_HUNGARIAN__H__