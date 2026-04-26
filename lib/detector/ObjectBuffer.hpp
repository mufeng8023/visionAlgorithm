/***
 * @Author       : gxs
 * @Date         : 2026-04-25 23:28:53
 * @LastEditors  : gxs
 * @LastEditTime : 2026-04-25 23:28:53
 * @FilePath     : /visionAlgorithm/lib/detector/ObjectBuffer.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __OBJECTBUFFER__H__
#define __OBJECTBUFFER__H__

#include <algorithm>
#include <cassert>
#include <numeric>
#include <string>
#include <vector>

#include "types.hpp"

namespace yolo
{
/***
 * @description: 边界框信息存储结构相关常量定义
 *              ObjectOffset:: 方式访问
 * @return
 */
struct ObjectOffset
{
    // 边界框信息存储结构相关常量定义
    static constexpr uint32 base_box_len = 6;  // 边界框基础长度, 包含6个基本元素
    static constexpr uint32 x_center = 0;      // 中心点 x 坐标 偏移量, 在数组中的位置
    static constexpr uint32 y_center = 1;      // 中心点 y 坐标 偏移量, 在数组中的位置
    static constexpr uint32 width = 2;         // 边界框宽度 偏移量, 在数组中的位置
    static constexpr uint32 height = 3;        // 边界框高度 偏移量, 在数组中的位置
    static constexpr uint32 score = 4;         // 边界框得分 偏移量, 在数组中的位置
    static constexpr uint32 cls_id = 5;        // 边界框类别 id 偏移量, 在数组中的位置
    static constexpr uint32 extra_start = 6;   // 额外信息起始偏移量, 从第6个位置开始存储额外信息
};

/***
 * @description: 检测的输出缓冲区, 用于存储检测的结果
 * 根据检测结果保存形式为: x y w h score cls_id [optional: pose / seg / obb]
 * @return
 */
class ObjectBuffer
{
   private:
    uint32 max_obj_count = 0;  // 检测框的最大数量
    uint32 stride = 0;         // 检测框的步长
    uint32 extra_dim = 0;      // 检测框的额外维度, 例如 姿态估计 / 分割 / 旋转框 等

    // 核心数据存储
    std::vector<float32> buffer;

    // 有效性标记: 标记对应的 index 是否存有有效目标
    // 使用 uint8 而非 bool 是因为 vector<bool> 在 C++ 中有特殊的行为 (位优化)
    // 在某些并行计算或指针操作时 uint8 表现更符合预期。
    std::vector<uint8> valid_mask;

   public:
    /***
     * @description: 构造函数
     * @param max_obj_count uint32 : 检测框的最大数量
     * @param extra_dim uint32 : 额外维度
     * @return
     */
    ObjectBuffer(uint32 max_obj_count, uint32 extra_dim = 0) : max_obj_count(max_obj_count), extra_dim(extra_dim)
    {
        // 计算每个检测目标信息的步长
        this->stride = ObjectOffset::base_box_len + extra_dim;

        // 初始化缓冲区
        // 计算总的元素数量
        size_t total_elements = static_cast<size_t>(max_obj_count * this->stride);

        // reserve 预留空间, 防止内存重分配
        this->buffer.reserve(total_elements);
        // 数组初始化为0.0, 可以通过索引直接赋值
        this->buffer.resize(total_elements, 0.0f);

        // 预留空间防止 push_back 时的内存重分配, 但 size 初始为 0
        this->valid_mask.reserve(static_cast<size_t>(this->max_obj_count));
    }

    // 禁止各种复制拷贝, 只引用传递
    /***
     * @description: 禁用拷贝构造函数, 防止对象被拷贝
     * @return
     */
    ObjectBuffer(const ObjectBuffer&) = delete;

    /***
     * @description: 禁用赋值操作符, 防止对象被赋值
     * @return
     */
    ObjectBuffer& operator=(const ObjectBuffer&) = delete;

    /***
     * @description: 禁用移动构造函数, 防止对象被移动
     * @return
     */
    ObjectBuffer(const ObjectBuffer&&) = delete;

    /***
     * @description: 禁用移动赋值操作符, 防止对象被移动赋值
     * @return
     */
    ObjectBuffer& operator=(const ObjectBuffer&&) = delete;

    // 只读属性获取
    /***
     * @description: 获取最大检测目标个数
     * @return
     */
    uint32 get_max_count() const { return this->max_obj_count; }

    /***
     * @description: 获取检测目标的步长, 包括所有信息的长度: x y w h score cls_id [optional: pose / seg / obb]
     * @return
     */
    uint32 get_stride() const { return this->stride; }

    /***
     * @description: 获取检测目标个数
     * @return
     */
    inline uint32 get_obj_count() const { return static_cast<uint32>(this->valid_mask.size()); }

    /***
     * @description: 获取有效目标的总数
     * @return
     */
    uint32 get_valid_count() const
    {
        //
        uint32 count = 0;
        for (uint8 v : valid_mask)
        {
            if (v != 0)
                count++;
        }
        return count;
    }

    /***
     * @description: 获取每个检测目标是否有效
     * @param obj_idx uint32 : 检测目标的索引
     * @return bool : 是否有效, true 表示有效, false 表示无效
     */
    bool is_valid(uint32 obj_idx) const
    {
        // 访问元素, 索引必须小于已存在的目标个数
        assert(obj_idx < this->get_obj_count() && "obj_idx out of range");

        // 0 表示 无效, 1 表示 有效
        return this->valid_mask[obj_idx] != 0;
    }

    /***
     * @description: 更改某个检测目标的合法性
     * @param obj_idx uint32 : 检测目标的索引
     * @param valid bool : 是否有效, true 表示有效, false 表示无效
     * @return 没有返回值
     */
    void set_valid(uint32 obj_idx, bool valid)
    {
        // 访问元素, 索引必须小于已存在的目标个数
        assert(obj_idx < this->get_obj_count() && "obj_idx out of range");

        // 0 表示 无效, 1 表示 有效
        this->valid_mask[obj_idx] = valid ? 1 : 0;
    }

    /***
     * @description: 获取第 i 个目标的起始地址 (非 const)
     * @param obj_idx uint32 : 检测目标的索引
     * @return
     */
    float32* at(uint32 obj_idx)
    {
        // 访问元素, 索引必须小于已存在的目标个数
        assert(obj_idx < this->get_obj_count() && "obj_idx out of range");

        return &(this->buffer[obj_idx * this->stride]);
    }

    /***
     * @description: 获取第 i 个目标的起始地址 (const)
     * @param obj_idx uint32 : 检测目标的索引
     * @return
     */
    const float32* at(uint32 obj_idx) const
    {
        // 访问元素, 索引必须小于已存在的目标个数
        assert(obj_idx < this->get_obj_count() && "obj_idx out of range");

        return &(this->buffer[obj_idx * stride]);
    }

    /***
     * @description: 重载 [] 运算符, 获取第 i 个目标的起始地址 (非 const)
     *               这种方式允许你使用 buffer[i][j] 的语法
     * @return
     */
    float32* operator[](uint32 obj_idx)
    {
        // 访问元素, 索引必须小于已存在的目标个数
        assert(obj_idx < this->get_obj_count() && "obj_idx out of range");

        return &(this->buffer[obj_idx * stride]);
    }

    /***
     * @description: 重载 [] 运算符, 获取第 i 个目标的起始地址 (const)
     *               这种方式允许你使用 buffer[i][j] 的语法
     * @return
     */
    const float32* operator[](uint32 obj_idx) const
    {
        // 访问元素, 索引必须小于已存在的目标个数
        assert(obj_idx < this->get_obj_count() && "obj_idx out of range");

        return &(this->buffer[obj_idx * stride]);
    }

    /***
     * @description: 添加一个检测目标到缓冲区最后一个位置
     * @param data float32* : 检测目标的指针, 长度必须和 this->stride 一致, 否则会崩溃
     * @return
     */
    void push_back(const float32* data)
    {
        assert(this->valid_mask.size() < this->max_obj_count && "Buffer overflow");

        if (data != nullptr)
        {
            // 获取当前检测目标个数
            uint32 current_idx = this->get_obj_count();
            // 将数据复制到缓冲区
            std::copy(data, data + this->stride, this->buffer.begin() + (current_idx * this->stride));

            // 标记为有效
            this->valid_mask.push_back(1);
        }
    }

    /***
     * @description: 添加一个检测目标到缓冲区指定位置, 但是不能超过 this->get_obj_count()
     * @param det_idx uint32 : 检测目标的索引, 赋值给指定的位置 可以指定 this->get_obj_count() 表示添加到末尾
     * @param data float32* : 检测目标的指针, 长度必须和 this->stride 一致, 否则会崩溃
     * @return
     */
    void append_at(uint32 det_idx, const float32* data)
    {
        assert(det_idx <= this->get_obj_count() && "det_idx out of range");

        if (data != nullptr)
        {
            if (det_idx == this->get_obj_count())
            {
                // 如果指定位置是末尾, 直接 push_back
                // 如果是新位置, 需要补齐前面的 mask 确保 size 正确
                this->valid_mask.push_back(1);
            }
            else
            {
                // 在中间, 覆盖原来的元素
                this->valid_mask[det_idx] = 1;
            }

            // 将数据复制到缓冲区
            std::copy(data, data + this->stride, this->buffer.begin() + (det_idx * this->stride));
        }
    }

    /***
     * @description: 获取按分数降序排序后的原始索引列表, 只获取实际目标个数的索引
     * @return
     */
    std::vector<uint32> get_sorted_indices() const
    {
        uint32 current_obj_count = this->get_obj_count();
        std::vector<uint32> indices(current_obj_count);

        // 快速生成一个从 0 到 n-1 的索引序列
        std::iota(indices.begin(), indices.end(), 0);  // 填充 0, 1, 2...

        // 按分数降序排序
        std::sort(indices.begin(), indices.end(),
                  [this](uint32 a, uint32 b)
                  {
                      // 这里的 this 是 const ObjectBuffer*
                      // 因此调用的是 const float32* at(uint32 obj_idx) const
                      const float32* data_a = this->at(a);
                      const float32* data_b = this->at(b);

                      // ObjectOffset::score 是 4
                      return data_a[ObjectOffset::score] > data_b[ObjectOffset::score];
                  });

        return indices;
    }

    /***
     * @description: 将缓存区的数据变成紧凑, 删除无效目标, 更快速访问
     * @return
     */
    void compact()
    {
        // 双指针算法, 将有效的元素移动到缓冲区连续位置
        // read_idx 从后往前遍历, write_idx 从前往后遍历
        uint32 read_idx = this->get_obj_count();  // 此时是总长度
        // 如果 read_idx 为 0, 说明缓冲区为空, 直接返回
        if (read_idx == 0)
            return;

        // 之后才是真实的最后一个索引, 索引比总长度小1
        read_idx = read_idx - 1;
        uint32 write_idx = 0;

        // 两个指针相遇时, 说明已经遍历完所有元素
        while (read_idx > write_idx)
        {
            if (!this->is_valid(write_idx) && !this->is_valid(read_idx))
            {
                // 两个都是无效的, 跳过, read_idx 找到一个有效的, 复制到 write_idx 位置
                read_idx--;
            }
            else if (!this->is_valid(write_idx) && this->is_valid(read_idx))
            {
                // write_idx 是无效的, read_idx 是有效的, 复制 read_idx 到 write_idx
                std::copy(this->buffer.begin() + (read_idx * this->stride),
                          this->buffer.begin() + (read_idx * this->stride) + this->stride,
                          this->buffer.begin() + (write_idx * this->stride));

                // 将 write_idx 标记为有效
                this->set_valid(write_idx, true);
                // 将 read_idx 标记为无效
                this->set_valid(read_idx, false);  // 这一步是多余的, 因为 read_idx 已经被复制了, 并且不会被访问到

                // 两个指针移动
                write_idx++;
                read_idx--;
            }
            else
            {
                // write_idx 有效, 不论 read_idx 是否有效, 都跳过, 且仅移动 write_idx
                write_idx++;
            }
        }  // while

        // 逻辑截断, 清空 write_idx 之后无效的元素
        this->valid_mask.resize(write_idx);  // resize 仅更改 size() 不更改其内存大小
    }

    /***
     * @description: 清空整个缓冲区
     * @return
     */
    void clear()
    {
        // 清空缓冲区, buffer中的数据可以保留, 下次直接覆盖就行
        this->valid_mask.clear();

        // 也可以将数据设置为零
        std::fill(this->buffer.begin(), this->buffer.end(), 0.0f);
    }

    /***
     * @description: 打印各种信息
     * @return
     */
    std::string to_string() const
    {
        std::string info = format_string(
            "max_obj_count: %d, obj_count: %d, stride: %d, buffer_size: %d, buffer_data*: %p, valid_mask_size: %d",
            this->max_obj_count, this->get_obj_count(), this->stride, this->buffer.size(), this->buffer.data(),
            this->valid_mask.size());

        return info;
    }
};

/***
 * @description: 重载输出流运算符, 方便打印
 * @return
 */
inline std::ostream& operator<<(std::ostream& os, const ObjectBuffer& obj)
{
    os << obj.to_string();
    return os;
}

}  // namespace yolo

#endif  // !__OBJECTBUFFER__H__