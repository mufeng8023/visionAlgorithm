/***
 * @Author       : gxs
 * @Date         : 2026-04-26 22:14:31
 * @LastEditors  : gxs
 * @LastEditTime : 2026-04-26 22:14:32
 * @FilePath     : /visionAlgorithm/lib/detector/NetOutput.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __NETOUTPUT__H__
#define __NETOUTPUT__H__

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include "types.hpp"  // 类型定义

namespace yolo
{
class NetOutput
{
   private:
    // 这几个数据不对外开放, 初始化之后仅是可读的
    uint32 batch_size = 0;  // 网络输出特征图的batch
    uint32 channel = 0;     // channel通道数
    uint32 height = 0;      // 特征图高度
    uint32 width = 0;       // 特征图宽度

    // 存储特征的内存空间, 使用这个是为了保证内存安全,
    // 我不能很好的做到内存安全, 所以只能这样了
    std::vector<float32> buffer;

   public:
    /***
     * @description: 构造函数, 存储看空间初始化, 空间初始化后不再更改
     * @param batch_size uint32 :
     * @param channel uint32 :
     * @param height uint32 :
     * @param width uint32 :
     * @return
     */
    NetOutput(uint32 batch_size, uint32 channel, uint32 height, uint32 width)
    {
        this->batch_size = batch_size;
        this->channel = channel;
        this->height = height;
        this->width = width;

        // 初始化缓冲区
        // 计算总的元素数量
        size_t total_elements = static_cast<size_t>(batch_size * channel * height * width);
        // 提前申请这么大的内存空间, 避免在run函数中频繁申请内存
        this->buffer.reserve(total_elements);
        // 数组初始化为0.0, 可以通过索引直接赋值
        this->buffer.resize(total_elements, 0.0f);
    }

    /***
     * @description: 默认构造函数, 创建一个空的 NetOutput 对象
     */
    NetOutput() = default;

    /***
     * @description: 拷贝构造函数, 禁用拷贝以避免大内存复制
     *               如需复制, 请使用深拷贝或引用传递
     * @param other const NetOutput& : 被拷贝的对象引用
     */
    NetOutput(const NetOutput& other) = delete;

    /***
     * @description: 拷贝赋值运算符, 禁用拷贝赋值以避免大内存复制
     * @param other const NetOutput& : 被拷贝的对象引用
     * @return NetOutput& : 返回引用以支持链式赋值
     */
    NetOutput& operator=(const NetOutput& other) = delete;

    /***
     * @description: 移动构造函数, 高效转移资源所有权
     * @param other NetOutput&& : 被移动的右值引用
     */
    NetOutput(NetOutput&& other) noexcept
        : batch_size(other.batch_size),
          channel(other.channel),
          height(other.height),
          width(other.width),
          buffer(std::move(other.buffer))
    {
        // 移动完成后重置源对象的成员变量, 防止悬垂访问
        other.batch_size = 0;
        other.channel = 0;
        other.height = 0;
        other.width = 0;
    }

    /***
     * @description: 移动赋值运算符, 高效转移资源所有权
     * @param other NetOutput&& : 被移动的右值引用
     * @return NetOutput& : 返回引用以支持链式赋值
     */
    NetOutput& operator=(NetOutput&& other) noexcept
    {
        // 检查自赋值, 避免不必要操作
        if (this != &other)
        {
            // 转移所有权
            batch_size = other.batch_size;
            channel = other.channel;
            height = other.height;
            width = other.width;

            buffer = std::move(other.buffer);

            // 移动完成后重置源对象的成员变量
            other.batch_size = 0;
            other.channel = 0;
            other.height = 0;
            other.width = 0;
        }
        return *this;
    }

    /***
     * @description: 析构函数, 默认实现
     */
    ~NetOutput() = default;

    /***
     * @description: 使用 float32* data 给 buffer 复制, 设置长度
     * @param data float32* : 输入的 float32* 数据
     * @param length uint32 : 输入数据的长度
     * @return
     */
    void set_data(float32* data, uint32 length)
    {
        // 检查长度是否正确, 长度必须和 batch_size * channel * height * width 一致
        assert(length == this->buffer.size() && "Length of data does not match buffer size");

        // 复制数据
        if (data != nullptr)
        {
            // 使用 std::copy 仅仅覆盖原有内存中的值, 不会触发 vector 的扩容或重分配
            std::copy(data, data + length, this->buffer.begin());
        }
    }

    /***
     * @description: 使用索引访问
     * @param index uint32 : 索引下标
     * @return 返回引用, 可修改原值
     */
    float32& operator[](uint32 index)
    {
        // 检查索引是否越界
        assert(index >= 0 && index < this->buffer.size());

        return this->buffer[index];
    }

    /***
     * @description: 使用索引访问
     * @param index uint32 : 索引下标
     * @return 返回引用, 只读
     */
    const float32& operator[](uint32 index) const
    {
        // 检查索引是否越界
        assert(index >= 0 && index < this->buffer.size());

        return this->buffer[index];
    }

    /**
     * @description: 返回指定索引处元素的副本（不返回引用）
     * @param index uint32 : 索引下标
     * @return float32 : 该位置数值的拷贝
     */
    float32 at(uint32 index) const
    {
        // 越界检查
        // 在 C++ 标准库中, at() 失败通常抛出 std::out_of_range 异常
        // 如果你倾向于使用 assert, 保持一致即可
        assert(index < this->buffer.size() && "Index out of range in at()");

        // 返回副本
        // 因为返回类型是 float32 而不是 float32&, 这里会自动发生值拷贝
        return this->buffer[index];
    }

    /***
     * @description: 获取底层指针, 方便对接 C 风格接口
     * @param output return :
     * @return
     */
    float32* data() { return this->buffer.data(); }

    /***
     * @description: 获取底层指针, 方便对接 C 风格接口, const 版本
     * @param this return :
     * @return
     */
    const float32* data() const { return this->buffer.data(); }

    /***
     * @description: 只读获取Batch
     * @return
     */
    uint32 get_batch_size() const { return this->batch_size; }

    /***
     * @description: 只读获取channel
     * @return
     */
    uint32 get_channel() const { return this->channel; }

    /***
     * @description: 只读获取height
     * @return
     */
    uint32 get_height() const { return this->height; }

    /***
     * @description: 只读获取width
     * @return
     */
    uint32 get_width() const { return this->width; }

    /***
     * @description: 获取缓冲区大小, 方便调试
     * @return
     */
    uint32 get_buffer_size() const { return this->buffer.size(); }

    /***
     * @description: 打印数据信息
     * @return
     */
    std::string to_string() const
    {
        // 将各种信息打印为字符串
        std::string info = format_string(
            "batch_size: %d, channel: %d, height: %d, width: %d, buffer_size: %d, "
            "buffer_capacity: %d, buffer_data*: %p",
            this->batch_size, this->channel, this->height, this->width, this->buffer.size(), this->buffer.capacity(),
            this->buffer.data());

        return info;
    }
};

/***
 * @description: 重载输出流运算符, 方便打印
 * @return
 */
inline std::ostream& operator<<(std::ostream& os, const NetOutput& obj)
{
    os << obj.to_string();
    return os;
}

}  // namespace yolo

#endif  // !__NETOUTPUT__H__