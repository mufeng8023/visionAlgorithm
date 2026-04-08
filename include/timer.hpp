/***
 * @Author       : gxs
 * @Date         : 2025-10-22 12:39:39
 * @LastEditors  : gxs
 * @LastEditTime : 2025-10-22 12:39:46
 * @FilePath     : /projectTemplate/include/timer.hpp
 * @Description  : 高性能计时器库, 提供单个计时器、计时器管理器和作用域自动计时器功能
 * @
 * @Copyright (c) 2025 by gxs, All Rights Reserved.
 */
// 头文件保护, 防止重复包含
#ifndef __TIMER__H__
#define __TIMER__H__

// 包含必要的C++标准库头文件
#include <chrono>         // 用于高精度时间测量
#include <iomanip>        // 用于输入输出格式化
#include <iostream>       // 用于标准输入输出
#include <mutex>          // 用于线程安全的互斥锁
#include <sstream>        // 用于字符串流操作
#include <string>         // 用于字符串操作
#include <unordered_map>  // 用于存储命名计时器的映射表

// =====================================================
// 单个计时器类
// =====================================================
class Timer
{
   public:
    /***
     * @description: 构造函数: 创建计时器并立即开始计时
     * @return {}
     */
    Timer()
    {
        this->reset();  // 调用reset方法初始化开始时间
    }

    /***
     * @description: 重置计时器: 将开始时间设置为当前时间
     * @return {}
     */
    void reset()
    {
        // 获取当前高精度时间点作为开始时间
        this->_m_start_time = std::chrono::high_resolution_clock::now();
    }

    /***
     * @description: 获取从reset到现在的毫秒数
     * @return {}
     */
    std::int64_t elapsed_ms() const
    {
        // 获取当前时间点
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        // 计算时间间隔并转换为毫秒
        std::chrono::milliseconds duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - this->_m_start_time);
        // 返回毫秒计数值
        return duration.count();
    }

    /***
     * @description: 获取格式化的时间字符串 (小时:分钟:秒:毫秒)
     * @return {}
     */
    std::string elapsed_str() const
    {
        // 获取总毫秒数
        std::int64_t ms = this->elapsed_ms();

        // 计算小时数: 总毫秒数除以每小时的毫秒数 (1000 * 60 * 60)
        std::int32_t hours = static_cast<std::int32_t>(ms / (1000LL * 60LL * 60LL));
        // 计算剩余毫秒数 (去除小时部分)
        ms %= (1000LL * 60LL * 60LL);

        // 计算分钟数: 剩余毫秒数除以每分钟的毫秒数 (1000 * 60)
        std::int32_t minutes = static_cast<std::int32_t>(ms / (1000LL * 60LL));
        // 计算剩余毫秒数 (去除分钟部分)
        ms %= (1000LL * 60LL);

        // 计算秒数: 剩余毫秒数除以每秒钟的毫秒数 (1000)
        std::int32_t seconds = static_cast<std::int32_t>(ms / 1000LL);
        // 计算剩余的毫秒数
        std::int32_t milliseconds = static_cast<std::int32_t>(ms % 1000LL);

        // 使用字符串流构建格式化的时间字符串
        std::ostringstream oss;
        // 如果小时数大于0, 添加小时部分
        if (hours > 0)
            oss << hours << "h ";
        // 如果分钟数大于0, 添加分钟部分
        if (minutes > 0)
            oss << minutes << "m ";
        // 如果秒数大于0, 添加秒部分
        if (seconds > 0)
            oss << seconds << "s ";
        // 添加毫秒部分
        oss << milliseconds << "ms";

        // 返回构建的时间字符串
        return oss.str();
    }

   private:
    // 私有成员: 存储计时开始的时间点
    std::chrono::high_resolution_clock::time_point _m_start_time;
};

// =====================================================
// 计时器管理器 (单例模式)
// =====================================================
class TimerManager
{
   public:
    /***
     * @description: 获取单例实例的静态方法
     * @return {}
     */
    static TimerManager& instance()
    {
        // 静态局部变量, 保证线程安全且只初始化一次
        static TimerManager instance;
        return instance;
    }

    /***
     * @description: 获取一个命名计时器 (如果不存在则创建)
     * @param &name {string} : 计时器名称
     * @return {}
     */
    Timer& get(const std::string& name)
    {
        // 使用互斥锁保证线程安全
        std::lock_guard<std::mutex> lock(this->_m_mutex);
        // 返回指定名称的计时器引用 (如果不存在会自动创建)
        return this->timers_map[name];
    }

    /***
     * @description: 重置指定名称的计时器
     * @param &name {string} : 计时器名称
     * @return {}
     */
    void reset(const std::string& name)
    {
        // 使用互斥锁保证线程安全
        std::lock_guard<std::mutex> lock(this->_m_mutex);
        // 调用指定计时器的reset方法
        this->timers_map[name].reset();
    }

    /***
     * @description: 获取指定名称计时器的耗时 (毫秒)
     * @param &name {string} : 计时器名称
     * @return {}
     */
    std::int64_t elapsed_ms(const std::string& name)
    {
        // 使用互斥锁保证线程安全
        std::lock_guard<std::mutex> lock(this->_m_mutex);
        // 返回指定计时器的毫秒耗时
        return this->timers_map[name].elapsed_ms();
    }

    /***
     * @description: 获取指定名称计时器的耗时 (格式化字符串)
     * @param &name {string} : 计时器名称
     * @return {}
     */
    std::string elapsed_str(const std::string& name)
    {
        // 使用互斥锁保证线程安全
        std::lock_guard<std::mutex> lock(this->_m_mutex);
        // 返回指定计时器的格式化时间字符串
        return this->timers_map[name].elapsed_str();
    }

   private:
    // 私有构造函数: 防止外部实例化
    TimerManager() {}
    // 删除拷贝构造函数: 防止复制单例实例
    TimerManager(const TimerManager&) = delete;
    // 删除赋值运算符: 防止赋值单例实例
    TimerManager& operator=(const TimerManager&) = delete;

    // 私有成员: 存储命名计时器的映射表
    std::unordered_map<std::string, Timer> timers_map;
    // 私有成员: 用于线程安全的互斥锁
    std::mutex _m_mutex;
};

// =====================================================
// 作用域自动计时器 (RAII模式)
// =====================================================
class ScopedTimer
{
   public:
    /***
     * @description: 显式构造函数: 创建作用域计时器并记录开始时间
     * @param &name {string} : 计时器名称
     * @return {}
     */
    explicit ScopedTimer(const std::string& name)
        : name_(name),                                            // 初始化计时器名称
          start_time_(std::chrono::high_resolution_clock::now())  // 初始化开始时间
    {
    }

    /***
     * @description: 析构函数: 在对象离开作用域时自动计算并输出耗时
     * @return {}
     */
    ~ScopedTimer()
    {
        // 获取结束时间点
        std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        // 计算时间间隔并转换为毫秒
        std::chrono::milliseconds duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - this->start_time_);
        // 输出计时结果到标准输出
        std::cout << "[TIMER] " << this->name_ << " took " << duration.count() << " ms" << std::endl;
    }

   private:
    // 私有成员: 存储计时器名称
    std::string name_;
    // 私有成员: 存储开始时间点
    std::chrono::high_resolution_clock::time_point start_time_;
};

// =====================================================
// 宏定义: 作用域自动计时器
// =====================================================
// 宏定义: 创建一个作用域自动计时器, 使用行号确保变量名唯一
#define TIME_SCOPE(name) ScopedTimer scoped_timer_##__LINE__(name)

// 结束头文件保护
#endif  // __TIMER__H__
