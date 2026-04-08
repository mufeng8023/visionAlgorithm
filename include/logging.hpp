/***
 * @Author       : gxs
 * @Date         : 2025-08-09 01:55:56
 * @LastEditors  : gxs
 * @LastEditTime : 2025-10-27 15:32:16
 * @FilePath     : /logging/include/logging.hpp
 * @Description  : 重构为 LogManager 管理多个 Logger 实例的单例模式
 *                 C++17 实现的日志系统, 支持多日志器、多线程、多进程安全
 *                 主要特性:
 *                 - Logger 类可实例化 (非单例)
 *                 - LogManager 单例管理多个 Logger 实例和默认日志器
 *                 - 线程安全, 支持进程安全的文件写入 (使用 POSIX open/write/flock)
 *                 - 保持向后兼容的宏定义
 * @
 * @Copyright (c) 2025 by gxs, All Rights Reserved.
 */
#ifndef __LOGGING__H__
#define __LOGGING__H__

#include <chrono>         // 时间相关操作
#include <cstdarg>        // 可变参数处理
#include <ctime>          // C风格时间函数
#include <deque>          // 双端队列容器
#include <fcntl.h>        // 文件控制操作
#include <filesystem>     // 文件系统操作 (C++17)
#include <fstream>        // 文件流操作
#include <iomanip>        // 输入输出流格式控制
#include <iostream>       // 标准输入输出流
#include <map>            // 映射容器 (键值对)
#include <memory>         // 智能指针等内存管理
#include <mutex>          // 互斥锁, 用于多线程同步
#include <sstream>        // 字符串流
#include <stdexcept>      // 异常类
#include <string>         // 字符串类
#include <unistd.h>       // POSIX操作系统API
#include <unordered_map>  // 无序映射容器
#include <vector>         // 动态数组容器

#include <sys/file.h>   // 文件操作
#include <sys/stat.h>   // 文件状态信息
#include <sys/types.h>  // 系统数据类型

// 为 std::filesystem 命名空间创建别名, 简化代码书写
namespace fs = std::filesystem;

// ---------- helper: printf-style formatting ----------
// 格式化字符串辅助函数, 提供类似 printf 的功能
/***
 * @description:  格式化字符串的辅助函数, 类似printf的功能, 返回格式化后的字符串
 * @param *fmt {char} :  格式化字符串, 类似printf中的格式化字符串
 * @return {} 返回格式化后的字符串
 */
inline static std::string _format_string(const char* fmt, ...)
{
    // 检查格式化字符串是否为空
    if (!fmt)
        throw std::invalid_argument("format_string: null format string");

    // 使用va_list处理可变参数
    va_list args1;
    va_start(args1, fmt);
    // 计算格式化后字符串所需的长度
    int size = std::vsnprintf(nullptr, 0, fmt, args1);
    va_end(args1);

    // 如果格式化出错, 抛出异常
    if (size < 0)
    {
        throw std::runtime_error(std::string("format_string: formatting error in '") + fmt + "'");
    }

    // 创建对应大小的字符串
    std::string result(static_cast<size_t>(size), '\0');
    // 再次使用va_list处理可变参数, 这次将格式化结果存入字符串
    va_list args2;
    va_start(args2, fmt);
    std::vsnprintf(&result[0], size + 1, fmt, args2);
    va_end(args2);
    return result;
}

#define LOG_LEVEL_COUNT 6  // 定义日志级别的数量
#define LOG_LEVEL_NAMES {"Trace", "Debug", "Info", "Warn", "Error", "Critical"}
// 定义各等级对应的宏常量 (预处理阶段生效, 供编译时判断使用)
#define LOG_COMPILE_LEVEL_TRACE 0     // 对应 LogLevel::Trace
#define LOG_COMPILE_LEVEL_DEBUG 1     // 对应 LogLevel::Debug
#define LOG_COMPILE_LEVEL_INFO 2      // 对应 LogLevel::Info
#define LOG_COMPILE_LEVEL_WARN 3      // 对应 LogLevel::Warn
#define LOG_COMPILE_LEVEL_ERROR 4     // 对应 LogLevel::Error
#define LOG_COMPILE_LEVEL_CRITICAL 5  // 对应 LogLevel::Critical

// 2. 编译时日志等级宏定义 (默认设为DEBUG, 可通过编译命令覆盖)
#ifndef LOG_COMPILE_LEVEL
#define LOG_COMPILE_LEVEL LOG_COMPILE_LEVEL_DEBUG  // 编译时默认等级: DEBUG及以上才编译
#endif

// 日志级别枚举定义, 定义了从最低级别到最高级别的日志等级
/***
 * @description: 定义了不同级别的日志, 从最低级别的Trace到最高级别的Critical
 * @return {}
 */
enum class LogLevel
{
    Trace = LOG_COMPILE_LEVEL_TRACE,        ///< 跟踪级别, 用于最详细的调试信息
    Debug = LOG_COMPILE_LEVEL_DEBUG,        ///< 调试级别, 用于调试信息
    Info = LOG_COMPILE_LEVEL_INFO,          ///< 信息级别, 用于一般性信息
    Warn = LOG_COMPILE_LEVEL_WARN,          ///< 警告级别, 用于警告信息
    Error = LOG_COMPILE_LEVEL_ERROR,        ///< 错误级别, 用于错误信息
    Critical = LOG_COMPILE_LEVEL_CRITICAL,  ///< 严重级别, 用于严重错误信息
};

/***
 * @description: 将 整数值 转换为对应的 LogLevel 枚举值
 * @param value {int} : 整数值
 * @return {}
 */
LogLevel intToLogLevel(int value)
{
    switch (value)
    {
        case static_cast<int>(LogLevel::Trace):
            return LogLevel::Trace;
        case static_cast<int>(LogLevel::Debug):
            return LogLevel::Debug;
        case static_cast<int>(LogLevel::Info):
            return LogLevel::Info;
        case static_cast<int>(LogLevel::Warn):
            return LogLevel::Warn;
        case static_cast<int>(LogLevel::Error):
            return LogLevel::Error;
        case static_cast<int>(LogLevel::Critical):
            return LogLevel::Critical;
        default:
            throw std::invalid_argument("无效的 LogLevel 整数值");
    }
}

/***
 * @description:
 * @param str string& :
 * @return
 */
LogLevel strToLogLevel(const std::string& str)
{
    if (str == "Trace")
        return LogLevel::Trace;
    else if (str == "Debug")
        return LogLevel::Debug;
    else if (str == "Info")
        return LogLevel::Info;
    else if (str == "Warn")
        return LogLevel::Warn;
    else if (str == "Error")
        return LogLevel::Error;
    else if (str == "Critical")
        return LogLevel::Critical;
    else
        throw std::invalid_argument(
            "无效的 LogLevel 字符串值, 请使用 'Trace', 'Debug', 'Info', 'Warn', 'Error', 'Critical' 之一");
}

/***
 * @description: 将 LogLevel 枚举值转换为对应的字符串表示
 * @param level LogLevel :
 * @return
 */
std::string logLevelToString(const LogLevel& level)
{
    switch (level)
    {
        case LogLevel::Trace:
            return "TRACE";
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warn:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
        case LogLevel::Critical:
            return "CRITICAL";
        default:
            return "UNKNOWN";
    }
}

// ---------- LoggerConfig ----------
// 日志配置结构体, 用于配置日志记录器的各项参数
/***
 * @description: 定义了日志记录器的配置参数,
 * 包括日志文件夹路径、
 * 最大文件大小、
 * 最大文件数量、
 * 是否输出到控制台、
 * 自定义日志名称、
 * 日志级别、
 * 是否进程安全等
 * @return {}
 */
struct LoggerConfig
{
    std::string m_folderPath = "";      // 日志文件存储的文件夹路径
    size_t m_maxFileSizeMB = 10;        // MB
    unsigned int m_maxFiles = 5;        // 日志文件的最大数量
    bool m_consoleOutput = true;        // 是否输出到控制台
    std::string m_customLogName = "";   // 自定义日志文件名
    LogLevel m_level = LogLevel::Info;  // 日志级别
    bool m_processSafe = false;         // whether to use file-lock + low-level writes for multi-process safety
    LoggerConfig() = default;
};

// ---------- Logger (instantiable) ----------
// Logger 类定义, 可实例化的日志记录器, 支持多线程和进程安全
/**
 * Logger类, 继承自std::enable_shared_from_this<Logger>
 * 这个类设计用于实现日志记录功能, 并支持通过shared_ptr进行安全地共享所有权
 * 继承std::enable_shared_from_this使得Logger对象可以安全地创建指向自身的shared_ptr
 */
class Logger : public std::enable_shared_from_this<Logger>
{
   public:
    // Public API - 公共接口方法
    /***
     * @description: 构造函数, 初始化日志记录器
     * @return {}
     */
    Logger()
        : m_logLevel(LogLevel::Info),
          m_consoleOutput(true),
          m_maxFileSize(10 * 1024 * 1024),
          m_currentFileSize(0),
          m_processSafe(false),
          m_fd(-1)
    {
    }

    /***
     * @description: 析构函数, 关闭日志文件
     * @return {}
     */
    ~Logger()
    {
        // 关闭日志文件
        this->closeLogFile();
    }

    /***
     * @description:  应用日志配置, 更新日志的各项参数
     * @param &cfg {LoggerConfig} : 日志配置对象, 包含日志路径、文件大小限制、文件数量等配置信息
     * @return {} : 无返回值
     */
    void applyConfig(const LoggerConfig& cfg)
    {
        // 使用互斥锁确保线程安全, 防止多线程同时修改配置
        std::lock_guard<std::mutex> lock(this->m_mutex);
        // 更新日志文件夹路径
        this->m_logFolderPath = cfg.m_folderPath;
        // 将配置中的MB转换为字节, 设置最大文件大小
        this->m_maxFileSize = cfg.m_maxFileSizeMB * 1024 * 1024;
        // 设置最大日志文件数量
        this->m_maxLogFiles = cfg.m_maxFiles;
        // 设置是否输出到控制台
        this->m_consoleOutput = cfg.m_consoleOutput;
        // 设置自定义日志名称
        this->m_customLogName = cfg.m_customLogName;
        // 设置日志级别
        this->m_logLevel = cfg.m_level;
        // 设置是否进程安全
        this->m_processSafe = cfg.m_processSafe;

        // reopen file with new config
        // 检查文件流是否打开或文件描述符是否有效
        if (this->m_fileStream.is_open() || this->m_fd != -1)
        {
            // 如果文件已打开或文件描述符有效, 则关闭日志文件
            this->closeLogFile();
        }
        // 打开日志文件
        this->openLogFile();
    }

    /***
     * @description: 获取日志记录器的名称
     * @return {}
     */
    std::string getName() const
    {
        // 获取日志记录器的名称
        return this->m_name;
    }

    /***
     * @description: 设置日志记录器的名称
     * @param &name {string} :
     * @return {}
     */
    void setName(const std::string& name)
    {
        // 设置日志记录器的名称
        this->m_name = name;
    }

    /***
     * @description: 获取日志记录器的日志级别
     * @param level {LogLevel} :
     * @return {}
     */
    void setLogLevel(LogLevel level)
    {
        // 设置日志记录器的日志级别
        this->m_logLevel = level;
    }

    /***
     * @description: 获取日志记录器是否输出到控制台
     * @param enabled {bool} :
     * @return {}
     */
    void setConsoleOutput(bool enabled) { this->m_consoleOutput = enabled; }

    /***
     * @description: 获取日志记录器的最多文件个数
     * @param maxFiles {unsigned int} :
     * @return {}
     */
    void setMaxLogFiles(unsigned int maxFiles)
    {
        if (maxFiles == 0)
            throw std::invalid_argument("setMaxLogFiles: must be > 0");
        this->m_maxLogFiles = maxFiles;
    }

    // Core log method that accepts already-formatted message
    /***
     * @description: 核心日志方法, 接受已格式化的消息
     * @param level {LogLevel} : 日志级别
     * @param &message {string} : 日志消息
     * @param *file {char} : 文件名
     * @param line {int} : 行号
     * @return {}
     */
    void log(LogLevel level, const std::string& message, const char* file, int line)
    {
        // 检查日志级别, 如果低于设置的日志级别则直接返回, 过滤掉低级别日志
        if (level < this->m_logLevel)
            return;  // filter out

        // 使用互斥锁确保线程安全
        std::lock_guard<std::mutex> lock(this->m_mutex);

        // 检查文件流和文件描述符是否都未打开, 如果是则打开日志文件
        if (!this->m_fileStream.is_open() && this->m_fd == -1)
        {
            this->openLogFile();
        }

        // 格式化日志消息并添加换行符
        std::string formattedMessage = this->formatLogMessage(level, message, file, line);
        formattedMessage.push_back('\n');

        // 如果启用了进程安全的日志写入方式
        // 当 m_processSafe 成员变量为 true 时
        // 这通常在配置中设置, 用于多进程环境下的安全写入
        // 进入此分支后, 会使用文件描述符 (fd) 和系统调用 (write, flock) 进行写入
        if (this->m_processSafe)
        {
            // 检查文件描述符是否有效, 如果无效则尝试重新打开文件描述符
            if (this->m_fd == -1)
            {
                // 尝试重新打开文件描述符
                this->reopenFd();
            }
            // 如果文件描述符有效
            if (this->m_fd != -1)
            {
#ifdef __unix__
                // 文件描述符有效性检查
                if (this->m_fd != -1)  // 检查文件描述符是否有效 (不等于-1表示文件已打开)
                {
                    // 加排他锁, 确保写入操作的原子性
                    ::flock(this->m_fd, LOCK_EX);
                    // 获取格式化消息的大小并转换为ssize_t类型
                    ssize_t to_write = static_cast<ssize_t>(formattedMessage.size());
                    // 获取消息缓冲区的指针
                    const char* buf = formattedMessage.c_str();
                    ssize_t written = 0;  // 已写入字节数计数器
                    // 循环写入, 直到所有数据写入完成或发生错误
                    while (written < to_write)
                    {
                        // 计算每次写入的字节数, 并执行写入操作
                        ssize_t n = ::write(this->m_fd, buf + written, static_cast<size_t>(to_write - written));
                        if (n <= 0)  // 如果写入失败或写入0字节, 则退出循环
                            break;
                        written += n;  // 更新已写入字节数
                    }
                    // 强制将文件数据写入磁盘, 确保数据持久化
                    ::fsync(this->m_fd);
                    // 释放文件锁
                    ::flock(this->m_fd, LOCK_UN);
                    // 更新当前文件大小
                    this->m_currentFileSize += formattedMessage.size();
                }
#endif
            }
        }
        // 当 m_processSafe 为 false 时
        // 这通常用于单进程环境或不需要进程安全性的场景
        // 进入此分支后, 会使用 C++ 的文件流 (ofstream) 进行写入
        else  // 如果条件不满足, 执行以下代码块
        {
            // 检查文件流是否已打开
            if (this->m_fileStream.is_open())
            {
                // 将格式化后的消息写入文件流
                this->m_fileStream << formattedMessage;
                // 刷新文件流, 确保数据写入文件
                this->m_fileStream.flush();
                // 更新当前文件大小, 增加写入消息的大小
                this->m_currentFileSize += formattedMessage.size();
            }
        }

        // 检查是否启用了控制台输出
        if (this->m_consoleOutput)
        {
            // 输出格式化后的消息 (已包含换行符)
            std::cout << formattedMessage;  // already has newline
        }

        // 检查当前文件大小是否超过最大文件大小限制
        if (this->m_currentFileSize >= this->m_maxFileSize)
        {
            // 检查文件大小并执行日志轮转 (如果需要)
            this->checkFileSizeAndRotate();
        }
    }

    // Variadic template formatting interface - 可变参数模板格式化接口
    /***
     * @description: 可变参数模板格式化接口
     * @param *file {char} : 文件名
     * @param line {int} : 行号
     * @param &format {string} : 格式化字符串
     * @param & {Args} : 可变参数
     * @return {}
     */
    template <typename... Args>
    void trace(const char* file, int line, const std::string& format, Args&&... args)
    {
        if (LogLevel::Trace >= this->m_logLevel)
        {
            std::string msg = this->formatString(format, std::forward<Args>(args)...);
            this->log(LogLevel::Trace, msg, file, line);
        }
    }

    /***
     * @description: 调试日志
     * @param *file {char} : 文件名
     * @param line {int} : 行号
     * @param &format {string} : 格式化字符串
     * @param & {Args} : 可变参数
     * @return {}
     */
    template <typename... Args>
    void debug(const char* file, int line, const std::string& format, Args&&... args)
    {
        if (LogLevel::Debug >= this->m_logLevel)
        {
            std::string msg = this->formatString(format, std::forward<Args>(args)...);
            this->log(LogLevel::Debug, msg, file, line);
        }
    }

    /***
     * @description: 信息日志
     * @param *file {char} :  源文件名
     * @param line {int} :  源代码行号
     * @param &format {string} :  日志格式字符串
     * @param & {Args} :  可变参数列表
     * @return {}
     */
    template <typename... Args>
    void info(const char* file, int line, const std::string& format, Args&&... args)
    {
        if (LogLevel::Info >= this->m_logLevel)
        {
            std::string msg = this->formatString(format, std::forward<Args>(args)...);
            this->log(LogLevel::Info, msg, file, line);
        }
    }

    /***
     * @description:  警告日志输出函数, 当日志级别达到警告级别时输出格式化后的警告信息
     * @param *file {char} :  调用该函数的源文件名
     * @param line {int} :  调用该函数的源文件行号
     * @param &format {string} :  格式化字符串, 用于格式化警告信息
     * @param & {Args} :  可变参数列表, 用于格式化字符串
     * @return {} : 无返回值
     */
    template <typename... Args>
    void warn(const char* file, int line, const std::string& format, Args&&... args)
    {
        // 检查当前日志级别是否达到警告级别
        if (LogLevel::Warn >= this->m_logLevel)
        {
            // 使用格式化字符串和可变参数生成完整的警告信息
            std::string msg = this->formatString(format, std::forward<Args>(args)...);
            // 调用日志输出函数, 输出警告级别的日志
            this->log(LogLevel::Warn, msg, file, line);
        }
    }

    /***
     * @description: 错误日志输出函数, 当日志级别达到错误级别时输出格式化后的错误信息
     * @param *file {char} :  源文件名, 用于记录日志产生的源文件位置
     * @param line {int} :  行号, 用于记录日志产生的源文件行号
     * @param &format {string} :  格式化字符串, 用于指定日志输出的格式
     * @param & {Args} :  可变参数列表, 用于格式化字符串中的占位符
     * @return {} : 无返回值
     */
    template <typename... Args>  // 模板参数包, 支持可变数量的参数
    void error(const char* file, int line, const std::string& format, Args&&... args)
    {
        // 检查当前日志级别是否达到错误级别
        if (LogLevel::Error >= this->m_logLevel)
        {
            // 使用完美转发格式化消息
            std::string msg = this->formatString(format, std::forward<Args>(args)...);
            // 调用日志输出函数, 传入错误级别、消息、文件名和行号
            this->log(LogLevel::Error, msg, file, line);
        }
    }

    /***
     * @description: 严重错误日志输出函数, 当日志级别达到严重错误级别时输出格式化后的严重错误信息
     * @param *file {char} :  源文件名指针, 用于记录日志发生的源文件位置
     * @param line {int} :  源文件行号, 用于记录日志发生的源文件行号
     * @param &format {string} :  日志格式化字符串, 用于定义日志输出的格式
     * @param & {Args} :  可变参数列表, 用于向格式化字符串中传递参数
     * @return {} : 无返回值
     */
    template <typename... Args>
    void critical(const char* file, int line, const std::string& format, Args&&... args)
    {
        // 检查当前日志级别是否达到严重错误级别
        if (LogLevel::Critical >= this->m_logLevel)
        {
            // 格式化日志信息
            std::string msg = this->formatString(format, std::forward<Args>(args)...);
            // 调用日志输出函数, 输出严重错误日志
            this->log(LogLevel::Critical, msg, file, line);
        }
    }

   private:
    // Member variables (m_ prefix) - 私有成员变量, 使用 m_ 前缀命名
    LogLevel m_logLevel;                     // 日志级别枚举类型, 控制日志输出的详细程度
    std::ofstream m_fileStream;              // 文件输出流对象, 用于将日志写入文件
    std::string m_logFileName;               // 日志文件名称
    std::string m_logFolderPath;             // 日志文件存储的文件夹路径
    bool m_consoleOutput;                    // 是否在控制台输出日志的标志位
    std::mutex m_mutex;                      // 互斥锁, 保证多线程环境下日志写入的安全
    size_t m_maxFileSize;                    // 单个日志文件的最大大小限制
    std::string m_customLogName;             // 自定义日志名称
    size_t m_currentFileSize;                // 当前日志文件的大小
    unsigned int m_maxLogFiles = 5;          // 最大日志文件数量限制, 默认为5个
    std::deque<std::string> m_logFileQueue;  // 日志文件队列, 用于管理多个日志文件
    std::string m_name = "";                 // 日志名称, 默认为空字符串
    bool m_processSafe;                      // 是否启用进程安全模式
    int m_fd;                                // 文件描述符, 用于进程安全的文件操作

    /***
     * @description: 格式化字符串, 将可变参数列表中的参数插入到格式化字符串中
     *                使用模板和完美转发实现类型安全的格式化
     * @param &format {string} :  格式化字符串, 包含占位符
     * @param & { T} :  可变参数列表中的第一个参数
     * @param & {Args} :  可变参数列表中的剩余参数
     * @return {} 返回格式化后的字符串, 如果格式化失败则返回原始格式字符串
     */
    template <typename T, typename... Args>
    std::string formatString(const std::string& format, T&& value, Args&&... args) noexcept
    {
        try
        {
            return _format_string(format.c_str(), std::forward<T>(value), std::forward<Args>(args)...);
        }
        catch (...)
        {
            return format;
        }
    }

    /***
     * @description: 无可变参数的格式化字符串重载函数
     *                当没有额外参数时直接返回原始格式字符串
     * @param &format {string} : 字符串, 不包含占位符
     * @return {} 返回原始格式字符串
     */
    std::string formatString(const std::string& format) noexcept { return format; }

    /***
     * @description: 格式化日志消息, 将日志级别、消息、文件路径和行号组合成统一的日志格式
     *                格式: [时间] [日志级别] 文件路径 : 行号 - 消息内容
     *                如果定义了 PROJECT_ROOT 宏, 会计算相对于项目根目录的文件路径
     * @param level {LogLevel} : 日志级别 (如INFO、ERROR等)
     * @param &message {string} : 日志消息内容
     * @param *file {char} : 源文件路径
     * @param line {int} : 源文件中的行号
     * @return {} : 返回格式化后的日志字符串
     */
    std::string formatLogMessage(LogLevel level, const std::string& message, const char* file, int line)
    {
        std::ostringstream oss;               // 使用字符串流来构建日志消息
        std::string relativeFilePath = file;  // 初始化文件路径为传入的完整路径

        // 如果定义了PROJECT_ROOT宏, 则尝试获取相对于项目根目录的文件路径
#ifdef PROJECT_ROOT
        try
        {
            // 获取项目根目录路径
            fs::path projectRoot = PROJECT_ROOT;
            // 将传入的文件路径转换为fs::path对象
            fs::path filePath(file);
            // 计算相对路径
            relativeFilePath = filePath.lexically_relative(projectRoot).string();
        }
        catch (...)  // 如果计算相对路径时出现异常, 则使用原始路径
        {
            relativeFilePath = file;
        }
#endif
        // 构建日志消息格式: 时间 [日志级别] 文件路径 : 行号 - 消息内容
        oss << this->getCurrentTime() << " [" << std::setw(8) << std::right << this->getLogLevelString(level) << "] "
            << relativeFilePath << " : " << std::setw(6) << std::right << line << " - " << message;

        return oss.str();  // 返回格式化后的日志字符串
    }

    /***
     * @description: 获取日志级别的字符串表示
     *                将 LogLevel 枚举值转换为对应的字符串标识
     * @param level {LogLevel} : 日志级别枚举值
     * @return {} 返回日志级别的字符串表示 (如 "INFO", "ERROR" 等)
     */
    std::string getLogLevelString(LogLevel level) { return logLevelToString(level); }

    /***
     * @description: 获取当前时间, 格式为YYYY-MM-DD HH:MM:SS
     *                使用 std::localtime 和 std::put_time 格式化时间
     * @return {} 返回格式化的当前时间字符串
     */
    std::string getCurrentTime()
    {
        // 获取当前时间戳
        std::time_t now = std::time(nullptr);
        // 将时间戳转换为本地时间结构
        std::tm* localTime = std::localtime(&now);
        // 创建字符串流对象用于格式化时间
        std::ostringstream oss;
        // 将时间格式化为"年-月-日 时:分:秒"的格式
        oss << std::put_time(localTime, "%Y-%m-%d %H:%M:%S");
        // 返回格式化后的时间字符串
        return oss.str();
    }

    /***
     * @description: 根据当前时间生成日志文件名
     *                文件名格式: log_[自定义名称]_时间戳_进程ID.log
     *                如果未设置自定义名称, 则格式: log_时间戳_进程ID.log
     * @return {} 返回生成的日志文件完整路径, 如果未设置日志文件夹路径则只返回文件名
     */
    std::string generateLogFileName()
    {
        // 获取当前时间
        std::time_t now = std::time(nullptr);
        // 将时间转换为本地时间格式
        std::tm* localTime = std::localtime(&now);
        // 创建字符串流用于格式化时间
        std::ostringstream oss;
        // 将时间格式化为年月日时分秒的字符串形式
        oss << std::put_time(localTime, "%Y%m%d%H%M%S");
        // 获取格式化后的时间戳字符串
        std::string timestamp = oss.str();
        // 获取当前进程ID
        pid_t pid = ::getpid();
        std::string fileName;
        // 如果设置了自定义日志名称, 则将其包含在文件名中
        if (!this->m_customLogName.empty())
        {
            fileName = "log_" + this->m_customLogName + "_" + timestamp + "_" + std::to_string(pid) + ".log";
        }
        else
        {
            // 如果未设置自定义日志名称, 则只使用时间戳和进程ID
            fileName = "log_" + this->m_name + timestamp + "_" + std::to_string(pid) + ".log";
        }
        // 如果设置了日志文件夹路径
        if (!this->m_logFolderPath.empty())
        {
            // 创建文件夹路径对象
            fs::path folder(this->m_logFolderPath);
            // 检查文件夹是否存在, 不存在则创建
            if (!fs::exists(folder))
                fs::create_directories(folder);
            // 组合完整路径
            fs::path full = folder / fileName;
            // 返回完整路径字符串
            return full.string();
        }
        // 如果未设置日志文件夹路径, 直接返回文件名
        return fileName;
    }

    /***
     * @description:  打开日志文件的主函数, 根据是否进程安全选择不同的打开方式
     *                进程安全模式使用文件描述符和系统调用
     *                非进程安全模式使用 C++ 文件流
     * @return {} 无返回值, 如果打开失败会抛出异常
     */
    void openLogFile()
    {
        // 检查并创建日志文件夹 (如果不存在)
        if (!this->m_logFolderPath.empty())
        {
            fs::path folder(this->m_logFolderPath);
            if (!fs::exists(folder))
                // 创建多级目录
                fs::create_directories(folder);
        }

        // 生成日志文件名
        this->m_logFileName = this->generateLogFileName();

        // 根据线程安全标志选择不同的文件打开方式
        if (this->m_processSafe)
        {
            // 使用文件描述符方式打开 (进程安全)
            this->reopenFd();
            if (this->m_fd != -1)
            {
                // 获取文件大小
                struct stat st;
                if (fstat(this->m_fd, &st) == 0)
                {
                    this->m_currentFileSize = static_cast<size_t>(st.st_size);
                }
                // 将文件名加入队列并维护最大文件数量
                this->m_logFileQueue.push_back(this->m_logFileName);
                this->maintainMaxLogFiles();
            }
            else
            {
                // 文件打开失败, 抛出异常
                throw std::runtime_error("Unable to open log file (fd) : " + this->m_logFileName);
            }
        }
        else
        {
            // 使用文件流方式打开 (非进程安全)
            this->m_fileStream.open(this->m_logFileName, std::ios::out | std::ios::app);
            if (!this->m_fileStream.is_open())
            {
                // 文件打开失败, 抛出异常
                throw std::runtime_error("Unable to open log file: " + this->m_logFileName);
            }
            // 获取文件大小
            struct stat st;
            if (stat(this->m_logFileName.c_str(), &st) == 0)
            {
                this->m_currentFileSize = static_cast<size_t>(st.st_size);
            }
            // 将文件名加入队列并维护最大文件数量
            this->m_logFileQueue.push_back(this->m_logFileName);
            this->maintainMaxLogFiles();
        }
    }

    /***
     * @description: 重新打开文件描述符的函数, 用于日志文件的重开操作
     *                在进程安全模式下使用, 关闭现有文件描述符并重新打开
     *                仅在 Unix 系统下有效
     * @return {} 无返回值
     */
    void reopenFd()
    {
        //  检查文件描述符是否有效 (不等于-1表示有效)
        if (this->m_fd != -1)
        {
            // 如果文件描述符有效, 则关闭对应的文件
            ::close(this->m_fd);
            // 将文件描述符重置为-1, 表示无效状态
            this->m_fd = -1;
        }
        // 设置文件打开标志: 只写、追加模式、如果文件不存在则创建
        int flags = O_WRONLY | O_APPEND | O_CREAT;
#ifdef __unix__
        // 如果是Unix系统, 尝试以指定标志打开日志文件     设置文件权限为0644 (所有者可读写, 其他用户只读)
        this->m_fd = ::open(this->m_logFileName.c_str(), flags, 0644);
#else
        // 如果不是Unix系统 (如Windows) , 将文件描述符设置为-1, 表示不打开文件
        this->m_fd = -1;
#endif
    }

    /***
     * @description: 关闭日志文件
     *                关闭文件流和文件描述符, 释放资源
     * @return {} 无返回值
     */
    void closeLogFile()
    {
        if (this->m_fileStream.is_open())
        {
            this->m_fileStream.close();
        }
        if (this->m_fd != -1)
        {
            ::close(this->m_fd);
            this->m_fd = -1;
        }
    }

    /***
     * @description: 检查文件大小并进行日志轮转
     *                当日志文件大小超过限制时, 关闭当前文件并创建新文件
     *                实现日志文件的自动轮转功能
     * @return {} 无返回值
     */
    void checkFileSizeAndRotate()
    {
        // 检查当前文件大小是否小于最大文件大小, 如果是则直接返回
        if (this->m_currentFileSize < this->m_maxFileSize)
            return;

        // 检查文件流是否打开或文件描述符是否有效
        if (this->m_fileStream.is_open() || this->m_fd != -1)
        {
            // 如果文件已打开, 则关闭当前日志文件
            this->closeLogFile();
        }

        // 打开新的日志文件
        this->openLogFile();
        // 重置当前文件大小为0
        this->m_currentFileSize = 0;
    }

    /***
     * @description:维护日志文件数量, 确保不超过最大值
     *                当日志文件数量超过预设的最大值时, 删除最旧的日志文件
     *                使用双端队列管理日志文件, 先进先出删除策略
     * @return {} 无返回值
     */
    void maintainMaxLogFiles()
    {
        // 当日志文件队列大小超过最大日志文件数量时, 执行删除操作
        while (this->m_logFileQueue.size() > this->m_maxLogFiles)
        {
            try
            {
                // 获取队列中最旧的日志文件名
                std::string oldest = this->m_logFileQueue.front();
                // 检查文件是否存在
                if (fs::exists(oldest))
                    // 如果文件存在, 则删除该文件
                    fs::remove(oldest);
                // 从队列中移除已删除的文件名
                this->m_logFileQueue.pop_front();
            }
            // 捕获所有可能的异常, 如果发生异常则退出循环
            catch (...)
            {
                break;
            }
        }
    }
};

// ---------- LogManager (singleton) ----------
// 日志管理器类, 单例模式, 管理多个 Logger 实例
class LogManager
{
   public:
    /***
     * @description: 获取单例实例
     *                使用局部静态变量实现线程安全的单例模式
     * @return {} 返回 LogManager 的单例引用
     */
    static LogManager& getInstance()
    {
        // 使用局部静态变量, C++11保证线程安全
        static LogManager instance;  // 创建静态实例, 首次调用时初始化
        return instance;             // 返回实例引用
    }

    /***
     * @description: 创建一个日志实例
     *                如果指定名称的日志器已存在, 则返回现有实例
     *                否则创建新的日志器并应用配置
     * @param &name {string} : 日志器名称
     * @param &cfg {LoggerConfig} : 日志配置对象
     * @return {} 返回共享指针指向新创建或现有的日志器
     */
    std::shared_ptr<Logger> createLogger(const std::string& name, const LoggerConfig& cfg)
    {
        // 获取互斥锁, 确保线程安全
        std::lock_guard<std::mutex> lock(this->m_mutex);
        // 在映射表中查找指定名称的日志器
        auto it = this->m_loggers_map.find(name);
        // 如果找到已存在的日志器, 直接返回
        if (it != this->m_loggers_map.end())
            return it->second;  // 返回现有的日志器共享指针

        // 创建新的日志器实例
        // note: 这里使用智能指针管理日志器对象, 避免内存泄漏
        std::shared_ptr<Logger> logger_shared = std::make_shared<Logger>();
        // 设置日志器名称
        logger_shared->setName(name);
        // 应用配置参数
        logger_shared->applyConfig(cfg);
        // 将新日志器添加到映射表中
        this->m_loggers_map[name] = logger_shared;

        // 如果还没有设置默认日志器, 将当前创建的日志器设为默认
        if (this->m_defaultLoggerName.empty())
        {
            this->m_defaultLoggerName = name;  // 设置默认日志器名称
        }
        // 返回新创建的日志器共享指针
        return logger_shared;
    }

    /***
     * @description: 根据名称获取日志器
     *                在日志器映射表中查找指定名称的日志器
     * @param &name {string} : 日志器名称
     * @return {} 返回共享指针指向找到的日志器, 如果未找到则返回空指针
     */
    std::shared_ptr<Logger> getLogger(const std::string& name)
    {
        // 获取互斥锁, 确保线程安全
        std::lock_guard<std::mutex> lock(this->m_mutex);
        // 在映射表中查找指定名称的日志器
        auto it = this->m_loggers_map.find(name);
        // 如果找到日志器, 返回共享指针
        if (it != this->m_loggers_map.end())
            return it->second;  // 返回找到的日志器
        // 未找到日志器, 返回空指针
        return nullptr;
    }

    /***
     * @description: 获取默认日志器
     *                如果默认日志器不存在, 则懒加载创建默认配置的日志器
     * @return {} 返回共享指针指向默认日志器
     */
    std::shared_ptr<Logger> getDefaultLogger()
    {
        // 获取互斥锁, 确保线程安全
        std::lock_guard<std::mutex> lock(this->m_mutex);
        // 检查是否已设置默认日志器名称
        if (!this->m_defaultLoggerName.empty())
        {
            // 根据默认日志器名称在映射表中查找
            auto it = this->m_loggers_map.find(this->m_defaultLoggerName);
            // 如果找到默认日志器, 返回共享指针
            if (it != this->m_loggers_map.end())
                return it->second;  // 返回默认日志器, 共享指针
        }

        // 懒加载创建默认日志器 (如果不存在)
        // 创建默认配置对象
        LoggerConfig cfg;
        cfg.m_folderPath = "./logs";      // 设置日志文件夹路径
        cfg.m_maxFileSizeMB = 10;         // 设置最大文件大小10MB
        cfg.m_maxFiles = 5;               // 设置最大文件数量5个
        cfg.m_consoleOutput = true;       // 启用控制台输出
        cfg.m_customLogName = "default";  // 设置自定义日志名称
        cfg.m_processSafe = false;        // 禁用进程安全模式
        // 创建新的日志器实例
        std::shared_ptr<Logger> logger_shared = std::make_shared<Logger>();
        // 设置日志器名称为"default"
        logger_shared->setName("default");
        // 应用默认配置
        logger_shared->applyConfig(cfg);
        // 将默认日志器添加到映射表中
        this->m_loggers_map["default"] = logger_shared;
        // 设置默认日志器名称
        this->m_defaultLoggerName = "default";
        // 返回默认日志器共享指针
        return logger_shared;
    }

    /***
     * @description: 设置默认日志器
     *                将指定名称的日志器设置为默认日志器
     * @param &name {string} : 要设置为默认的日志器名称
     * @return {} 无返回值, 如果日志器不存在会抛出异常
     */
    void setDefaultLogger(const std::string& name)
    {
        // 获取互斥锁, 确保线程安全
        std::lock_guard<std::mutex> lock(this->m_mutex);
        // 检查指定名称的日志器是否存在
        if (this->m_loggers_map.find(name) == this->m_loggers_map.end())
        {
            // 日志器不存在, 抛出异常
            throw std::runtime_error("setDefaultLogger: logger not found: " + name);
        }
        // 设置默认日志器名称
        this->m_defaultLoggerName = name;
    }

    /***
     * @description: 移除指定名称的日志器
     *                从日志器映射表中删除指定名称的日志器
     *                如果被删除的是默认日志器, 则清空默认日志器名称
     * @param &name {string} : 要移除的日志器名称
     * @return {} 无返回值
     */
    void removeLogger(const std::string& name)
    {
        // 获取互斥锁, 确保线程安全
        std::lock_guard<std::mutex> lock(this->m_mutex);
        // 在映射表中查找指定名称的日志器
        auto it = this->m_loggers_map.find(name);
        // 如果找到日志器
        if (it != this->m_loggers_map.end())
        {
            // 从映射表中删除日志器
            this->m_loggers_map.erase(it);
            // 如果被删除的是默认日志器, 清空默认日志器名称
            if (this->m_defaultLoggerName == name)
                // 清空默认日志器名称
                this->m_defaultLoggerName.clear();
        }
    }

   private:
    /***
     * @description:日志管理器类构造函数
     *                私有构造函数, 确保单例模式
     * @return {} 无返回值
     */
    LogManager() {}  // 私有构造函数, 防止外部实例化

    /***
     * @description: 日志管理器类析构函数
     *                私有析构函数, 确保单例模式
     *                由于使用了智能指针, 析构函数不需要手动释放资源
     * @return {} 无返回值
     */
    ~LogManager() {}  // 私有析构函数

    /***
     * @description:拷贝构造函数 (已删除)
     *                防止通过拷贝构造函数创建新的日志管理器实例
     *                确保单例模式的唯一性
     * @param & {LogManager} : 其他 LogManager 实例 (未使用)
     * @return {} 无返回值
     */
    LogManager(const LogManager&) = delete;

    /***
     * @description: 赋值运算符 (已删除)
     *                防止通过赋值操作创建新的日志管理器实例
     *                确保单例模式的唯一性
     * @return {} 无返回值
     */
    LogManager& operator=(const LogManager&) = delete;

    // 使用unordered_map存储日志器名称到日志器对象的共享指针映射
    // 键为日志器名称(string类型), 值为日志器对象的共享指针
    std::unordered_map<std::string, std::shared_ptr<Logger>> m_loggers_map;

    // 互斥锁, 用于保护多线程环境下对共享资源的安全访问
    std::mutex m_mutex;

    // 默认日志器的名称, 用于快速访问默认日志器
    std::string m_defaultLoggerName;
};

/***
 * @description: 日志流类, 用于方便地记录日志信息
 * 该类通过操作符重载支持类似cout的日志记录方式, 在析构时自动记录日志
 * @return {}
 */
class LoggerStream
{
   public:
    /***
     * @description: 日志流类构造函数
     * @param logger {shared_ptr<Logger>} : 日志器实例的共享指针
     * @param level {LogLevel} : 日志级别
     * @param *file {char} : 文件名
     * @param line {int} : 行号
     * @return {}
     */
    LoggerStream(std::shared_ptr<Logger> logger, LogLevel level, const char* file, int line)
        : logger_(logger), level_(level), file_(file), line_(line)
    {
    }

    /***
     * @description: 日志流类析构函数, 在析构时记录日志
     * @return {}
     */
    ~LoggerStream()
    {
        if (this->logger_)
        {
            this->logger_->log(this->level_, this->oss_.str().c_str(), this->file_, this->line_);
        }
    }

    /***
     * @description: 重载左移操作符, 用于将数据写入日志流
     * @return {}
     */
    template <typename T>
    LoggerStream& operator<<(const T& value)
    {
        oss_ << value;
        return *this;
    }

   private:
    // 共享指针, 用于指向日志记录器(Logger)对象
    std::shared_ptr<Logger> logger_;
    // 日志级别, 用于控制日志输出的详细程度
    LogLevel level_;
    // 记录日志的源文件名, 用于标识日志产生的位置
    const char* file_;
    // 记录日志的源文件行号, 用于精确定位日志产生的位置
    int line_;
    // 字符串输出流, 用于构建日志消息内容
    std::ostringstream oss_;
};

/***
 * @description: 空日志流类, 用于禁用日志记录
 * 该类重载了左移操作符, 但不执行任何操作, 相当于一个“黑洞”
 * @return
 */
class NullLoggerStream
{
   public:
    template <typename T>
    NullLoggerStream& operator<<(T&&)
    {
        return *this;
    }
};

// ---------------------------------- 宏定义部分 ---------------------------
// 日志管理器宏定义
#define LogManagerInstance LogManager::getInstance()
#define LOGGER_DEFAULT LogManager::getInstance().getDefaultLogger()

// 默认日志宏, 使用默认日志记录器
// 这些宏提供了方便的日志记录接口, 自动获取文件名和行号
#define LOG_DEFAULT_TRACE(format, ...)                                        \
    do                                                                        \
    {                                                                         \
        if constexpr (LOG_COMPILE_LEVEL_TRACE >= LOG_COMPILE_LEVEL)           \
        {                                                                     \
            LOGGER_DEFAULT->trace(__FILE__, __LINE__, format, ##__VA_ARGS__); \
        }                                                                     \
    } while (0)
#define LOG_DEFAULT_DEBUG(format, ...)                                        \
    do                                                                        \
    {                                                                         \
        if constexpr (LOG_COMPILE_LEVEL_DEBUG >= LOG_COMPILE_LEVEL)           \
        {                                                                     \
            LOGGER_DEFAULT->debug(__FILE__, __LINE__, format, ##__VA_ARGS__); \
        }                                                                     \
    } while (0)
#define LOG_DEFAULT_INFO(format, ...)                                        \
    do                                                                       \
    {                                                                        \
        if constexpr (LOG_COMPILE_LEVEL_INFO >= LOG_COMPILE_LEVEL)           \
        {                                                                    \
            LOGGER_DEFAULT->info(__FILE__, __LINE__, format, ##__VA_ARGS__); \
        }                                                                    \
    } while (0)
#define LOG_DEFAULT_WARN(format, ...)                                        \
    do                                                                       \
    {                                                                        \
        if constexpr (LOG_COMPILE_LEVEL_WARN >= LOG_COMPILE_LEVEL)           \
        {                                                                    \
            LOGGER_DEFAULT->warn(__FILE__, __LINE__, format, ##__VA_ARGS__); \
        }                                                                    \
    } while (0)
#define LOG_DEFAULT_ERROR(format, ...)                                        \
    do                                                                        \
    {                                                                         \
        if constexpr (LOG_COMPILE_LEVEL_ERROR >= LOG_COMPILE_LEVEL)           \
        {                                                                     \
            LOGGER_DEFAULT->error(__FILE__, __LINE__, format, ##__VA_ARGS__); \
        }                                                                     \
    } while (0)
#define LOG_DEFAULT_CRITICAL(format, ...)                                        \
    do                                                                           \
    {                                                                            \
        if constexpr (LOG_COMPILE_LEVEL_CRITICAL >= LOG_COMPILE_LEVEL)           \
        {                                                                        \
            LOGGER_DEFAULT->critical(__FILE__, __LINE__, format, ##__VA_ARGS__); \
        }                                                                        \
    } while (0)

// 封装普通日志宏, 需要显式指定日志器实例
// 适用于使用多个不同日志器的场景
#define LOG_TRACE(logger_shared, fmt, ...)                                      \
    do                                                                          \
    {                                                                           \
        if constexpr (LOG_COMPILE_LEVEL_TRACE >= LOG_COMPILE_LEVEL)             \
        {                                                                       \
            if (logger_shared)                                                  \
                (logger_shared)->trace(__FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        }                                                                       \
    } while (0)
#define LOG_DEBUG(logger_shared, fmt, ...)                                      \
    do                                                                          \
    {                                                                           \
        if constexpr (LOG_COMPILE_LEVEL_DEBUG >= LOG_COMPILE_LEVEL)             \
        {                                                                       \
            if (logger_shared)                                                  \
                (logger_shared)->debug(__FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        }                                                                       \
    } while (0)
#define LOG_INFO(logger_shared, fmt, ...)                                      \
    do                                                                         \
    {                                                                          \
        if constexpr (LOG_COMPILE_LEVEL_INFO >= LOG_COMPILE_LEVEL)             \
        {                                                                      \
            if (logger_shared)                                                 \
                (logger_shared)->info(__FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        }                                                                      \
    } while (0)
#define LOG_WARN(logger_shared, fmt, ...)                                      \
    do                                                                         \
    {                                                                          \
        if constexpr (LOG_COMPILE_LEVEL_WARN >= LOG_COMPILE_LEVEL)             \
        {                                                                      \
            if (logger_shared)                                                 \
                (logger_shared)->warn(__FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        }                                                                      \
    } while (0)
#define LOG_ERROR(logger_shared, fmt, ...)                                      \
    do                                                                          \
    {                                                                           \
        if constexpr (LOG_COMPILE_LEVEL_ERROR >= LOG_COMPILE_LEVEL)             \
        {                                                                       \
            if (logger_shared)                                                  \
                (logger_shared)->error(__FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        }                                                                       \
    } while (0)
#define LOG_CRITICAL(logger_shared, fmt, ...)                                      \
    do                                                                             \
    {                                                                              \
        if constexpr (LOG_COMPILE_LEVEL_CRITICAL >= LOG_COMPILE_LEVEL)             \
        {                                                                          \
            if (logger_shared)                                                     \
                (logger_shared)->critical(__FILE__, __LINE__, fmt, ##__VA_ARGS__); \
        }                                                                          \
    } while (0)

// 封装流式日志宏, 需要显式指定日志器实例,默认日志器版本
#if LOG_COMPILE_LEVEL_TRACE >= LOG_COMPILE_LEVEL
#define LOG_DEFAULT_TRACE_OS LoggerStream(LOGGER_DEFAULT, LogLevel::Debug, __FILE__, __LINE__)
#else
#define LOG_DEFAULT_TRACE_OS NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_DEBUG >= LOG_COMPILE_LEVEL
#define LOG_DEFAULT_DEBUG_OS LoggerStream(LOGGER_DEFAULT, LogLevel::Debug, __FILE__, __LINE__)
#else
#define LOG_DEFAULT_DEBUG_OS NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_INFO >= LOG_COMPILE_LEVEL
#define LOG_DEFAULT_INFO_OS LoggerStream(LOGGER_DEFAULT, LogLevel::Info, __FILE__, __LINE__)
#else
#define LOG_DEFAULT_INFO_OS NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_WARN >= LOG_COMPILE_LEVEL
#define LOG_DEFAULT_WARN_OS LoggerStream(LOGGER_DEFAULT, LogLevel::Warn, __FILE__, __LINE__)
#else
#define LOG_DEFAULT_WARN_OS NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_ERROR >= LOG_COMPILE_LEVEL
#define LOG_DEFAULT_ERROR_OS LoggerStream(LOGGER_DEFAULT, LogLevel::Error, __FILE__, __LINE__)
#else
#define LOG_DEFAULT_ERROR_OS NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_CRITICAL >= LOG_COMPILE_LEVEL
#define LOG_DEFAULT_CRITICAL_OS LoggerStream(LOGGER_DEFAULT, LogLevel::Critical, __FILE__, __LINE__)
#else
#define LOG_DEFAULT_CRITICAL_OS NullLoggerStream()
#endif

// 封装流式日志宏, 需要显式指定日志器实例
#if LOG_COMPILE_LEVEL_TRACE >= LOG_COMPILE_LEVEL
#define LOG_TRACE_OS(logger_shared) LoggerStream(logger_shared, LogLevel::Trace, __FILE__, __LINE__)
#else
#define LOG_TRACE_OS(logger_shared) NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_DEBUG >= LOG_COMPILE_LEVEL
#define LOG_DEBUG_OS(logger_shared) LoggerStream(logger_shared, LogLevel::Debug, __FILE__, __LINE__)
#else
#define LOG_DEBUG_OS(logger_shared) NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_INFO >= LOG_COMPILE_LEVEL
#define LOG_INFO_OS(logger_shared) LoggerStream(logger_shared, LogLevel::Info, __FILE__, __LINE__)
#else
#define LOG_INFO_OS(logger_shared) NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_WARN >= LOG_COMPILE_LEVEL
#define LOG_WARN_OS(logger_shared) LoggerStream(logger_shared, LogLevel::Warn, __FILE__, __LINE__)
#else
#define LOG_WARN_OS(logger_shared) NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_ERROR >= LOG_COMPILE_LEVEL
#define LOG_ERROR_OS(logger_shared) LoggerStream(logger_shared, LogLevel::Error, __FILE__, __LINE__)
#else
#define LOG_ERROR_OS(logger_shared) NullLoggerStream()
#endif

#if LOG_COMPILE_LEVEL_CRITICAL >= LOG_COMPILE_LEVEL
#define LOG_CRITICAL_OS(logger_shared) LoggerStream(logger_shared, LogLevel::Critical, __FILE__, __LINE__)
#else
#define LOG_CRITICAL_OS(logger_shared) NullLoggerStream()
#endif

#endif  // !__LOGGING__H__
