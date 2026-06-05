# logging.hpp - C++17 日志系统

## 概述

`logging.hpp` 是一个基于 C++17 实现的日志系统, 采用 LogManager 单例管理多个 Logger 实例; 支持多线程、多进程安全, 提供丰富的日志级别和灵活的配置选项;

## 日志级别

| 级别     | 枚举值 | 说明                           |
| -------- | ------ | ------------------------------ |
| Trace    | `0`    | 跟踪级别, 用于最详细的调试信息 |
| Debug    | `1`    | 调试级别                       |
| Info     | `2`    | 信息级别 (默认)                |
| Warn     | `3`    | 警告级别                       |
| Error    | `4`    | 错误级别                       |
| Critical | `5`    | 严重错误级别                   |

可通过 `LOG_COMPILE_LEVEL` 宏在编译时控制日志输出范围, 默认值为 `LOG_COMPILE_LEVEL_DEBUG`;

## 辅助函数

| 函数                      | 说明                                                   |
| ------------------------- | ------------------------------------------------------ |
| `intToLogLevel(value)`    | 将整数值转换为 LogLevel 枚举                           |
| `strToLogLevel(str)`      | 将字符串 ("Trace"/"Debug"/...) 转换为 LogLevel         |
| `logLevelToString(level)` | 将 LogLevel 枚举转换为大写字符串 ("TRACE"/"DEBUG"/...) |

## LoggerConfig 结构体

| 字段              | 类型           | 默认值  | 说明                                 |
| ----------------- | -------------- | ------- | ------------------------------------ |
| `m_folderPath`    | `string`       | `""`    | 日志文件存储文件夹路径               |
| `m_maxFileSizeMB` | `size_t`       | `10`    | 单个日志文件最大大小 (MB)            |
| `m_maxFiles`      | `unsigned int` | `5`     | 最大日志文件数量                     |
| `m_consoleOutput` | `bool`         | `true`  | 是否同时输出到控制台                 |
| `m_customLogName` | `string`       | `""`    | 自定义日志文件名                     |
| `m_level`         | `LogLevel`     | `Info`  | 日志级别                             |
| `m_processSafe`   | `bool`         | `false` | 是否启用多进程安全 (文件锁+系统调用) |

## Logger 类

可实例化的日志记录器, 继承自 `std::enable_shared_from_this<Logger>`;

### 公共方法

| 方法                                | 说明                                     |
| ----------------------------------- | ---------------------------------------- |
| `applyConfig(cfg)`                  | 应用日志配置, 更新参数并重新打开日志文件 |
| `setName(name)`                     | 设置日志器名称                           |
| `setLogLevel(level)`                | 设置日志级别                             |
| `setConsoleOutput(enabled)`         | 设置是否输出到控制台                     |
| `setMaxLogFiles(maxFiles)`          | 设置最大文件数量                         |
| `trace(file, line, format, ...)`    | 输出 Trace 级别日志                      |
| `debug(file, line, format, ...)`    | 输出 Debug 级别日志                      |
| `info(file, line, format, ...)`     | 输出 Info 级别日志                       |
| `warn(file, line, format, ...)`     | 输出 Warn 级别日志                       |
| `error(file, line, format, ...)`    | 输出 Error 级别日志                      |
| `critical(file, line, format, ...)` | 输出 Critical 级别日志                   |

### 核心功能

- **日志消息格式**: `[时间] [日志级别] 文件路径 : 行号 - 消息内容`
- **日志轮转**: 当前文件超过最大大小时自动创建新文件, 并删除最旧的文件
- **进程安全模式**: 使用 POSIX `flock` 加锁 + `write`/`fsync` 系统调用, 确保多进程写入安全
- **文件描述符管理**: 支持通过文件描述符进行底层 I/O 操作

## LogManager 类

单例模式, 管理多个 Logger 实例;

| 方法                      | 说明                      |
| ------------------------- | ------------------------- |
| `getInstance()`           | 获取全局唯一实例          |
| `createLogger(name, cfg)` | 创建/获取指定名称的日志器 |
| `getLogger(name)`         | 根据名称获取日志器        |
| `getDefaultLogger()`      | 获取默认日志器 (懒加载)   |
| `setDefaultLogger(name)`  | 设置默认日志器            |
| `removeLogger(name)`      | 移除指定日志器            |

## LoggerStream / NullLoggerStream

- **LoggerStream**: 流式日志类, 在析构时自动记录日志, 支持 `operator<<` 链式调用
- **NullLoggerStream**: 空日志流类, 所有操作无实际效果, 用于编译时禁用日志

## 宏定义

### 默认日志器宏

| 宏                                  | 说明                           |
| ----------------------------------- | ------------------------------ |
| `LOG_DEFAULT_TRACE(format, ...)`    | Trace 级别日志 (默认日志器)    |
| `LOG_DEFAULT_DEBUG(format, ...)`    | Debug 级别日志 (默认日志器)    |
| `LOG_DEFAULT_INFO(format, ...)`     | Info 级别日志 (默认日志器)     |
| `LOG_DEFAULT_WARN(format, ...)`     | Warn 级别日志 (默认日志器)     |
| `LOG_DEFAULT_ERROR(format, ...)`    | Error 级别日志 (默认日志器)    |
| `LOG_DEFAULT_CRITICAL(format, ...)` | Critical 级别日志 (默认日志器) |

### 指定日志器宏

| 宏                               | 说明                           |
| -------------------------------- | ------------------------------ |
| `LOG_TRACE(logger, fmt, ...)`    | Trace 级别日志 (指定日志器)    |
| `LOG_DEBUG(logger, fmt, ...)`    | Debug 级别日志 (指定日志器)    |
| `LOG_INFO(logger, fmt, ...)`     | Info 级别日志 (指定日志器)     |
| `LOG_WARN(logger, fmt, ...)`     | Warn 级别日志 (指定日志器)     |
| `LOG_ERROR(logger, fmt, ...)`    | Error 级别日志 (指定日志器)    |
| `LOG_CRITICAL(logger, fmt, ...)` | Critical 级别日志 (指定日志器) |

### 流式日志宏

| 宏                                                    | 说明               |
| ----------------------------------------------------- | ------------------ |
| `LOG_DEFAULT_TRACE_OS` / `LOG_TRACE_OS(logger)`       | 流式 Trace 日志    |
| `LOG_DEFAULT_DEBUG_OS` / `LOG_DEBUG_OS(logger)`       | 流式 Debug 日志    |
| `LOG_DEFAULT_INFO_OS` / `LOG_INFO_OS(logger)`         | 流式 Info 日志     |
| `LOG_DEFAULT_WARN_OS` / `LOG_WARN_OS(logger)`         | 流式 Warn 日志     |
| `LOG_DEFAULT_ERROR_OS` / `LOG_ERROR_OS(logger)`       | 流式 Error 日志    |
| `LOG_DEFAULT_CRITICAL_OS` / `LOG_CRITICAL_OS(logger)` | 流式 Critical 日志 |

所有宏均使用 `if constexpr` 在编译时检查日志级别, 未达到编译级别时展开为 `NullLoggerStream`, 零运行时开销;

## 日志配置文件 (log_config.ini)

日志配置通过 INI 文件管理, 使用 `IniParser` 加载并初始化多个 Logger 实例;

```ini
; 日志配置文件示例
[default]

; 默认日志文件名称
custom_log_name = default

; 日志文件存放路径
log_folder = ../logs

; 日志级别: Trace=0, Debug=1, Info=2, Warn=3, Error=4, Critical=5
log_level = 1

; 单个日志文件的最大大小 (MB)
max_file_size_mb = 10

; 最多保留的日志文件数量
max_files = 5

; 是否启用控制台输出日志
enable_console = true

; 是否启用进程安全模式 (多进程环境下使用文件锁)
process_safe = false
```

## 使用示例

```cpp
#include "logging.hpp"
#include "ini_parser.hpp"

// 根据 INI 配置文件初始化日志器
void init_logger(const std::string& ini_path)
{
    IniParser parser;
    parser.load(ini_path);

    std::vector<std::string> sections = parser.get_sections();
    for (const std::string& section : sections)
    {
        LoggerConfig logger_config;

        // 从配置节读取日志参数
        logger_config.m_customLogName = parser.get_string(section, "custom_log_name", section);
        logger_config.m_level = intToLogLevel(parser.get_int(section, "log_level", 2));
        logger_config.m_folderPath = parser.get_string(section, "log_folder",
            std::string("../logs/") + section);
        logger_config.m_maxFileSizeMB = parser.get_int(section, "max_file_size_mb", 10);
        logger_config.m_maxFiles = parser.get_int(section, "max_files", 5);
        logger_config.m_consoleOutput = parser.get_bool(section, "enable_console", false);
        logger_config.m_processSafe = parser.get_bool(section, "process_safe", false);

        // 创建日志器并注册到 LogManager
        LogManager::getInstance().createLogger(logger_config.m_customLogName, logger_config);

        // 如果节名为 "default", 将其设为默认日志器
        if (section == "default")
        {
            LogManager::getInstance().setDefaultLogger(logger_config.m_customLogName);
        }
    }
}

int main(int argc, char* argv[])
{
    // 通过命令行参数传入日志配置文件路径
    init_logger("../log_config.ini");

    // 使用默认日志器记录日志
    LOG_DEFAULT_INFO("Application started, version=%d", 1);

    // 流式日志
    LOG_DEFAULT_INFO_OS << "Stream " << "log " << 123;

    // 使用默认日志器输出各级别日志
    LOG_DEFAULT_DEBUG("这是一个调试信息");
    LOG_DEFAULT_INFO("程序运行正常, count=%d", 42);
    LOG_DEFAULT_WARN("发现潜在问题, %s", "disk usage high");
    LOG_DEFAULT_ERROR("发生错误, code=%d", -1);

    return 0;
}
```

### 日志消息输出格式

日志消息格式为: `[时间] [日志级别] 文件路径 : 行号 - 消息内容`

示例输出:

```bash
2026-06-05 17:59:00 [    INFO] src/main.cpp :    237 - Application started, version=1
2026-06-05 17:59:00 [   DEBUG] src/main.cpp :    242 - 这是一个调试信息
2026-06-05 17:59:00 [    INFO] src/main.cpp :    243 - 程序运行正常, count=42
2026-06-05 17:59:00 [    WARN] src/main.cpp :    244 - 发现潜在问题, disk usage high
2026-06-05 17:59:00 [   ERROR] src/main.cpp :    245 - 发生错误, code=-1
```
