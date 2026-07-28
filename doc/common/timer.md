# timer.hpp - 高性能计时器库

## 概述

`timer.hpp` 提供高性能计时功能, 包含三个核心组件: `Timer` 单次计时器、`TimerManager` 单例计时器管理器和便捷的宏定义;

## 核心类

### Timer 类

单次计时器类, 基于 `std::chrono::high_resolution_clock` 实现高精度计时;

| 方法            | 说明                                          |
| --------------- | --------------------------------------------- |
| `Timer()`       | 构造函数, 创建时自动开始计时                  |
| `reset()`       | 重置计时器, 将开始时间设为当前时刻            |
| `elapsed_ms()`  | 返回从 reset 到现在的毫秒数 (`std::int64_t`)  |
| `elapsed_str()` | 返回格式化时间字符串, 如 `"1h 30m 20s 500ms"` |

### TimerManager 类

单例模式的计时器管理器, 管理多个命名计时器, 支持线程安全访问;

| 方法                | 说明                         |
| ------------------- | ---------------------------- |
| `instance()`        | 获取全局唯一实例             |
| `get(name)`         | 获取/创建指定名称的计时器    |
| `remove(name)`      | 销毁指定名称的计时器         |
| `reset(name)`       | 重置指定名称的计时器         |
| `elapsed_ms(name)`  | 获取指定计时器的毫秒耗时     |
| `elapsed_str(name)` | 获取指定计时器的格式化字符串 |

## 宏定义

### RELEASE 宏 (始终启用)

| 宏                        | 说明                               |
| ------------------------- | ---------------------------------- |
| `TIMER_START(name)`       | 开始/重置一个持久计时器            |
| `TIMER_ELAPSED_STR(name)` | 获取格式化时间字符串, 不销毁计时器 |
| `TIMER_ELAPSED_MS(name)`  | 获取毫秒数, 不销毁计时器           |
| `TIMER_REMOVE(name)`      | 手动销毁计时器, 释放内存           |

### DEBUG 宏 (由 `TIMER_DEBUG` 宏控制开关)

当定义了 `TIMER_DEBUG` 时, `TIMER_START_DEBUG`/`TIMER_ELAPSED_STR_DEBUG`/`TIMER_ELAPSED_MS_DEBUG`/`TIMER_REMOVE_DEBUG` 映射到实际行为; 未定义时全部展开为空操作, 零运行时开销;

## 使用示例

```cpp
#include "timer.hpp"

// 使用 TimerManager
TIMER_START("myTask");
// ... 执行任务 ...
std::cout << "耗时: " << TIMER_ELAPSED_STR("myTask") << std::endl;
TIMER_REMOVE("myTask");

// 使用 Timer 类
Timer t;
// ... 执行操作 ...
std::int64_t ms = t.elapsed_ms();
std::string str = t.elapsed_str();

// Debug 模式计时
#ifdef TIMER_DEBUG
TIMER_START_DEBUG("debug-task");
// ... 调试代码 ...
std::cout << TIMER_ELAPSED_STR_DEBUG("debug-task") << std::endl;
TIMER_REMOVE_DEBUG("debug-task");
#endif
```
