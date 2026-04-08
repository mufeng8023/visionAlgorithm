/***
 * @Author       : gxs
 * @Date         : 2025-09-25 23:33:00
 * @LastEditors  : gxs
 * @LastEditTime : 2025-09-25 23:33:00
 * @FilePath     : \visionAlgorithm\include\types.hpp
 * @Description  : 基础数据类型定义头文件
 * @
 * @Copyright (c) 2025 by gxs, All Rights Reserved.
 */
#ifndef __TYPES__H__
#define __TYPES__H__

#include <algorithm>  // 算法函数 (transform等)
#include <cstdarg>
#include <cstdint>
#include <iomanip>  // 输入输出格式化
#include <limits>   // 包含标准库的limits头文件
#include <regex>    // 正则表达式
#include <sstream>  // 字符串流
#include <string>   // 字符串处理
#include <string>
#include <type_traits>  // 类型特征

/**
 * @description: 基础数据类型定义
 * 提供跨平台的固定大小整数和浮点数类型定义
 */

// 8位整数类型
typedef std::uint8_t uint8;  // 无符号8位整数 (0 ~ 255)
typedef std::int8_t int8;    // 有符号8位整数 (-128 ~ 127)

// 8位整数类型的最大值最小值常量
constexpr uint8 UINT8_MAX_VALUE = std::numeric_limits<uint8>::max();  // uint8最大值
constexpr uint8 UINT8_MIN_VALUE = std::numeric_limits<uint8>::min();  // uint8最小值
constexpr int8 INT8_MAX_VALUE = std::numeric_limits<int8>::max();     // int8最大值
constexpr int8 INT8_MIN_VALUE = std::numeric_limits<int8>::min();     // int8最小值

// 16位整数类型
typedef std::uint16_t uint16;  // 无符号16位整数 (0 ~ 65,535)
typedef std::int16_t int16;    // 有符号16位整数 (-32,768 ~ 32,767)

// 16位整数类型的最大值最小值常量
constexpr uint16 UINT16_MAX_VALUE = std::numeric_limits<uint16>::max();  // uint16最大值
constexpr uint16 UINT16_MIN_VALUE = std::numeric_limits<uint16>::min();  // uint16最小值
constexpr int16 INT16_MAX_VALUE = std::numeric_limits<int16>::max();     // int16最大值
constexpr int16 INT16_MIN_VALUE = std::numeric_limits<int16>::min();     // int16最小值

// 32位整数类型
typedef std::uint32_t uint32;  // 无符号32位整数 (0 ~ 4,294,967,295)
typedef std::int32_t int32;    // 有符号32位整数 (-2,147,483,648 ~ 2,147,483,647)

// 32位整数类型的最大值最小值常量
constexpr uint32 UINT32_MAX_VALUE = std::numeric_limits<uint32>::max();  // uint32最大值
constexpr uint32 UINT32_MIN_VALUE = std::numeric_limits<uint32>::min();  // uint32最小值
constexpr int32 INT32_MAX_VALUE = std::numeric_limits<int32>::max();     // int32最大值
constexpr int32 INT32_MIN_VALUE = std::numeric_limits<int32>::min();     // int32最小值

// 64位整数类型
typedef std::uint64_t uint64;  // 无符号64位整数 (0 ~ 18,446,744,073,709,551,615)
typedef std::int64_t int64;    // 有符号64位整数 (-9,223,372,036,854,775,808 ~ 9,223,372,036,854,775,807)

// 64位整数类型的最大值最小值常量
constexpr uint64 UINT64_MAX_VALUE = std::numeric_limits<uint64>::max();  // uint64最大值
constexpr uint64 UINT64_MIN_VALUE = std::numeric_limits<uint64>::min();  // uint64最小值
constexpr int64 INT64_MAX_VALUE = std::numeric_limits<int64>::max();     // int64最大值
constexpr int64 INT64_MIN_VALUE = std::numeric_limits<int64>::min();     // int64最小值

// 浮点数类型
typedef float float32;   // 32位单精度浮点数 (IEEE 754)
typedef double float64;  // 64位双精度浮点数 (IEEE 754)

// 浮点数类型的最大值最小值常量
constexpr float32 FLOAT32_MAX_VALUE = std::numeric_limits<float32>::max();     // float32最大值
constexpr float32 FLOAT32_MIN_VALUE = std::numeric_limits<float32>::lowest();  // float32最小值
constexpr float32 FLOAT32_EPSILON = std::numeric_limits<float32>::epsilon();   // float32精度
constexpr float64 FLOAT64_MAX_VALUE = std::numeric_limits<float64>::max();     // float64最大值
constexpr float64 FLOAT64_MIN_VALUE = std::numeric_limits<float64>::lowest();  // float64最小值
constexpr float64 FLOAT64_EPSILON = std::numeric_limits<float64>::epsilon();   // float64精度

// 布尔类型
typedef bool bool8;  // 8位布尔类型 (true/false)

// 布尔类型的常量
constexpr bool8 BOOL8_TRUE = true;    // bool8真值
constexpr bool8 BOOL8_FALSE = false;  // bool8假值

// 字符类型
typedef char char8;  // 8位字符类型

// 字符类型的最大值最小值常量
constexpr char8 CHAR8_MAX_VALUE = std::numeric_limits<char8>::max();  // char8最大值
constexpr char8 CHAR8_MIN_VALUE = std::numeric_limits<char8>::min();  // char8最小值

/**
 * @description: 类型大小验证宏
 * 用于在编译时验证类型大小是否符合预期
 */
#define STATIC_ASSERT_TYPE_SIZE(type, expected_size) \
    static_assert(sizeof(type) == expected_size, "Size of " #type " is not " #expected_size " bytes")

// 验证类型大小
STATIC_ASSERT_TYPE_SIZE(uint8, 1);
STATIC_ASSERT_TYPE_SIZE(int8, 1);
STATIC_ASSERT_TYPE_SIZE(uint16, 2);
STATIC_ASSERT_TYPE_SIZE(int16, 2);
STATIC_ASSERT_TYPE_SIZE(uint32, 4);
STATIC_ASSERT_TYPE_SIZE(int32, 4);
STATIC_ASSERT_TYPE_SIZE(uint64, 8);
STATIC_ASSERT_TYPE_SIZE(int64, 8);
STATIC_ASSERT_TYPE_SIZE(float32, 4);
STATIC_ASSERT_TYPE_SIZE(float64, 8);
STATIC_ASSERT_TYPE_SIZE(bool8, 1);
STATIC_ASSERT_TYPE_SIZE(char8, 1);

/***
 * @description: 以 printf 风格格式化字符串, 返回格式化后的 std::string
 * @param *fmt {char} : C 风格格式字符串, 不能为 nullptr 支持的格式如 "%d", "%s", "%.2f" 等
 * @param ... 可变参数, 数量应与格式字符串中的占位符匹配
 * @return {std::string} 格式化后的字符串
 * @throws std::invalid_argument 当传入 fmt 为 nullptr 时抛出
 * @throws std::runtime_error 当格式化失败 (参数不足或格式错误) 时抛出
 * @note 该函数兼容 C++11, 使用标准库, 无需外部依赖
 */
inline static std::string format_string(const char* fmt, ...)
{
    // 检查格式字符串是否为空指针, 避免崩溃
    if (!fmt)
    {
        throw std::invalid_argument("format_string: null format string");
    }

    // 第一次调用 vsnprintf, 传入 nullptr 和 0, 获取格式化后的字符串长度
    va_list args1;
    va_start(args1, fmt);
    int32 size = std::vsnprintf(nullptr, 0, fmt, args1);
    va_end(args1);

    // vsnprintf 返回负值表示格式化失败 (参数不足、格式错误等)
    if (size < 0)
    {
        throw std::runtime_error(std::string("format_string: formatting error in '") + fmt +
                                 "' (too few arguments or invalid format string)");
    }

    // 根据格式化后长度, 分配对应大小的 std::string (不含 null 终止符)
    std::string result(size, '\0');

    // 第二次调用 vsnprintf, 将格式化内容写入 std::string 内部缓冲区
    va_list args2;
    va_start(args2, fmt);
    std::vsnprintf(&result[0], size + 1, fmt, args2);  // size + 1 以包含 null 终止符
    va_end(args2);

    // 返回格式化后的字符串
    return result;
}

#endif  // !__TYPES__H__
