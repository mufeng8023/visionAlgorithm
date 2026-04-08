/**
 * @Author       : gxs
 * @Date         : 2025-03-20 23:01:41
 * @LastEditors  : gxs
 * @LastEditTime : 2025-03-20 23:33:54
 * @FilePath     : /visionAlgorithm/include/common.hpp
 * @Description  :
 * @
 * @Copyright (c) 2025 by gxs, All Rights Reserved.
 **/
#ifndef __COMMON__H__
#define __COMMON__H__

#include <cxxabi.h>
#include <string>
#include <typeinfo>

/**
 * @description: 将编译器编码的类名解码为人类可读的形式, 方便在程序中使用和输出,
 * @return {*} 返回类别 名称
 **/
template <typename T>
std::string get_class_name(const T& object)
{
    int status = 0;
    // abi::__cxa_demangle 是一个特定于 GCC 和 Clang 编译器的函数, 在其他编译器上可能不可用
    char* p = abi::__cxa_demangle(std::string(typeid(object).name()).c_str(), 0, 0, &status);
    std::string ret(p);
    free(p);
    return ret;
}

#endif  // !__COMMON__H__