/*
 * @Author: gxs
 * @Date: 2024-09-22 03:41:28
 * @LastEditors: gxs
 * @LastEditTime: 2024-09-26 02:28:43
 * @FilePath: /yolo/include/common.hpp
 * @Description:
 *
 * Copyright (c) 2024 by gxs, All Rights Reserved.
 */
#ifndef __PRINTFUN__H__
#define __PRINTFUN__H__

#include <array>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

/***
 * @description: 添加打印vector<float>的函数
 * @return {}
 */
std::ostream& operator<<(std::ostream& os, const std::vector<float>& obj)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "[ ";
    for (size_t i = 0; i < obj.size(); ++i)
    {
        ss << obj[i] << ", ";
    }
    ss << " ]";

    std::cout << ss.str();
    return os;
}

/***
 * @description: 添加打印vector<int>的函数
 * @return {}
 */
std::ostream& operator<<(std::ostream& os, const std::vector<int>& obj)
{
    std::stringstream ss;
    ss << "[ ";
    for (size_t i = 0; i < obj.size(); ++i)
    {
        ss << obj[i] << ", ";
    }
    ss << " ]";

    std::cout << ss.str();
    return os;
}

/***
 * @description: 添加打印array<float, 4>的函数
 * @return {}
 */
std::ostream& operator<<(std::ostream& os, const std::array<float, 4>& obj)
{
    std::stringstream ss;
    ss << "[ ";
    for (size_t i = 0; i < obj.size(); ++i)
    {
        ss << obj[i] << ", ";
    }
    ss << " ]";

    std::cout << ss.str();
    return os;
}

/***
 * @description: 添加打印array<int, 4>的函数
 * @return {}
 */
std::ostream& operator<<(std::ostream& os, const std::array<int, 4>& obj)
{
    std::stringstream ss;
    ss << "[ ";
    for (size_t i = 0; i < obj.size(); ++i)
    {
        ss << obj[i] << ", ";
    }
    ss << " ]";

    std::cout << ss.str();
    return os;
}

#endif  // !__PRINTFUN__H__
