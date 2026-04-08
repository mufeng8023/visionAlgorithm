/*
 * @Author: gxs
 * @Date: 2024-12-31 09:47:38
 * @LastEditors: gxs
 * @LastEditTime: 2024-12-31 09:58:56
 * @FilePath: /visionAlgorithm/include/myFilesystem.h
 * @Description: //note:在filesystem之外包一层, 因为在C++17之后filesysytem才是在g++编译器之内的,
 *
 * Copyright (c) 2024 by gxs, All Rights Reserved.
 */
#ifndef __MYFILESYSTEM__H__
#define __MYFILESYSTEM__H__

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;  // 命名空间别名

namespace myfs
{
/***
 * @description: 连接两个路径
 * @param &path1 {string} : 第一个路径
 * @param &path2 {string} : 第二个路径
 * @return {string} : 连接后的路径
 */
inline std::string path_join(const std::string& path1, const std::string& path2)
{
    // 使用 std::filesystem::path 的 / 运算符来连接路径
    return (fs::path(path1) / path2).string();
}

/***
 * @description: 连接两个路径
 * @param &path1 {path} : 第一个路径
 * @param &path2 {path} : 第二个路径
 * @return {path} : 连接后的路径
 */
inline fs::path path_join(const fs::path& path1, const fs::path& path2)
{
    // 直接使用 fs::path 的 / 运算符
    return path1 / path2;
}

/***
 * @description: 检查路径是否存在
 * @param &path {string} : 要检查的路径
 * @return {bool} : 如果路径存在则返回 true, 否则返回 false
 */
inline bool path_exists(const std::string& path)
{
    // 使用 std::filesystem::exists 检查
    std::error_code ec;
    bool exists = fs::exists(fs::path(path), ec);
    return !ec && exists;
}

/***
 * @description: 检查路径是否存在
 * @param &path {path} : 要检查的路径
 * @return {bool} : 如果路径存在则返回 true, 否则返回 false
 */
inline bool path_exists(const fs::path& path)
{
    // 使用 std::filesystem::exists 检查
    std::error_code ec;
    bool exists = fs::exists(path, ec);
    return !ec && exists;
}

/***
 * @description: 创建目录, 包括所有不存在的父目录
 * @param &path {string} : 要创建的目录路径
 * @return {bool} : 如果成功创建则返回 true
 */
inline bool makedirs(const std::string& path)
{
    // 使用 std::filesystem::create_directories
    std::error_code ec;
    fs::create_directories(fs::path(path), ec);
    return !ec;
}

/***
 * @description: 创建目录, 包括所有不存在的父目录
 * @param &path {path} : 要创建的目录路径
 * @return {bool} : 如果成功创建则返回 true
 */
inline bool makedirs(const fs::path& path)
{
    // 使用 std::filesystem::create_directories
    std::error_code ec;
    fs::create_directories(path, ec);
    return !ec;
}

/***
 * @description: 获取文件大小
 * @param &path {string} : 文件路径
 * @return {uintmax_t} : 文件大小 (字节)
 */
inline uintmax_t path_getsize(const std::string& path)
{
    // 使用 std::filesystem::file_size
    fs::path p(path);
    std::error_code ec;
    if (!fs::exists(p, ec) || !fs::is_regular_file(p, ec))
        return 0;
    return fs::file_size(p, ec);
}

/***
 * @description: 获取文件大小
 * @param &path {path} : 文件路径
 * @return {uintmax_t} : 文件大小 (字节)
 */
inline uintmax_t path_getsize(const fs::path& path)
{
    // 使用 std::filesystem::file_size
    std::error_code ec;
    if (!fs::exists(path, ec) || !fs::is_regular_file(path, ec))
        return 0;
    return fs::file_size(path, ec);
}

/***
 * @description: 递归遍历目录下的所有文件和目录
 * @param &path {string} : 要遍历的目录路径
 * @return {vector<string>} : 包含所有文件和目录路径的向量
 */
inline std::vector<std::string> walk(const std::string& path)
{
    // 使用 recursive_directory_iterator 遍历
    std::vector<std::string> result;
    std::error_code ec;
    if (!fs::exists(path, ec))
        return result;

    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(path, ec))
    {
        result.push_back(entry.path().string());
    }
    return result;
}

/***
 * @description: 递归遍历目录下的所有文件和目录
 * @param &path {path} : 要遍历的目录路径
 * @return {vector<path>} : 包含所有文件和目录路径的向量
 */
inline std::vector<fs::path> walk(const fs::path& path)
{
    // 使用 recursive_directory_iterator 遍历
    std::vector<fs::path> result;
    std::error_code ec;
    if (!fs::exists(path, ec))
        return result;

    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(path, ec))
    {
        result.push_back(entry.path());
    }
    return result;
}

/***
 * @description: 列出目录中的所有文件和目录 (不递归)
 * @param &path {string} : 目录路径
 * @return {vector<string>} : 包含所有文件和目录路径的向量
 */
inline std::vector<std::string> listdirs(const std::string& path)
{
    // 使用 directory_iterator 遍历
    std::vector<std::string> result;
    std::error_code ec;
    if (!fs::exists(path, ec) || !fs::is_directory(path, ec))
        return result;

    for (const fs::directory_entry& entry : fs::directory_iterator(path, ec))
    {
        result.push_back(entry.path().string());
    }
    return result;
}

/***
 * @description: 列出目录中的所有文件和目录 (不递归)
 * @param &path {path} : 目录路径
 * @return {vector<path>} : 包含所有文件和目录路径的向量
 */
inline std::vector<fs::path> listdirs(const fs::path& path)
{
    // 使用 directory_iterator 遍历
    std::vector<fs::path> result;
    std::error_code ec;
    if (!fs::exists(path, ec) || !fs::is_directory(path, ec))
        return result;

    for (const fs::directory_entry& entry : fs::directory_iterator(path, ec))
    {
        result.push_back(entry.path());
    }
    return result;
}

/***
 * @description: 分离路径的文件名和扩展名
 * @param &path {string} : 文件路径
 * @return {pair<string, string>} : 包含主干和扩展名的 pair
 */
inline std::pair<std::string, std::string> path_splitext(const std::string& path)
{
    // 创建一个 fs::path 对象
    fs::path p(path);
    // 返回包含主干和扩展名的 pair
    return std::pair<std::string, std::string>(p.stem().string(), p.extension().string());
}

/***
 * @description: 分离路径的文件名和扩展名
 * @param &path {path} : 文件路径
 * @return {pair<path, path>} : 包含主干和扩展名的 pair
 */
inline std::pair<fs::path, fs::path> path_splitext(const fs::path& path)
{
    // 返回包含主干和扩展名的 pair
    return std::pair<fs::path, fs::path>(path.stem(), path.extension());
}

/***
 * @description: 获取路径的目录部分
 * @param &path {string} : 文件路径
 * @return {string} : 目录路径
 */
inline std::string path_dirname(const std::string& path)
{
    // 返回父路径的字符串表示
    return fs::path(path).parent_path().string();
}

/***
 * @description: 获取路径的目录部分
 * @param &path {path} : 文件路径
 * @return {path} : 目录路径
 */
inline fs::path path_dirname(const fs::path& path)
{
    // 返回父路径
    return path.parent_path();
}

/***
 * @description: 获取路径的文件名部分
 * @param &path {string} : 文件路径
 * @return {string} : 文件名
 */
inline std::string path_basename(const std::string& path)
{
    // 返回文件名的字符串表示
    return fs::path(path).filename().string();
}

/***
 * @description: 获取路径的文件名部分
 * @param &path {path} : 文件路径
 * @return {path} : 文件名
 */
inline fs::path path_basename(const fs::path& path)
{
    // 返回文件名
    return path.filename();
}

/***
 * @description: 检查路径是否为文件
 * @param &path {string} : 文件路径
 * @return {bool} : 如果是文件则返回 true
 */
inline bool path_isfile(const std::string& path)
{
    // 使用 std::filesystem::is_regular_file
    std::error_code ec;
    bool is_file = fs::is_regular_file(fs::path(path), ec);
    return !ec && is_file;
}

/***
 * @description: 检查路径是否为文件
 * @param &path {path} : 文件路径
 * @return {bool} : 如果是文件则返回 true
 */
inline bool path_isfile(const fs::path& path)
{
    // 使用 std::filesystem::is_regular_file
    std::error_code ec;
    bool is_file = fs::is_regular_file(path, ec);
    return !ec && is_file;
}

/***
 * @description: 检查路径是否为目录
 * @param &path {string} : 目录路径
 * @return {bool} : 如果是目录则返回 true
 */
inline bool path_isdir(const std::string& path)
{
    // 使用 std::filesystem::is_directory
    std::error_code ec;
    bool is_dir = fs::is_directory(fs::path(path), ec);
    return !ec && is_dir;
}

/***
 * @description: 检查路径是否为目录
 * @param &path {path} : 目录路径
 * @return {bool} : 如果是目录则返回 true
 */
inline bool path_isdir(const fs::path& path)
{
    // 使用 std::filesystem::is_directory
    std::error_code ec;
    bool is_dir = fs::is_directory(path, ec);
    return !ec && is_dir;
}

/***
 * @description: 获取路径的绝对路径
 * @param &path {string} : 输入的路径
 * @return {string} : 对应的绝对路径
 */
inline std::string path_absolute(const std::string& path)
{
    // 使用 std::filesystem::absolute 获取绝对路径, 并规范化路径
    std::error_code ec;
    return fs::absolute(fs::path(path), ec).lexically_normal().string();
}

/***
 * @description: 获取路径的绝对路径
 * @param &path {path} : 输入的路径
 * @return {path} : 对应的绝对路径
 */
inline fs::path path_absolute(const fs::path& path)
{
    // 使用 std::filesystem::absolute 获取绝对路径, 并规范化路径
    std::error_code ec;
    return fs::absolute(path, ec).lexically_normal();
}

/***
 * @description: 获取一个路径相对于另一个基准路径的相对路径
 * @param &path {string} : 要转换的路径
 * @param &base {string} : 基准路径
 * @return {string} : 相对路径
 */
inline std::string path_relative(const std::string& path, const std::string& base = ".")
{
    // 使用 std::filesystem::relative 计算相对路径
    std::error_code ec;
    return fs::relative(fs::path(path), fs::path(base), ec).string();
}

/***
 * @description: 获取一个路径相对于另一个基准路径的相对路径
 * @param &path {path} : 要转换的路径
 * @param &base {path} : 基准路径
 * @return {path} : 相对路径
 */
inline fs::path path_relative(const fs::path& path, const fs::path& base = ".")
{
    // 使用 std::filesystem::relative 计算相对路径
    std::error_code ec;
    return fs::relative(path, base, ec);
}

/***
 * @description: 复制文件
 * @param &from_path {string} : 源文件路径
 * @param &to_path {string} : 目标文件路径
 * @param overwrite_existing {bool} : 是否覆盖已存在的文件
 * @return {bool} : 成功返回 true, 失败返回 false
 */
inline bool copy_file(const std::string& from_path, const std::string& to_path, bool overwrite_existing = false)
{
    // 根据 overwrite_existing 参数选择合适的复制选项
    auto options = overwrite_existing ? fs::copy_options::overwrite_existing : fs::copy_options::none;
    std::error_code ec;
    // 使用 std::filesystem::copy_file 进行复制
    fs::copy_file(from_path, to_path, options, ec);
    // 如果没有错误则返回 true
    return !ec;
}

/***
 * @description: 复制文件
 * @param &from_path {path} : 源文件路径
 * @param &to_path {path} : 目标文件路径
 * @param overwrite_existing {bool} : 是否覆盖已存在的文件
 * @return {bool} : 成功返回 true, 失败返回 false
 */
inline bool copy_file(const fs::path& from_path, const fs::path& to_path, bool overwrite_existing = false)
{
    // 根据 overwrite_existing 参数选择合适的复制选项
    auto options = overwrite_existing ? fs::copy_options::overwrite_existing : fs::copy_options::none;
    std::error_code ec;
    // 使用 std::filesystem::copy_file 进行复制
    fs::copy_file(from_path, to_path, options, ec);
    // 如果没有错误则返回 true
    return !ec;
}

/***
 * @description: 重命名或移动文件/目录
 * @param &from_path {string} : 源路径
 * @param &to_path {string} : 目标路径
 * @return {bool} : 成功返回 true, 失败返回 false
 */
inline bool rename_path(const std::string& from_path, const std::string& to_path)
{
    std::error_code ec;
    // 使用 std::filesystem::rename
    fs::rename(from_path, to_path, ec);
    return !ec;
}

/***
 * @description: 重命名或移动文件/目录
 * @param &from_path {path} : 源路径
 * @param &to_path {path} : 目标路径
 * @return {bool} : 成功返回 true, 失败返回 false
 */
inline bool rename_path(const fs::path& from_path, const fs::path& to_path)
{
    std::error_code ec;
    // 使用 std::filesystem::rename
    fs::rename(from_path, to_path, ec);
    return !ec;
}

/***
 * @description: 删除文件或目录 (可递归)
 * @param &path {string} : 要删除的路径
 * @return {bool} : 成功返回 true, 失败返回 false
 */
inline bool remove_path(const std::string& path)
{
    std::error_code ec;
    // 使用 std::filesystem::remove_all, 可以删除文件或递归删除目录
    fs::remove_all(path, ec);
    return !ec;
}

/***
 * @description: 删除文件或目录 (可递归)
 * @param &path {path} : 要删除的路径
 * @return {bool} : 成功返回 true, 失败返回 false
 */
inline bool remove_path(const fs::path& path)
{
    std::error_code ec;
    // 使用 std::filesystem::remove_all, 可以删除文件或递归删除目录
    fs::remove_all(path, ec);
    return !ec;
}

}  // namespace myfs

#endif  // !__MYFILESYSTEM__H__
