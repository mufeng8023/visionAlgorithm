# myFilesystem.hpp - 文件系统工具库

## 概述

`myFilesystem.hpp` 是对 C++17 `<filesystem>` 标准库的一层封装, 提供更简洁、易用的文件系统操作接口; 所有函数都位于 `myfs` 命名空间中, 并同时提供 `std::string` 和 `fs::path` 两种重载版本;

## 核心函数

### 路径操作

| 函数                            | 说明                                              |
| ------------------------------- | ------------------------------------------------- |
| `path_join(path1, path2)`       | 连接两个路径                                      |
| `path_splitext(path)`           | 分离文件名和扩展名, 返回 `{stem, extension}` pair |
| `path_dirname(path)`            | 获取路径的目录部分 (父目录)                       |
| `path_basename(path)`           | 获取路径的文件名部分                              |
| `path_absolute(path)`           | 获取路径的绝对路径 (自动规范化)                   |
| `path_relative(path, base=".")` | 获取路径相对于基准路径的相对路径                  |

### 路径检查

| 函数                 | 说明                   |
| -------------------- | ---------------------- |
| `path_exists(path)`  | 检查路径是否存在       |
| `path_isfile(path)`  | 检查路径是否为常规文件 |
| `path_isdir(path)`   | 检查路径是否为目录     |
| `path_getsize(path)` | 获取文件大小 (字节)    |

### 目录操作

| 函数             | 说明                                |
| ---------------- | ----------------------------------- |
| `makedirs(path)` | 创建目录, 包括所有不存在的父目录    |
| `walk(path)`     | 递归遍历目录下的所有文件和目录      |
| `listdirs(path)` | 列出目录中的所有文件和目录 (不递归) |

### 文件操作

| 函数                                                      | 说明                               |
| --------------------------------------------------------- | ---------------------------------- |
| `copy_file(from_path, to_path, overwrite_existing=false)` | 复制文件, 可选项是否覆盖已存在文件 |
| `rename_path(from_path, to_path)`                         | 重命名或移动文件/目录              |
| `remove_path(path)`                                       | 递归删除文件或目录                 |

## 设计特点

- **双重载设计**: 每个函数都提供 `const std::string&` 和 `const fs::path&` 两种参数形式, 方便不同场景使用
- **错误码处理**: 所有文件系统操作均使用 `std::error_code` 捕获错误, 而非抛出异常, 确保异常安全
- **简洁返回值**: 返回 `bool` 表示操作是否成功, 或直接返回结果数据

## 使用示例

```cpp
#include "myFilesystem.hpp"

using namespace myfs;

// 路径连接
std::string full_path = path_join("/data", "images");  // "/data/images"

// 路径检查
if (path_exists("/data/images/photo.jpg"))
{
    uintmax_t size = path_getsize("/data/images/photo.jpg");
    bool is_file = path_isfile("/data/images/photo.jpg");
    bool is_dir = path_isdir("/data/images");
}

// 分离文件名和扩展名
auto [stem, ext] = path_splitext("photo.jpg");
// stem = "photo", ext = ".jpg"

// 路径拆分
std::string dir = path_dirname("/data/images/photo.jpg");   // "/data/images"
std::string base = path_basename("/data/images/photo.jpg");  // "photo.jpg"

// 绝对路径和相对路径
std::string abs = path_absolute("./images/photo.jpg");
std::string rel = path_relative("/data/images/photo.jpg", "/data");

// 目录操作
makedirs("/data/output/results");      // 创建多级目录
std::vector<std::string> all = walk("/data");          // 递归遍历
std::vector<std::string> top = listdirs("/data");      // 仅当前层

// 文件操作
copy_file("source.txt", "dest.txt", true);  // 覆盖复制
rename_path("old_name.txt", "new_name.txt");
remove_path("/tmp/temp_folder");            // 递归删除
```
