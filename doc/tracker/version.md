# version 版本信息

<!-- vscode-markdown-toc -->
- [version 版本信息](#version-版本信息)
  - [概述](#概述)
  - [宏定义](#宏定义)
  - [查询函数](#查询函数)
  - [使用方式](#使用方式)

<!-- vscode-markdown-toc -->

## 概述

`version.hpp` 定义了跟踪器的版本信息宏和查询函数。版本号在代码中手动维护, 编译时间和 git 信息由 CMake 在编译时自动注入。所有宏均以 `TRACK_` 为前缀。

**文件**: `lib/tracker/version.hpp`
**命名空间**: `tracker`

## 宏定义

| 宏                     | 默认值      | 说明                       |
| ---------------------- | ----------- | -------------------------- |
| `TRACK_VERSION_STRING` | `"0.1.0"`   | 固定版本号字符串(手动维护) |
| `TRACK_BUILD_TIME`     | `"unknown"` | 编译时间(由 cmake 注入)    |
| `TRACK_GIT_HASH`       | `"unknown"` | 当前 git commit 完整哈希   |
| `TRACK_GIT_BRANCH`     | `"unknown"` | 当前 git 分支名            |
| `TRACK_PROJECT_NAME`   | `"track"`   | 项目名称(由 cmake 定义)    |

## 查询函数

所有函数位于 `tracker` 命名空间:

```cpp
namespace tracker {

// 获取版本字符串
inline const char* version_string();

// 获取编译时间字符串
inline const char* build_time();

// 获取 git 哈希字符串
inline const char* git_hash();

// 获取 git 分支字符串
inline const char* git_branch();

// 获取项目名称字符串
inline const char* project_name();

}  // namespace tracker
```

## 使用方式

```cpp
#include "tracker/version.hpp"

std::cout << "Version: " << tracker::version_string() << std::endl;
std::cout << "Build Time: " << tracker::build_time() << std::endl;
std::cout << "Git Hash: " << tracker::git_hash() << std::endl;
std::cout << "Git Branch: " << tracker::git_branch() << std::endl;
std::cout << "Project: " << tracker::project_name() << std::endl;
