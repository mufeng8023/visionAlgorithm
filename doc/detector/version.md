# version 版本信息

<!-- vscode-markdown-toc -->
- [version 版本信息](#version-版本信息)
  - [概述](#概述)
  - [宏定义](#宏定义)
  - [查询函数](#查询函数)
  - [使用方式](#使用方式)

<!-- vscode-markdown-toc -->

## 概述

`version.hpp` 定义了项目的版本信息宏和查询函数。版本号在代码中手动维护, 编译时间和 git 信息由 CMake 在编译时自动注入。所有宏均以 `DETECT_` 为前缀。

## 宏定义

| 宏                      | 默认值      | 说明                       |
| ----------------------- | ----------- | -------------------------- |
| `DETECT_VERSION_STRING` | `"1.0.0"`   | 固定版本号字符串(手动维护) |
| `DETECT_BUILD_TIME`     | `"unknown"` | 编译时间(由 cmake 注入)    |
| `DETECT_GIT_HASH`       | `"unknown"` | 当前 git commit 完整哈希   |
| `DETECT_GIT_BRANCH`     | `"unknown"` | 当前 git 分支名            |
| `DETECT_PROJECT_NAME`   | `"yolo"`    | 项目名称(由 cmake 定义)    |

## 查询函数

所有函数位于 `yolo` 命名空间:

```cpp
namespace yolo {

// 获取版本字符串
const char* version_string();

// 获取编译时间字符串
const char* build_time();

// 获取 git 哈希字符串
const char* git_hash();

// 获取 git 分支字符串
const char* git_branch();

// 获取项目名称字符串
const char* project_name();

}  // namespace yolo
```

## 使用方式

```cpp
#include "detector/version.hpp"

std::cout << "Version: " << yolo::version_string() << std::endl;
std::cout << "Build Time: " << yolo::build_time() << std::endl;
std::cout << "Git Hash: " << yolo::git_hash() << std::endl;
std::cout << "Git Branch: " << yolo::git_branch() << std::endl;
std::cout << "Project: " << yolo::project_name() << std::endl;
