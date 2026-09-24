/***
 * @Author       : gxs
 * @Date         : 2026-06-04 16:56:57
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-04 16:57:00
 * @FilePath     : /visionAlgorithm/lib/detector/version.hpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __VERSION__H__
#define __VERSION__H__

// 版本号宏定义, 全部以 DETECT_ 为前缀;
// 以下宏若无 cmake 注入, 则使用默认值;

// 固定版本号字符串(直接在代码中手动维护)
#define DETECT_VERSION_STRING "1.0.0"

// 编译时间(由 cmake 注入, 格式: YYYYMMDD_HHMMSS)
#ifndef DETECT_BUILD_TIME
#define DETECT_BUILD_TIME "unknown"
#endif

// 当前 git commit 完整哈希(由 cmake 注入)
#ifndef DETECT_GIT_HASH
#define DETECT_GIT_HASH "unknown"
#endif

// 当前 git 分支名(由 cmake 注入)
#ifndef DETECT_GIT_BRANCH
#define DETECT_GIT_BRANCH "unknown"
#endif

// 项目名称(由 cmake project() 定义并注入)
#ifndef DETECT_PROJECT_NAME
#define DETECT_PROJECT_NAME "yolo"
#endif

namespace yolo
{

// 版本信息查询函数
/***
 * @description: 获取版本字符串
 * @return
 */
inline const char* version_string()
{
    return DETECT_VERSION_STRING;
}

/***
 * @description: 获取编译时间字符串
 * @return
 */
inline const char* build_time()
{
    return DETECT_BUILD_TIME;
}

/***
 * @description: 获取 git 哈希字符串
 * @return
 */
inline const char* git_hash()
{
    return DETECT_GIT_HASH;
}

/***
 * @description: 获取 git 分支字符串
 * @return
 */
inline const char* git_branch()
{
    return DETECT_GIT_BRANCH;
}

/***
 * @description: 获取项目名称字符串
 * @return
 */
inline const char* project_name()
{
    return DETECT_PROJECT_NAME;
}

}  // namespace yolo

#endif  // !__VERSION__H__