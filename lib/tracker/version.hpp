/***
 * @Author       : gxs
 * @Date         : 2026-06-21 15:58:20
 * @LastEditors  : gxs
 * @LastEditTime : 2026-06-21 15:58:20
 * @FilePath     : /visionAlgorithm/lib/tracker/version.hpp
 * @Description  : tracker 版本信息定义头文件; 参照 detector/version.hpp 实现;
 * 跟踪代码参考 https://github.com/shaoshengsong/DeepSORT.git 实现;
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __TRACKER_VERSION__H__
#define __TRACKER_VERSION__H__

// 版本号宏定义, 全部以 TRACK_ 为前缀;
// 以下宏若无 cmake 注入, 则使用默认值;

// 固定版本号字符串(直接在代码中手动维护)
#define TRACK_VERSION_STRING "0.1.0"

// 编译时间(由 cmake 注入, 格式: YYYYMMDD_HHMMSS)
#ifndef TRACK_BUILD_TIME
#define TRACK_BUILD_TIME "unknown"
#endif

// 当前 git commit 完整哈希(由 cmake 注入)
#ifndef TRACK_GIT_HASH
#define TRACK_GIT_HASH "unknown"
#endif

// 当前 git 分支名(由 cmake 注入)
#ifndef TRACK_GIT_BRANCH
#define TRACK_GIT_BRANCH "unknown"
#endif

// 项目名称(由 cmake project() 定义并注入)
#ifndef TRACK_PROJECT_NAME
#define TRACK_PROJECT_NAME "track"
#endif

namespace tracker
{

// 版本信息查询函数
/***
 * @description: 获取版本字符串
 * @return
 */
inline const char* version_string()
{
    return TRACK_VERSION_STRING;
}

/***
 * @description: 获取编译时间字符串
 * @return
 */
inline const char* build_time()
{
    return TRACK_BUILD_TIME;
}

/***
 * @description: 获取 git 哈希字符串
 * @return
 */
inline const char* git_hash()
{
    return TRACK_GIT_HASH;
}

/***
 * @description: 获取 git 分支字符串
 * @return
 */
inline const char* git_branch()
{
    return TRACK_GIT_BRANCH;
}

/***
 * @description: 获取项目名称字符串
 * @return
 */
inline const char* project_name()
{
    return TRACK_PROJECT_NAME;
}

}  // namespace tracker

#endif  // !__TRACKER_VERSION__H__