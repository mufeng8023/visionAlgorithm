/***
 * @description: C 语言 API 包装层, 供 Python ctypes 调用;
 *               包装 tracker::TrackerRuntime 的所有核心接口;
 *
 *               设计目标:
 *               - 纯 C 接口 (无 C++ 名称修饰, 无 STL 类型);
 *               - 所有内存由 C++ 侧管理, Python 通过指针句柄操作;
 *               - 检测框和跟踪结果通过扁平 float/int 数组传递;
 *
 *               检测框输入格式 (每帧):
 *                 每个检测框: [x1, y1, x2, y2, score, cls_id];
 *                 总数据长度: n_dets * 6;
 *
 *               跟踪结果输出格式:
 *                 每个结果: [track_id, det_index, cls_id, score, l, t, w, h];
 *                 总数据长度: n_results * 8;
 *
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __TRACKER_C_API__H__
#define __TRACKER_C_API__H__

#ifdef __cplusplus
extern "C"
{
#endif

    // ================================================================
    // 句柄类型 (不透明指针)
    // ================================================================

    // TrackerRuntime 实例句柄;
    // Python 侧仅保存此指针, 不直接访问内部结构;
    typedef void* TrackerHandle;

    // ================================================================
    // 生命周期管理
    // ================================================================

    /***
     * @description: 创建 TrackerRuntime 实例;
     * @return TrackerHandle : 追踪器句柄, 失败返回 NULL;
     */
    TrackerHandle tracker_create();

    /***
     * @description: 销毁 TrackerRuntime 实例;
     * @param handle TrackerHandle : 由 tracker_create() 返回的句柄;
     */
    void tracker_destroy(TrackerHandle handle);

    // ================================================================
    // 初始化
    // ================================================================

    /***
     * @description: 初始化追踪器, 从 ini 配置文件加载参数;
     * @param handle      TrackerHandle : 追踪器句柄;
     * @param ini_path    const char*   : ini 配置文件路径;
     * @param max_history int           : 最大历史帧数, 默认 150;
     * @return int : 0 成功, -1 失败;
     */
    int tracker_init(TrackerHandle handle, const char* ini_path, int max_history);

    // ================================================================
    // 核心跟踪 API
    // ================================================================

    /***
     * @description: 单帧更新, 将检测结果送入追踪器;
     *
     *               检测框格式 (输入):
     *                 dets_data 数组长度为 n_dets * 6;
     *                 每个检测: [x1, y1, x2, y2, score, cls_id];
     *
     *               跟踪结果格式 (输出):
     *                 results_data 数组长度至少为 max_results * 8;
     *                 每个结果: [track_id, det_index, cls_id, score, l, t, w, h];
     *                 实际结果数量通过 n_results 返回;
     *
     * @param handle       TrackerHandle : 追踪器句柄;
     * @param dets_data    const float*  : 检测框数据 (flatten array);
     * @param n_dets       int           : 检测框数量;
     * @param results_data float*        : 输出缓冲区 (由调用方分配);
     * @param max_results  int           : 输出缓冲区最大容量;
     * @param n_results    int*          : 输出参数, 实际结果数量;
     * @return int : 0 成功, -1 失败;
     */
    int tracker_update(TrackerHandle handle,    //
                       const float* dets_data,  //
                       int n_dets,              //
                       float* results_data,     //
                       int max_results,         //
                       int* n_results);

    /***
     * @description: 重置追踪器 (清空所有轨迹和历史);
     * @param handle TrackerHandle : 追踪器句柄;
     * @return int : 0 成功, -1 失败;
     */
    int tracker_reset(TrackerHandle handle);

    // ================================================================
    // 状态查询
    // ================================================================

    /***
     * @description: 获取当前帧活跃轨迹数;
     * @param handle TrackerHandle : 追踪器句柄;
     * @return int : 活跃轨迹数, -1 失败;
     */
    int tracker_active_count(TrackerHandle handle);

    /***
     * @description: 获取总轨迹数 (历史累计);
     * @param handle TrackerHandle : 追踪器句柄;
     * @return int : 总轨迹数, -1 失败;
     */
    int tracker_total_count(TrackerHandle handle);

    /***
     * @description: 判断追踪器是否已初始化;
     * @param handle TrackerHandle : 追踪器句柄;
     * @return int : 1 已初始化, 0 未初始化, -1 无效句柄;
     */
    int tracker_is_initialized(TrackerHandle handle);

    /***
     * @description: 获取版本信息字符串;
     * @return const char* : 版本字符串;
     */
    const char* tracker_version();

#ifdef __cplusplus
}
#endif

#endif  // !__TRACKER_C_API__H__