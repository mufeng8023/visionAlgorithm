/***
 * @description: C 语言 API 包装层 - YOLO 检测器;
 *               包装 yolo::RunTime 供 Python ctypes 调用;
 *               检测结果通过扁平 float 数组传递;
 *
 *               检测结果格式:
 *                 每个目标: [x1, y1, x2, y2, score, cls_id];
 *                 输出长度: n_dets * 6;
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#ifndef __DETECTOR_C_API__H__
#define __DETECTOR_C_API__H__

#ifdef __cplusplus
extern "C"
{
#endif

    // 不透明句柄
    typedef void* DetectorHandle;

    /***
     * @description: 创建检测器实例;
     * @param ini_path     const char* : 检测模型 ini 配置文件路径;
     * @param onnx_path    const char* : ONNX 模型文件路径;
     * @param device       int         : GPU 设备号 (-1 为 CPU);
     * @return DetectorHandle : 成功返回句柄, 失败返回 NULL;
     */
    DetectorHandle detector_create(const char* ini_path, const char* onnx_path, int device);

    /***
     * @description: 销毁检测器实例;
     */
    void detector_destroy(DetectorHandle handle);

    /***
     * @description: 单图推理;
     * @param handle       DetectorHandle : 检测器句柄;
     * @param image_path   const char*    : 图片路径;
     * @param out_data     float*         : 输出缓冲区 [x1,y1,x2,y2,score,cls_id] * N;
     * @param max_dets     int            : 输出缓冲区最大容量;
     * @param n_dets       int*           : 输出, 实际检测数;
     * @return int : 0 成功, -1 失败;
     */
    int detector_detect(DetectorHandle handle,   //
                        const char* image_path,  //
                        float* out_data,         //
                        int max_dets,            //
                        int* n_dets);

#ifdef __cplusplus
}
#endif

#endif  // !__DETECTOR_C_API__H__