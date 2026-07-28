/***
 * @Author       : gxs
 * @Date         : 2026-05-19 21:23:17
 * @LastEditors  : gxs
 * @LastEditTime : 2026-05-19 21:23:20
 * @FilePath     : /visionAlgorithm/lib/detector/YoloObject.h
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */

#ifndef __YOLOOBJECT__H__
#define __YOLOOBJECT__H__

#include <vector>

#include "types.hpp"

namespace yolo
{

typedef struct
{
    uint32 cls_id;
    float32 score;
    float32 x1;
    float32 y1;
    float32 x2;
    float32 y2;
} Box;

typedef struct
{
    float32 x;
    float32 y;
    float32 score;
} KeyPoint;

typedef struct
{
    // === 共有属性 ===
    TaskType type = TaskType::detection;
    Box box;

    // === 动态数据: 未 resize 前, 不占用任何堆内存 ===
    std::vector<KeyPoint> kpts;  // Pose 专属
    std::vector<uint8> mask;     // Seg  专属

    // === 标量数据: 直接平铺, 消除内存覆盖隐患 ===
    float32 angle = 0.0f;    // OBB 专属
    uint32 mask_width = 0;   // Seg 专属
    uint32 mask_height = 0;  // Seg 专属
} YoloObject;

}  // namespace yolo

#endif  // !__YOLOOBJECT__H__