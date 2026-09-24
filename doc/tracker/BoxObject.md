# BoxObject 检测目标数据结构

<!-- vscode-markdown-toc -->
- [BoxObject 检测目标数据结构](#boxobject-检测目标数据结构)
  - [概述](#概述)
  - [类型别名](#类型别名)
  - [类定义](#类定义)
  - [核心接口](#核心接口)
  - [数据流说明](#数据流说明)

<!-- vscode-markdown-toc -->

## 概述

`BoxObject` 是跟踪器输入检测目标的统一数据结构。跟踪器接收 `std::vector<BoxObject>` 作为输入, 每个 BoxObject 包含一个检测框及其关联信息。

BoxObject 继承自 `BaseTrack`, 因此也具备轨迹的完整功能。但在作为输入时, 仅使用其存储检测结果的部分。

**文件**: `lib/tracker/BoxObject.hpp`
**命名空间**: `tracker`

## 类型别名

```cpp
using BOX_MEAN = Eigen::Matrix<float32, 1, 8, Eigen::RowMajor>;   // [cx, cy, a, h, v_cx, v_cy, v_a, v_h]
using BOX_COVA = Eigen::Matrix<float32, 8, 8, Eigen::RowMajor>;   // 8x8 协方差矩阵
```

## 类定义

```cpp
class BoxObject : public BaseTrack {
public:
    float32 conf = 1.0f;          // 检测置信度, 用于 ByteTrack 的高低分框分流
    float32 cls_conf = 0.0f;      // 类别置信度
    std::vector<float32> kpts;    // 关键点数据 (可选)

    // 默认构造函数
    BoxObject() {}

    // 从检测结果构造 BoxObject
    BoxObject(int32 track_id, int32 frame_id, float32 expand_box_rate = 0.0f);

    // 从原始检测数据直接初始化 (使用 ltwh)
    BoxObject(float32 x1, float32 y1, float32 x2, float32 y2,
              float32 score, int32 cls_id,
              int32 track_id = -1, int32 frame_id = 0);

    // 标记丢失/移除 (纯虚函数实现)
    void mark_lost() override;
    void mark_removed() override;

    // 从原始检测框初始化 (创建后使用此方法)
    void init_box(float32 x1, float32 y1, float32 x2, float32 y2,
                  float32 score, int32 cls_id,
                  const std::vector<float32>& kpts = {});
};
```

## 核心接口

| 方法                                      | 说明                                           |
| ----------------------------------------- | ---------------------------------------------- |
| `init_box(x1,y1,x2,y2,score,cls_id,kpts)` | 从检测结果初始化边界框, 执行扩展计算           |
| `update_ltwh()`                           | 覆盖基类方法, 跟踪器内部根据轨道状态更新扩展框 |
| `mark_lost()`                             | 空实现 (输入检测目标无需状态管理)              |
| `mark_removed()`                          | 空实现 (输入检测目标无需状态管理)              |

## 数据流说明

```
检测器输出 (检测框) -> BoxObject 构造函数
  -> init_box 保存原始 ltwh
  -> 计算 ltwh_expand: ltwh 各方向按 _expand_box_rate 扩展
  -> 输入跟踪器 update()

跟踪器输出 (轨迹) -> TrackResult
  -> 读取原始 ltwh (原始检测框, 未扩展)
```

关键点: 外部用户读取跟踪结果时, 读取的是原始 `ltwh`, 而不是 `ltwh_expand`。`ltwh_expand` 仅在跟踪器内部用于卡尔曼滤波器操作。
