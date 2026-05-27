# ObjectBuffer 检测结果缓冲区

<!-- vscode-markdown-toc -->
- [ObjectBuffer 检测结果缓冲区](#objectbuffer-检测结果缓冲区)
  - [概述](#)
  - [类定义](#)
  - [核心接口](#)
  - [数据布局](#)

<!-- vscode-markdown-toc -->

## 概述

`ObjectBuffer` 是检测结果的高效存储容器, 采用连续内存布局存储所有检测目标的信息。每个目标包含基础边界框信息 (x_center, y_center, width, height, score, cls_id) 和可选的额外信息 (关键点、分割掩码等)。支持有效性标记和紧凑化操作。

## 类定义

**文件**: `lib/detector/ObjectBuffer.hpp`  
**命名空间**: `yolo`

```cpp
class ObjectBuffer {
private:
    uint32 max_obj_count;           // 最大检测目标数
    uint32 stride;                  // 每个目标的步长 (base_box_len + extra_dim)
    uint32 extra_dim;               // 额外信息维度
    std::vector<float32> buffer;    // 数据缓冲区
    std::vector<ObjStatus> valid_mask; // 有效性标记
};
```

## 核心接口

| 方法                                     | 说明                            |
| ---------------------------------------- | ------------------------------- |
| `ObjectBuffer(max_obj_count, extra_dim)` | 构造函数, 预分配缓冲区          |
| `get_max_count()`                        | 获取最大检测目标数              |
| `get_stride()`                           | 获取每个目标的步长              |
| `get_obj_count()`                        | 获取当前目标总数 (有效 + 无效)  |
| `get_valid_count()`                      | 获取有效目标数                  |
| `is_valid(obj_idx)`                      | 判断目标是否有效                |
| `set_valid(obj_idx, bool)`               | 设置目标有效性                  |
| `at(obj_idx)`                            | 获取目标数据起始地址            |
| `operator[](obj_idx)`                    | 同 at, 支持 `buffer[i][j]` 语法 |
| `expand_obj()`                           | 扩容, 添加一个无效目标槽位      |
| `push_back(data)`                        | 添加一个有效目标到末尾          |
| `append_at(det_idx, data)`               | 在指定位置添加目标              |
| `get_sorted_indices()`                   | 获取按分数降序排列的索引列表    |
| `compact()`                              | 压缩缓冲区, 移除无效目标        |
| `clear()`                                | 清空缓冲区                      |

## 数据布局

每个目标在缓冲区中的存储格式:

| 偏移 | 字段     | 说明                |
| ---- | -------- | ------------------- |
| 0    | x_center | 中心点 x 坐标       |
| 1    | y_center | 中心点 y 坐标       |
| 2    | width    | 边界框宽度          |
| 3    | height   | 边界框高度          |
| 4    | score    | 置信度分数          |
| 5    | cls_id   | 类别 ID             |
| 6+   | extra    | 额外信息 (关键点等) |

`ObjectOffset` 结构体定义了各字段的偏移常量。
