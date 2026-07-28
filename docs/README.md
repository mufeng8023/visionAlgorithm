<!-- ============================================================================
   文档站点首页 (README.md)
   ============================================================================
   这是你的文档站点的首页内容。
   docsify 会将此文件渲染为网站首页（homepage）。

   使用说明：
     1. 替换下面的标题和内容为你自己的文档介绍
     2. 可根据需要增删章节
     3. 保持开头的标题（# 你的文档名称），docsify 会用它作为页面标题

   提示：
     封面页 (_coverpage.md) 是进入网站前的欢迎屏，与首页不同。
     如果你配置了封面页，用户需要点击封面上的按钮才能进入此首页。
   ============================================================================ -->

# visionAlgorithm

> visionAlgorithm 是一个基于 C++17 的轻量级 YOLO 系列模型推理框架, 支持通过 OpenCV DNN 后端加载 ONNX 模型, 完成目标检测与姿态估计任务。项目采用模块化设计, 将模型加载、推理、后处理、结果可视化等环节解耦, 便于扩展和维护。

## 链接

- **GitHub 仓库**: [https://github.com/mufeng8023/visionAlgorithm.git](https://github.com/mufeng8023/visionAlgorithm.git)
- **GitHub Pages 文档**: [https://mufeng8023.github.io/visionAlgorithm](https://mufeng8023.github.io/visionAlgorithm)

## 快速开始

### 环境依赖

- GCC >= 11 (C++17)
- CMake >= 3.10
- OpenCV (需包含 DNN 模块, 如需 GPU 推理需编译 CUDA 支持)

### 构建步骤

```bash
# 克隆仓库
git clone https://github.com/mufeng8023/visionAlgorithm.git
cd visionAlgorithm

# Debug 构建 (启用 TIMER_DEBUG 宏, 默认日志级别为 Debug(1))
./build_debug.sh

# Release 构建 (禁用 TIMER_DEBUG 宏, 默认日志级别为 Info(2))
./build_release.sh
```

> 更多详细步骤请参考 [detector 模块文档](detector/index.md)。

## 文档索引

本项目的文档按模块组织为三个主要章节, 涵盖推理核心组件、目标跟踪算法与通用基础设施:

### 🧩 核心模块 — detector

detector 模块是推理框架的核心, 包含以下组件:

| 类别         | 说明                                   | 链接                                                                           |
| ------------ | -------------------------------------- | ------------------------------------------------------------------------------ |
| **核心调度** | 运行时调度器, 串联推理全流程           | [RunTime](detector/RunTime.md)                                                 |
| **网络推理** | 模型加载与推理抽象及 OpenCV DNN 实现   | [BaseNet](detector/BaseNet.md) / [OpencvNet](detector/OpencvNet.md)            |
| **后处理**   | YOLOv5/v8/v26 检测与姿态估计后处理     | [Det 系列](detector/index.md#后处理) / [Pose 系列](detector/index.md#后处理-1) |
| **数据结构** | 模型配置、网络输出、检测结果等数据结构 | [NetConfig](detector/NetConfig.md) / [YoloObject](detector/YoloObject.md)      |
| **工具函数** | 结果可视化与配置/图像预处理工具        | [draw_result](detector/draw_result.md) / [utils](detector/utils.md)            |

📄 完整索引请查看 [detector 模块首页](detector/index.md)。

### 🎯 跟踪模块 — tracker

tracker 模块提供多目标跟踪算法实现, 包含以下组件:

| 类别           | 说明                                      | 链接                                                                                                                  |
| -------------- | ----------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| **核心调度**   | 跟踪运行时调度器, 封装 ByteTrack/DeepSORT | [TrackerRuntime](tracker/TrackerRuntime.md)                                                                           |
| **卡尔曼滤波** | 泛型卡尔曼滤波基类, Box 与关键点平滑实现  | [BaseKalmanFilter](tracker/BaseKalmanFilter.md) / [BoxKalmanFilter](tracker/BoxKalmanFilter.md)                       |
| **数据结构**   | 轨迹基类、检测目标、跟踪结果、配置等结构  | [BaseTrack](tracker/BaseTrack.md) / [TrackResult](tracker/TrackResult.md) / [TrackerConfig](tracker/TrackerConfig.md) |
| **ByteTrack**  | 高低分检测框三轮 IoU 关联跟踪             | [ByteTracker](tracker/bytetrack/ByteTracker.md) / [matching](tracker/bytetrack/matching.md)                           |
| **DeepSORT**   | 级联匹配 + IoU 二次匹配, 支持 ReID 特征   | [DeepSortTracker](tracker/deepsort/DeepSortTracker.md) / [NNMetric](tracker/deepsort/NNMetric.md)                     |
| **工具函数**   | INI 配置解析、跟踪器工厂创建与版本信息    | [utils](tracker/utils.md) / [version](tracker/version.md)                                                             |

📄 完整索引请查看 [tracker 模块首页](tracker/index.md)。

### 🛠 通用组件 — common

common 模块提供框架通用的基础设施组件:

| 文档         | 说明                              | 链接                                   |
| ------------ | --------------------------------- | -------------------------------------- |
| 日志系统     | 基于 C++17 的多线程安全日志管理器 | [logging](common/logging.md)           |
| 文件系统工具 | 路径操作、目录遍历等文件系统封装  | [myFilesystem](common/myFilesystem.md) |
| 计时器工具   | 高精度计时与性能分析辅助          | [timer](common/timer.md)               |

## 特点

- 🚀 **高性能推理** — 基于 OpenCV DNN 后端, 支持 CPU/GPU 推理
- 🔌 **模块化设计** — 模型加载、推理、后处理、可视化各环节完全解耦
- 🎯 **多模型支持** — 兼容 YOLOv5/v8/v26 检测与姿态估计模型
- 🧪 **调试友好** — 内置 TIMER_DEBUG 宏, 支持运行时性能分析
- 📝 **完善日志** — 基于 C++17 的多线程安全日志系统, 支持多级日志
- 🔄 **多目标跟踪** — 内置 ByteTrack 与 DeepSORT 算法, 支持卡尔曼滤波轨迹平滑

## 文档目录概览

| 章节                                   | 说明                                                      |
| -------------------------------------- | --------------------------------------------------------- |
| [detector 模块文档](detector/index.md) | 核心推理模块: 调度、网络、后处理、数据结构、工具          |
| [tracker 模块文档](tracker/index.md)   | 多目标跟踪模块: ByteTrack、DeepSORT、卡尔曼滤波、数据结构 |
| [common 模块文档](common/index.md)     | 通用组件: 日志、文件系统、计时器                          |

## 贡献

欢迎提交 Issue 和 Pull Request 来改进文档!

## 许可证

本项目基于 [MIT License](https://opensource.org/licenses/MIT) 开源。
