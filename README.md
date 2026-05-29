# visionAlgorithm

<!-- vscode-markdown-toc -->
- [visionAlgorithm](#visionalgorithm)
  - [项目简介](#项目简介)
  - [支持的模型](#支持的模型)
    - [推理后端](#推理后端)
    - [检测任务 (detection)](#检测任务-detection)
    - [姿态估计任务 (pose)](#姿态估计任务-pose)
    - [其他任务类型](#其他任务类型)
  - [支持的功能](#支持的功能)
  - [项目结构](#项目结构)
  - [构建与运行](#构建与运行)
    - [环境依赖](#环境依赖)
    - [构建步骤](#构建步骤)
    - [运行示例](#运行示例)
  - [详细文档](#详细文档)
  - [参考项目](#参考项目)
  - [许可证](#许可证)

<!-- vscode-markdown-toc -->

## 项目简介

visionAlgorithm 是一个基于 C++17 的轻量级 YOLO 系列模型推理框架, 支持通过 OpenCV DNN 后端加载 ONNX 模型, 完成目标检测与姿态估计任务。项目采用模块化设计, 将模型加载、推理、后处理、结果可视化等环节解耦, 便于扩展和维护。

## 支持的模型

### 推理后端

| 后端   | 支持状态 | 说明                      |
| ------ | -------- | ------------------------- |
| OpenCV | 已支持   | 基于 OpenCV DNN 加载 ONNX |
| 其他   | 未支持   | 代码预留 TODO, 待扩展     |

### 检测任务 (detection)

| 模型    | 后处理                         | 支持状态 |
| ------- | ------------------------------ | -------- |
| yolov5  | DetPostProcessV5 (anchor-base) | 已支持   |
| yolov3u | DetPostProcessV8 (anchor-free) | 已支持   |
| yolov5u | DetPostProcessV8 (anchor-free) | 已支持   |
| yolov6u | DetPostProcessV8 (anchor-free) | 已支持   |
| yolov7u | DetPostProcessV8 (anchor-free) | 已支持   |
| yolov8  | DetPostProcessV8 (anchor-free) | 已支持   |
| yolov9  | DetPostProcessV8 (anchor-free) | 已支持   |
| yolov10 | DetPostProcessV8 (anchor-free) | 已支持   |
| yolo11  | DetPostProcessV8 (anchor-free) | 已支持   |
| yolo12  | DetPostProcessV8 (anchor-free) | 已支持   |
| yolov4  | -                              | 不支持   |
| yolo26  | -                              | 不支持   |

### 姿态估计任务 (pose)

| 模型     | 后处理                          | 支持状态 |
| -------- | ------------------------------- | -------- |
| yolov5   | PosePostProcessV5 (anchor-base) | 已支持   |
| yolov5u  | PosePostProcessV8 (anchor-free) | 已支持   |
| yolov8   | PosePostProcessV8 (anchor-free) | 已支持   |
| yolo11   | PosePostProcessV8 (anchor-free) | 已支持   |
| 其余模型 | -                               | 不支持   |

### 其他任务类型

| 任务           | 支持状态 |
| -------------- | -------- |
| classification | 不支持   |
| segmentation   | 不支持   |
| obb            | 不支持   |

> 完整支持矩阵请参见 [RunTime 运行时调度器](doc/RunTime.md)。

## 支持的功能

- **模型推理**: 基于 OpenCV DNN 加载 ONNX 模型, 支持 CPU / CUDA GPU 推理
- **后处理**: 内置 NMS (非极大值抑制), 支持类别感知 / 非感知模式
- **结果可视化**: 自动绘制检测框、类别标签、置信度分数以及姿态关键点骨架
- **日志系统**: 基于 INI 配置文件的多日志器管理, 支持分级日志输出
- **配置驱动**: 通过 INI 配置文件管理模型参数, 无需重新编译

## 项目结构

```bash
visionAlgorithm/
├── CMakeLists.txt          # CMake 构建配置
├── build.sh                # 构建脚本
├── test_run.sh             # 测试运行脚本
├── log_config.ini          # 日志配置文件
├── config/                 # 模型 INI 配置文件
├── onnx/                   # ONNX 模型文件
├── test_img/               # 测试图片
├── include/                # 公共头文件
│   ├── args.hpp            # 命令行参数解析 (Taywee/args)
│   ├── common.hpp          # 公共工具函数
│   ├── ini_parser.hpp      # INI 配置文件解析器
│   ├── logging.hpp         # 日志系统
│   ├── myFilesystem.hpp    # 文件系统工具
│   ├── types.hpp           # 基础类型定义
│   └── timer.hpp           # 计时工具
├── src/
│   └── main.cpp            # 主程序入口
├── lib/detector/           # 检测器核心库
│   ├── RunTime.hpp         # 运行时调度器
│   ├── BaseNet.hpp         # 网络推理基类
│   ├── OpencvNet.hpp       # OpenCV DNN 推理实现
│   ├── BasePostProcess.hpp # 后处理基类 & NMS
│   ├── DetPostProcessV5.hpp# YOLOv5 检测后处理
│   ├── DetPostProcessV8.hpp# YOLOv8 检测后处理
│   ├── PosePostProcessV5.hpp# YOLOv5 姿态后处理
│   ├── PosePostProcessV8.hpp# YOLOv8 姿态后处理
│   ├── NetConfig.hpp       # 模型配置数据结构
│   ├── NetOutput.hpp       # 网络输出特征图容器
│   ├── ObjectBuffer.hpp    # 检测结果缓冲区
│   ├── YoloObject.h        # 检测结果数据结构
│   ├── draw_result.hpp     # 结果可视化绘制
│   └── utils.hpp           # 工具函数 & 配置解析
├── scripts/                # Python 辅助脚本
│   ├── pt2onnx.py          # PyTorch 模型转 ONNX
│   └── run_onnx_img.py     # ONNX 模型推理脚本
└── doc/                    # 详细文档目录
```

## 构建与运行

### 环境依赖

- GCC >= 11 (C++17)
- CMake >= 3.10
- OpenCV (需包含 DNN 模块, 如需 GPU 推理需编译 CUDA 支持)

### 构建步骤

```bash
# 使用构建脚本 (推荐)
./build.sh

# 或手动构建
mkdir build && cd build
cmake .. -DPROJECT_ROOT=$(pwd)/..
make -j12
```

构建参数说明:

- `-DPROJECT_ROOT=[path]`: 指定项目根路径 (运行时文件访问), 默认为 CMakeLists.txt 所在目录
- `-DLOG_COMPILE_LEVEL=[0-5]`: 编译时日志级别 (0=Trace, 1=Debug, 2=Info, 3=Warn, 4=Error, 5=Critical)

### 运行示例

```bash
# 目标检测
./yolo \
  --log_ini_path=../log_config.ini \
  --model_bench=OpenCV \
  --model_ini_path=../config/fire2ClsDet-bn-sim.ini \
  --model_path=../onnx/fire2ClsDet-bn-sim.onnx \
  --device=0 \
  --test_image_path=../test_img/test_fire01.jpg

# 姿态估计
./yolo \
  --log_ini_path=../log_config.ini \
  --model_bench=OpenCV \
  --model_ini_path=../config/yolov8nDetPose-bn.ini \
  --model_path=../onnx/yolov8nPose-bn.onnx \
  --device=0 \
  --test_image_path=../test_img/test_v8pose01.jpg
```

命令行参数说明:

| 参数                | 说明                         |
| ------------------- | ---------------------------- |
| `--log_ini_path`    | 日志配置文件路径             |
| `--model_bench`     | 模型框架 (当前仅支持 OpenCV) |
| `--model_ini_path`  | 模型 INI 配置文件路径        |
| `--model_path`      | ONNX 模型文件路径            |
| `--device`          | GPU 设备编号 (-1 为 CPU)     |
| `--test_image_path` | 测试图片路径                 |

## 详细文档

各模块的详细说明请参见 `doc/` 目录:

- [RunTime 运行时调度器](doc/RunTime.md) - 项目核心调度入口
- [BaseNet 网络推理基类](doc/BaseNet.md) - 网络推理抽象接口
- [OpencvNet OpenCV DNN 推理实现](doc/OpencvNet.md) - 基于 OpenCV DNN 的模型加载与推理
- [BasePostProcess 后处理基类](doc/BasePostProcess.md) - 后处理抽象接口与 NMS 实现
- [DetPostProcessV5 YOLOv5 检测后处理](doc/DetPostProcessV5.md) - YOLOv5 anchor-base 检测解码
- [DetPostProcessV8 YOLOv8 检测后处理](doc/DetPostProcessV8.md) - YOLOv8 anchor-free 检测解码
- [PosePostProcessV5 YOLOv5 姿态后处理](doc/PosePostProcessV5.md) - YOLOv5 姿态估计关键点解码
- [PosePostProcessV8 YOLOv8 姿态后处理](doc/PosePostProcessV8.md) - YOLOv8 姿态估计关键点解码
- [NetConfig 模型配置](doc/NetConfig.md) - 模型配置数据结构与枚举定义
- [NetOutput 网络输出容器](doc/NetOutput.md) - 网络输出特征图存储
- [ObjectBuffer 检测结果缓冲区](doc/ObjectBuffer.md) - 检测结果高效存储与管理
- [YoloObject 检测结果数据结构](doc/YoloObject.md) - 检测结果统一数据结构
- [draw_result 结果可视化](doc/draw_result.md) - 检测框与姿态关键点绘制
- [utils 工具函数](doc/utils.md) - 配置解析与图像预处理工具

## 参考项目

本项目参考了以下开源仓库:

- [args](https://github.com/Taywee/args) - C++ 命令行参数解析库
- [json](https://github.com/nlohmann/json) - C++ JSON 解析库 (JSON for Modern C++)
- [yolov5-face](https://github.com/deepcam-cn/yolov5-face.git) - YOLOv5 人脸检测与关键点检测
- [ultralytics](https://github.com/ultralytics/ultralytics.git) - YOLOv8 / YOLO11 系列模型框架
- [yolov5](https://github.com/ultralytics/yolov5.git) - YOLOv5 目标检测框架

## 许可证

本项目基于 MIT 许可证开源, 详见 [LICENSE](LICENSE) 文件。
