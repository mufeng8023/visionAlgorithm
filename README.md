# visionAlgorithm

<!-- vscode-markdown-toc -->
- [visionAlgorithm](#visionalgorithm)
  - [项目简介](#项目简介)
  - [构建与运行](#构建与运行)
    - [环境依赖](#环境依赖)
    - [构建步骤](#构建步骤)
    - [运行示例](#运行示例)
  - [详细文档](#详细文档)
  - [许可证](#许可证)

<!-- vscode-markdown-toc -->

## 项目简介

visionAlgorithm 是一个基于 C++17 的轻量级 YOLO 系列模型推理框架, 支持通过 OpenCV DNN 后端加载 ONNX 模型, 完成目标检测与姿态估计任务。项目采用模块化设计, 将模型加载、推理、后处理、结果可视化等环节解耦, 便于扩展和维护。

> 各模块的详细设计文档请参见 [doc/](doc/README.md) 目录。

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

- `-DPROJECT_ROOT=[path]`: 指定项目根路径 (运行时文件访问), 默认为 CMakeLists.txt 所在目录; 影响打印的日志显示的路径`2026-05-31 21:15:48 [    INFO] lib/detector/utils.hpp : ...` 指定了`PROJECT_ROOT`会显示相对路径, 不指定显示绝对路径
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

- [doc 文档目录](doc/README.md) - 文档索引
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

## 许可证

本项目基于 MIT 许可证开源, 详见 [LICENSE](LICENSE) 文件。
