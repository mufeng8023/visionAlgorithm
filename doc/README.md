# visionAlgorithm

<!-- vscode-markdown-toc -->
- [visionAlgorithm](#visionalgorithm)
  - [项目简介](#项目简介)
  - [链接](#链接)
  - [构建与运行](#构建与运行)
    - [环境依赖](#环境依赖)
    - [构建步骤](#构建步骤)
    - [CMake 选项说明](#cmake-选项说明)
    - [编译器与 C++ 标准](#编译器与-c-标准)
    - [版本信息注入](#版本信息注入)
    - [运行示例](#运行示例)
  - [模型 INI 配置文件说明](#模型-ini-配置文件说明)
    - [配置项总览](#配置项总览)
    - [模型类型说明](#模型类型说明)
    - [配置示例](#配置示例)
  - [支持的模型列表](#支持的模型列表)
    - [目标检测模型](#目标检测模型)
    - [姿态估计模型](#姿态估计模型)
  - [详细文档](#详细文档)
  - [许可证](#许可证)

## 项目简介

visionAlgorithm 是一个基于 C++17 的轻量级 YOLO 系列模型推理框架, 支持通过 OpenCV DNN 后端加载 ONNX 模型, 完成目标检测与姿态估计任务。项目采用模块化设计, 将模型加载、推理、后处理、结果可视化等环节解耦, 便于扩展和维护。

> 各模块的详细设计文档请参见 [doc/](doc/README.md) 目录。

## 链接

- **GitHub 仓库**: [https://github.com/mufeng8023/visionAlgorithm.git](https://github.com/mufeng8023/visionAlgorithm.git)
- **GitHub Pages 文档**: [https://mufeng8023.github.io/visionAlgorithm](https://mufeng8023.github.io/visionAlgorithm)

## 构建与运行

### 环境依赖

- GCC >= 11 (C++17)
- CMake >= 3.10
- OpenCV (需包含 DNN 模块, 如需 GPU 推理需编译 CUDA 支持)

### 构建步骤

项目提供两个构建脚本, 分别用于 Debug 和 Release 构建:

```bash
# Debug 构建 (启用 TIMER_DEBUG 宏, 默认日志级别为 Debug(1))
./build_debug.sh

# Release 构建 (禁用 TIMER_DEBUG 宏, 默认日志级别为 Info(2))
./build_release.sh
```

构建脚本内部逻辑 (以 `build_debug.sh` 为例):

```bash
rm -r build          # 清空旧的 build 目录
mkdir build          # 重新创建 build 目录
cd build
cmake .. \
    -DPROJECT_ROOT=$(pwd)/.. \
    -DDEBUG=ON
make clean           # 清理上一次编译产物
make -j12            # 编译 (使用 12 个并行任务)
cd ..
```

> `build_release.sh` 仅将 `-DDEBUG=ON` 替换为 `-DDEBUG=OFF`, 其余逻辑完全相同。

也可手动构建, 通过 CMake 选项灵活控制编译行为:

```bash
mkdir build && cd build

# Debug 构建示例
cmake .. \
    -DPROJECT_ROOT=$(pwd)/.. \
    -DDEBUG=ON \
    -DLOG_LEVEL=1

# Release 构建示例
cmake .. \
    -DPROJECT_ROOT=$(pwd)/.. \
    -DDEBUG=OFF \
    -DLOG_LEVEL=2

# 每次修改 CMake 选项后, 务必 clean 后完整重编
make clean
make -j$(nproc)
```

> **重要**: CMakeLists.txt 中通过 `set(CMAKE_CXX_COMPILER /usr/bin/g++-11)` 显式指定了 g++-11 编译器。如果你的环境中 g++-11 路径不同, 或希望使用其他版本, 请通过 `-DCMAKE_CXX_COMPILER=/path/to/g++` 覆盖。C++ 标准固定为 C++17 (`-std=c++17`)。
>
> 构建产物为名为 `yolo` 的可执行文件, 在 `build/` 目录下生成。

### CMake 选项说明

| 选项                    | 类型   | 默认值 | 说明                                                                                                                                                                     |
| ----------------------- | ------ | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-DPROJECT_ROOT=[path]` | STRING | 自动   | 指定项目根路径 (运行时文件访问). 默认为 CMakeLists.txt 所在目录 (`CMAKE_SOURCE_DIR`). 指定后影响日志输出的路径显示, 例如显示相对路径`lib/detector/utils.hpp`而非绝对路径 |
| `-DDEBUG=ON/OFF`        | BOOL   | OFF    | 是否启用 Debug 模式. **ON**: 向编译器注入 `TIMER_DEBUG` 宏, 可在代码中启用计时/调试逻辑                                                                                  |
| `-DLOG_LEVEL=[0-5]`     | STRING | 自动   | 编译时日志级别, 可选值: `0`=Trace, `1`=Debug, `2`=Info, `3`=Warn, `4`=Error, `5`=Critical                                                                                |

> **日志级别与 DEBUG 模式的联动规则:**
>
> - **`DEBUG=ON`**: 默认日志级别为 **Debug(1)**, 可通过 `-DLOG_LEVEL=N` 覆盖为任意值 (0~5);
> - **`DEBUG=OFF`**: 默认日志级别为 **Info(2)**, 可通过 `-DLOG_LEVEL=N` 上调级别; 但 **不允许** 设置为低于 Info(2) 的值 (即 0 或 1), 若传入此类值, CMake 会发出警告并强制回退为 Info(2);
>
> 以上所有选项均通过预处理器宏 (`-D`) 在编译期注入代码, 更改后需 **重新运行 cmake 并 clean 后完整重编** 方可生效。

### 编译器与 C++ 标准

- 编译器: `g++-11` (通过 `CMAKE_CXX_COMPILER` 指定, 位于 `/usr/bin/g++-11`)
- C++ 标准: **C++17** (`-std=c++17`)
- 编译选项: `-Wall -g -O2` (启用警告、调试符号、二级优化)
- 构建系统: `make` (CMake 生成的 Unix Makefiles)

### 版本信息注入

构建时自动将以下信息编译进可执行文件 (通过 `-D` 宏定义注入):

| 宏定义                | 来源                              | 示例                       |
| --------------------- | --------------------------------- | -------------------------- |
| `DETECT_BUILD_TIME`   | 系统时间戳                        | `20260605_174510`          |
| `DETECT_GIT_HASH`     | `git rev-parse HEAD`              | `df6da12c225690676...`     |
| `DETECT_GIT_BRANCH`   | `git rev-parse --abbrev-ref HEAD` | `main`                     |
| `DETECT_PROJECT_NAME` | CMake `project()` 名              | `yolo`                     |
| `PROJECT_ROOT`        | CMake 选项 (`-DPROJECT_ROOT`)     | 见上方 `PROJECT_ROOT` 说明 |
| `LOG_COMPILE_LEVEL`   | CMake 选项 (`-DLOG_LEVEL`)        | `1` (Debug), `2` (Info)    |

> 版本信息定义位于 `lib/detector/version.h`, 运行时可通过相应宏查看当前可执行文件的编译时间、Git Commit 等信息。

### 运行示例

```bash
# 目标检测
./yolo \
  --log_ini_path=../log_config.ini \
  --model_bench=OpenCV \
  --model_ini_path=../config/yolov5nDetVOC-bn.ini \
  --model_path=../onnx/yolov5nDetVOC-bn.onnx \
  --device=0 \
  --test_image_path=../test_img/test_v8pose01.jpg

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

## 模型 INI 配置文件说明

每个模型在运行时都需要一个对应的 INI 配置文件, 通过 `--model_ini_path` 参数指定。配置文件采用 `[detection]` 节(section) 组织, 支持 `;` 和 `#` 注释。

### 配置项总览

| 配置项          | 必填   | 默认值  | 说明                                                                          |
| --------------- | ------ | ------- | ----------------------------------------------------------------------------- |
| `model_name`    | 是     | -       | 模型标识名称, 如 `yolov8nDetVOC-bn`                                           |
| `model_type`    | 是     | -       | 模型架构类型, 可选值见下方                                                    |
| `task`          | 是     | -       | 任务类型: `classification`, `detection`, `segmentation`, `pose`, `obb`        |
| `has_conf`      | 否     | `false` | 输出是否包含置信度; anchor-base 模型设为 `true`, anchor-free 模型设为 `false` |
| `names`         | 是     | -       | 类别名称列表, 格式: `{ "cls1", "cls2" }`                                      |
| `scale_outputs` | 是     | -       | 输出反量化系数, 每个输出层对应一个, 非量化模型设为 `{1.0, 1.0, 1.0}`          |
| `conf_thrs`     | 否     | `{0.3}` | 置信度阈值, 可对每个类别单独设置, 不足时用最后一个值填充                      |
| `iou_thrs`      | 否     | `0.45`  | NMS 的 IoU 阈值                                                               |
| `max_det`       | 否     | `300`   | 每张图片最大检测目标数                                                        |
| `agnostic`      | 否     | `false` | 是否进行类别无关 NMS; `false` 为类别相关 NMS, `true` 为类别无关 NMS           |
| `batch_size`    | 否     | `1`     | Batch Size                                                                    |
| `kpt_count`     | 否     | `0`     | 关键点数量, 仅 `pose` 任务需要设置                                            |
| `kpt_dim`       | 否     | `0`     | 关键点维度, 通常为 `3` (x, y, confidence); yolov5-face 为 `2` (x, y)          |
| `input_chw`     | 是     | -       | 输入图像尺寸, 格式: `{channels, height, width}`, 如 `{3, 384, 640}`           |
| `strides`       | 是     | -       | 各输出层的步长, 如 `{8, 16, 32}`                                              |
| `anchors`       | 见说明 | -       | Anchor 信息; anchor-base 模型必填, anchor-free 模型留空 `{}`                  |

### 模型类型说明

`model_type` 可选值及其特性:

| 模型类型                                   | 解码方式             | 是否需要 anchors | 是否需要 NMS |
| ------------------------------------------ | -------------------- | ---------------- | ------------ |
| `yolov3`, `yolov4`, `yolov5`               | anchor-base          | 是               | 是           |
| `yolov3u`, `yolov5u`, `yolov6u`, `yolov7u` | anchor-free          | 否               | 是           |
| `yolov8`, `yolov9`, `yolo11`, `yolo12`     | anchor-free          | 否               | 是           |
| `yolov10`, `yolo26`                        | anchor-free (端到端) | 否               | 否           |

### 配置示例

**anchor-base 检测模型 (yolov5 人脸检测):**

```ini
[detection]
model_name = yolov5ssFaceDet-bn
model_type = yolov5
task = detection
has_conf = true
names = { "face" }
scale_outputs = {1.0, 1.0, 1.0}
conf_thrs = {0.35}
iou_thrs = 0.45
max_det = 300
agnostic = false
batch_size = 1
kpt_count = 0
kpt_dim = 0
input_chw = {3, 384, 640}
strides = {8, 16, 32}
# 数组不支持换行
anchors = { {4, 5, 8, 10, 13, 16}, {23, 29, 43, 55, 73, 105}, {146, 217, 231, 300, 335, 433} }
```

**anchor-free 检测模型 (yolov8 VOC 检测):**

```ini
[detection]
model_name = yolov8nDetVOC-bn
model_type = yolov8
task = detection
has_conf = false
# 数组不支持换行
names = { "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor" }
scale_outputs = {1.0, 1.0, 1.0}
conf_thrs = {0.4}
iou_thrs = 0.45
max_det = 300
agnostic = false
batch_size = 1
kpt_count = 0
kpt_dim = 0
input_chw = {3, 384, 640}
strides = {8, 16, 32}
anchors = {}
```

**姿态估计模型 (yolov8-pose):**

```ini
[detection]
model_name = yolov8nDetPerson-bn
model_type = yolov8
task = pose
has_conf = false
names = { "person" }
scale_outputs = {1.0, 1.0, 1.0}
conf_thrs = {0.4}
iou_thrs = 0.45
max_det = 300
agnostic = false
batch_size = 1
kpt_count = 17
kpt_dim = 3
input_chw = {3, 384, 640}
strides = {8, 16, 32}
anchors = {}
```

> 所有预置的 INI 配置文件位于 `config/` 目录下, 可直接参考使用。

## 支持的模型列表

本项目目前已适配以下 YOLO 系列模型, 所有模型均通过 OpenCV DNN 后端加载 ONNX 格式进行推理。

### 目标检测模型

| 模型架构             | 配置文件                   | 数据集 | 说明                               |
| -------------------- | -------------------------- | ------ | ---------------------------------- |
| YOLOv3-ultralytics   | `yolov3uDetVOC-bn.ini`     | VOC    | anchor-free 检测                   |
| YOLOv3u-SPP          | `yolov3uSppDetVOC-bn.ini`  | VOC    | anchor-free 检测, SPP 变体         |
| YOLOv3u-Tiny         | `yolov3uTinyDetVOC-bn.ini` | VOC    | anchor-free 检测, 轻量版 (2 层)    |
| YOLOv5 (anchor-base) | `yolov5nDetVOC-bn.ini`     | VOC    | anchor-base 检测                   |
| YOLOv5 (anchor-base) | `yolov5ssFaceDet-bn.ini`   | 自定义 | 人脸检测                           |
| YOLOv5-ultralytics   | `yolov5nuDetVOC-bn.ini`    | VOC    | anchor-free 检测                   |
| YOLOv6-ultralytics   | `yolov6nuDetVOC-bn.ini`    | VOC    | anchor-free 检测                   |
| YOLOv8               | `yolov8nDetVOC-bn.ini`     | VOC    | anchor-free 检测                   |
| YOLOv8               | `yolov8nDetFace-bn.ini`    | 自定义 | 人脸检测                           |
| YOLOv8               | `yolov8nDetPerson-bn.ini`  | 自定义 | 行人检测                           |
| YOLOv8-P2            | `yolov8nP2DetVOC-bn.ini`   | VOC    | anchor-free 检测, P2 输出头 (4 层) |
| YOLOv8-P6            | `yolov8nP6DetVOC-bn.ini`   | VOC    | anchor-free 检测, P6 输出头 (4 层) |
| YOLOv9               | `yolov9tDetVOC-bn.ini`     | VOC    | anchor-free 检测                   |
| YOLOv10              | `yolov10nDetVOC-bn.ini`    | VOC    | 端到端检测, 无需 NMS               |
| YOLO11               | `yolo11nDetVOC-bn.ini`     | VOC    | anchor-free 检测                   |
| YOLO12               | `yolo12nDetVOC-bn.ini`     | VOC    | anchor-free 检测                   |
| YOLO26               | `yolo26nDetVOC-bn.ini`     | VOC    | 端到端检测, 无需 NMS               |

### 姿态估计模型

| 模型架构        | 配置文件                  | 关键点       | 说明                            |
| --------------- | ------------------------- | ------------ | ------------------------------- |
| YOLOv5-faceKpt2 | `yolov5ssFaceKpt2-bn.ini` | 5 点 (2 维)  | 人脸关键点, 仅 (x, y)           |
| YOLOv5-faceKpt3 | `yolov5ssFaceKpt3-bn.ini` | 5 点 (3 维)  | 人脸关键点, (x, y, v)           |
| YOLOv8-pose     | `yolov8nDetPose-bn.ini`   | 17 点 (3 维) | 人体姿态估计, (x, y, v)         |
| YOLO26-pose     | `yolo26nPose-bn.ini`      | 17 点 (3 维) | 人体姿态估计, 端到端, (x, y, v) |

> 所有预置的 ONNX 模型文件应位于 `onnx/` 目录下, 对应的 INI 配置文件位于 `config/` 目录下。
> onnx 模型请参考 release 中下载;

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
