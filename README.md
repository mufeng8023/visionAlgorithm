# visionAlgorithm

<!-- vscode-markdown-toc -->
- [visionAlgorithm](#visionalgorithm)
  - [项目简介](#项目简介)
  - [构建与运行](#构建与运行)
    - [环境依赖](#环境依赖)
    - [构建步骤](#构建步骤)
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
anchors = { {4, 5, 8, 10, 13, 16}, {23, 29, 43, 55, 73, 105}, {146, 217, 231, 300, 335, 433} }
```

**anchor-free 检测模型 (yolov8 VOC 检测):**

```ini
[detection]
model_name = yolov8nDetVOC-bn
model_type = yolov8
task = detection
has_conf = false
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
| YOLOv5 (anchor-base) | `fire2ClsDet-bn-sim.ini`   | 自定义 | 烟火检测 (fire, smoke)             |
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

| 模型架构          | 配置文件                  | 关键点       | 说明                     |
| ----------------- | ------------------------- | ------------ | ------------------------ |
| YOLOv5-face (2pt) | `yolov5ssFaceKpt2-bn.ini` | 5 点 (2 维)  | 人脸关键点, 仅 (x, y)    |
| YOLOv5-face (3pt) | `yolov5ssFaceKpt3-bn.ini` | 5 点 (3 维)  | 人脸关键点, (x, y, conf) |
| YOLOv8-pose       | `yolov8nDetPose-bn.ini`   | 17 点 (3 维) | 人体姿态估计             |
| YOLO26-pose       | `yolo26nPose-bn.ini`      | 17 点 (3 维) | 人体姿态估计, 端到端     |

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
