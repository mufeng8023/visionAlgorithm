# doc 文档目录

<!-- vscode-markdown-toc -->
- [doc 文档目录](#doc-文档目录)
  - [核心调度](#核心调度)
  - [网络推理](#网络推理)
  - [后处理](#后处理)
  - [数据结构](#数据结构)
  - [工具函数](#工具函数)

<!-- vscode-markdown-toc -->

## 核心调度

- [RunTime 运行时调度器](RunTime.md) - 项目核心调度入口, 串联推理全流程

## 网络推理

- [BaseNet 网络推理基类](BaseNet.md) - 网络推理抽象接口
- [OpencvNet OpenCV DNN 推理实现](OpencvNet.md) - 基于 OpenCV DNN 的模型加载与推理

## 后处理

- [BasePostProcess 后处理基类](BasePostProcess.md) - 后处理抽象接口与 NMS 实现
- [DetPostProcessV5 YOLOv5 检测后处理](DetPostProcessV5.md) - YOLOv5 anchor-base 检测解码
- [DetPostProcessV8 YOLOv8 检测后处理](DetPostProcessV8.md) - YOLOv8 anchor-free 检测解码
- [DetPostProcess26 YOLOv26 检测后处理](DetPostProcess26.md) - YOLOv26 端到端检测解码
- [PosePostProcessV5 YOLOv5 姿态后处理](PosePostProcessV5.md) - YOLOv5 姿态估计关键点解码
- [PosePostProcessV8 YOLOv8 姿态后处理](PosePostProcessV8.md) - YOLOv8 anchor-free 姿态估计关键点解码
- [PosePostProcess26 YOLOv26 姿态后处理](PosePostProcess26.md) - YOLOv26 端到端姿态估计关键点解码

## 数据结构

- [NetConfig 模型配置](NetConfig.md) - 模型配置数据结构与枚举定义
- [NetOutput 网络输出容器](NetOutput.md) - 网络输出特征图存储
- [ObjectBuffer 检测结果缓冲区](ObjectBuffer.md) - 检测结果高效存储与管理
- [YoloObject 检测结果数据结构](YoloObject.md) - 检测结果统一数据结构

## 工具函数

- [draw_result 结果可视化](draw_result.md) - 检测框与姿态关键点绘制
- [utils 工具函数](utils.md) - 配置解析与图像预处理工具
