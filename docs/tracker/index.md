# doc 文档目录

<!-- vscode-markdown-toc -->
- [doc 文档目录](#doc-文档目录)
  - [核心调度](#核心调度)
  - [卡尔曼滤波](#卡尔曼滤波)
  - [数据结构](#数据结构)
  - [跟踪器实现](#跟踪器实现)
    - [ByteTrack](#bytetrack)
    - [DeepSORT](#deepsort)
  - [工具函数](#工具函数)

<!-- vscode-markdown-toc -->

## 核心调度

- [TrackerRuntime 跟踪运行时调度器](tracker/TrackerRuntime.md) - 跟踪器统一调度入口, 封装 ByteTrack/DeepSORT, 提供统一高级接口

## 卡尔曼滤波

- [BaseKalmanFilter 卡尔曼滤波器泛型基类](tracker/BaseKalmanFilter.md) - 模板化卡尔曼滤波核心, 支持任意状态/观测维度
- [BoxKalmanFilter Box 跟踪卡尔曼滤波器](tracker/BoxKalmanFilter.md) - 8 维恒定速度模型, 适用于 SORT/ByteTrack/DeepSORT
- [KeypointKalmanFilter 关键点平滑卡尔曼滤波器](tracker/KeypointKalmanFilter.md) - 4 维恒定速度模型, 适用于姿态关键点平滑

## 数据结构

- [BaseTrack 轨迹基类](tracker/BaseTrack.md) - 定义边界框双轨存储 (ltwh / ltwh_expand) 与卡尔曼状态管理
- [BoxObject 检测目标数据结构](tracker/BoxObject.md) - 跟踪器输入检测目标的统一数据结构
- [TrackResult 跟踪结果结构体](tracker/TrackResult.md) - 跟踪器输出结果的统一数据结构 (含 det_index 回溯机制)
- [TrackHistory 轨迹历史记录](tracker/TrackHistory.md) - 轨迹多帧历史记录, 支持客流量统计与行为分析
- [TrackState 轨迹状态枚举](tracker/TrackState.md) - 轨迹生命周期状态定义与状态机转换
- [TrackerConfig 跟踪器配置](tracker/TrackerConfig.md) - 跟踪器配置数据结构与枚举定义

## 跟踪器实现

### ByteTrack

- [ByteTracker ByteTrack 跟踪器](tracker/bytetrack/ByteTracker.md) - 高低分检测框三轮 IoU 关联, 继承 BaseTracker
- [BytetrackTrack ByteTrack 轨迹](tracker/bytetrack/BytetrackTrack.md) - ByteTrack 轨迹类, 每个轨迹持有自己的 KFBox 副本
- [matching 匹配算法](tracker/bytetrack/matching.md) - IoU 距离计算, LAPJV 匈牙利匹配, 轨迹列表管理工具

### DeepSORT

- [DeepSortTracker DeepSORT 跟踪器](tracker/deepsort/DeepSortTracker.md) - 级联匹配 + IoU 二次匹配, 继承 BaseTracker
- [DeepSortTrack DeepSORT 轨迹](tracker/deepsort/DeepSortTrack.md) - DeepSORT 轨迹类, 共享全局 KFBox, 支持 ReID 特征
- [NNMetric 外观特征度量学习](tracker/deepsort/NNMetric.md) - 余弦距离计算与最近邻匹配, 特征库增量维护
- [matching 匹配算法](tracker/deepsort/matching.md) - 级联匹配, 马氏距离门控, 融合距离矩阵计算

## 工具函数

- [utils 工具函数](tracker/utils.md) - INI 配置解析与跟踪器创建工厂
- [version 版本信息](tracker/version.md) - 版本号与编译信息查询
