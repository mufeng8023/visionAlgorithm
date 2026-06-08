<!-- ============================================================================
   侧边栏配置文件 (_sidebar.md)
   ============================================================================
   使用说明：
     此文件由 docsify 自动加载（通过 index.html 中的 loadSidebar 配置）。
     docsify 会解析这个文件中的 Markdown 链接，生成侧边栏导航。

   格式说明：
     - 使用 Markdown 列表格式
     - 每个列表项是一个导航链接：[显示文字](文件路径)
     - 使用缩进表示层级嵌套
     - 文件路径相对于 docs/ 目录（或 index.html 所在目录）

   ============================================================================
   多子目录索引方案说明：
   ============================================================================
   本项目的 docs/ 下有 detector/ 和 common/ 两个子目录模块，
   采用方案二（章节分组 + 关键页链接）组织侧边栏，
   各子目录下创建 README.md 作为章节索引页，列出完整页面清单。

   详细说明请参考 目录首页索引指南.md
   ============================================================================ -->

- 📖 **visionAlgorithm**
  - [首页](README.md)

- 🧩 **detector 核心模块**
  - [📋 模块索引](detector/index.md)
  - [🚀 RunTime 运行时调度器](detector/RunTime.md)
  - **网络推理**
    - [BaseNet 网络推理基类](detector/BaseNet.md)
    - [OpencvNet OpenCV DNN 推理](detector/OpencvNet.md)
  - **后处理**
    - [BasePostProcess 后处理基类](detector/BasePostProcess.md)
    - [DetPostProcessV5 YOLOv5 检测](detector/DetPostProcessV5.md)
    - [DetPostProcessV8 YOLOv8 检测](detector/DetPostProcessV8.md)
    - [DetPostProcess26 YOLOv26 检测](detector/DetPostProcess26.md)
    - [PosePostProcessV5 YOLOv5 姿态](detector/PosePostProcessV5.md)
    - [PosePostProcessV8 YOLOv8 姿态](detector/PosePostProcessV8.md)
    - [PosePostProcess26 YOLOv26 姿态](detector/PosePostProcess26.md)
  - **数据结构**
    - [NetConfig 模型配置](detector/NetConfig.md)
    - [NetOutput 网络输出容器](detector/NetOutput.md)
    - [ObjectBuffer 检测结果缓冲区](detector/ObjectBuffer.md)
    - [YoloObject 检测结果数据结构](detector/YoloObject.md)
  - **工具函数**
    - [draw_result 结果可视化](detector/draw_result.md)
    - [utils 工具函数](detector/utils.md)

- 🛠 **common 通用组件**
  - [📋 模块索引](common/index.md)
  - [logging 日志系统](common/logging.md)
  - [myFilesystem 文件系统工具](common/myFilesystem.md)
  - [timer 计时器工具](common/timer.md)

<!-- ============================================================================
   设计说明：
   ============================================================================
   detector 模块页面较多（15+ 页），因此在侧边栏中直接展开全部页面（方案一），
   方便快速导航。common 模块页面较少（3 页），直接列出即可。

   各子目录下的 index.md 作为章节索引页，
   访客点击后可在内容区看到该模块的完整文档列表。
   ============================================================================ -->