# 🛠 common 通用组件

> common 模块提供 visionAlgorithm 框架通用的基础设施组件, 包括日志系统、文件系统工具和高性能计时器;

## 📄 文档列表

| 文档                      | 说明                                                         | 链接                                      |
| ------------------------- | ------------------------------------------------------------ | ----------------------------------------- |
| logging 日志系统          | 基于 C++17 的多线程安全日志管理器, 支持多级日志与格式化输出  | [logging.md](common/logging.md)           |
| myFilesystem 文件系统工具 | 对 C++17 `<filesystem>` 的封装, 提供路径操作、目录遍历等接口 | [myFilesystem.md](common/myFilesystem.md) |
| timer 计时器工具          | 高性能计时器库, 包含单次计时器、计时器管理器和便捷宏定义     | [timer.md](common/timer.md)               |

## 功能概览

### 📝 logging 日志系统

- 采用 LogManager 单例管理多个 Logger 实例
- 支持多线程、多进程安全
- 提供丰富的日志级别和灵活的配置选项

### 📂 myFilesystem 文件系统工具

- 对 C++17 标准 `<filesystem>` 的一层封装
- 提供更简洁、易用的文件系统操作接口
- 所有函数位于 `myfs` 命名空间, 同时提供 `std::string` 和 `fs::path` 两种重载

### ⏱ timer 计时器工具

- 包含 `Timer` 单次计时器、`TimerManager` 单例计时器管理器
- 提供便捷的宏定义用于快速性能分析
- 支持 TIMER_DEBUG 编译开关控制

## 🔗 相关章节

- [detector 核心模块](detector/index.md)
- [项目首页](README.md)
