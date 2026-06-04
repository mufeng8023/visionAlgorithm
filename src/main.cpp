/***
 * @Author       : gxs
 * @Date         : 2026-05-17 19:56:58
 * @LastEditors  : gxs
 * @LastEditTime : 2026-05-17 19:57:00
 * @FilePath     : /visionAlgorithm/src/main.cpp
 * @Description  :
 * @
 * @Copyright (c) 2026 by gxs, All Rights Reserved.
 */
#include <unordered_map>
#include <vector>

#include "RunTime.hpp"
#include "YoloObject.h"
#include "args.hpp"
#include "logging.hpp"
#include "myFilesystem.hpp"
#include "types.hpp"
#include "version.hpp"

struct ArgsConfig
{
    // 日志配置文件路径
    std::string log_ini_path = "";
    // 模型配置文件
    std::string model_ini_path = "";
    // 模型框架
    yolo::ModelBench model_bench = yolo::ModelBench::OpenCV;
    // 模型配置
    yolo::ModelPathParams model_path_param;
    // 使用的设备编号
    int32 device = -1;
    // 测试图片路径
    std::string test_image_path = "";

} ArgsConfig;  // 一个全局变量,专门接收命令行参数

int32 parser_args(int argc, char* argv[])
{
    // 创建一个 ArgumentParser 对象, 用于解析命令行参数
    // 第一个参数是程序的描述信息, 第二个参数是选项后的额外信息
    args::ArgumentParser parser("visionAlgorithm", "A simple vision algorithm tool.");
    // 添加帮助标志, 用户可以通过 -h 或 --help 查看帮助信息
    args::HelpFlag help(parser, "help", "显示此帮助菜单", {'h', "help"});
    // 添加版本标志, 用户可以通过 -v 或 --version 查看版本信息
    args::Flag version(parser, "version", "显示版本信息", {'v', "version"});

    // 添加值标记,
    // args::ValueFlag<std::string> cfg_path(parser,            命令行解析实例
    //                                       "config_path",     命令行的命令参数
    //                                       "配置文件路径",     命令行参数的描述
    //                                       {"cfg_path"},      命令行参数的别名
    //                                       "./default.json"); 命令行参数的默认值
    // 添加一个
    // args::ValueFlagList<int> box(parser,     命令行解析实例
    //                              "box",      命令行的命令参数;使用方法-b x1 -b y1 ...
    //                              "一个box区域,(x1, y1, x2, y2)",     命令行参数的描述
    //                              {'b', "box"});      命令行参数的别名;

    args::ValueFlag<std::string> log_ini_path(parser, "log_ini_path",
                                              format_string("日志配置目录; 默认: %s", ArgsConfig.log_ini_path.c_str()),
                                              {"log_ini_path"}, ArgsConfig.log_ini_path);

    // 定义允许的可选列表映射
    // 键Key 是用户在命令行输入的字符串, 值Value 是程序实际接收到的值
    const std::unordered_map<std::string, yolo::ModelBench> allowed_models = {
        {"OpenCV", yolo::ModelBench::OpenCV},  // onnx
    };
    // 定义匿名函数
    auto get_keys_string = [](const std::unordered_map<std::string, yolo::ModelBench>& map) -> std::string
    {
        std::string result;
        bool first = true;
        for (const auto& pair : map)
        {
            if (!first)
                result += ", ";
            result += pair.first;
            first = false;
        }
        return result;
    };
    // 使用 args::MapFlag 代替 args::ValueFlag
    args::MapFlag<std::string, yolo::ModelBench> model_bench(
        parser, "model_bench",
        format_string("模型框架, 可选: %s \n默认: %s", get_keys_string(allowed_models).c_str(),
                      model_bench_to_string(ArgsConfig.model_bench).c_str()),
        {"model_bench"},
        allowed_models,         // 传入允许的选择列表
        ArgsConfig.model_bench  // 默认值
    );

    args::ValueFlag<std::string> model_ini_path(
        parser, "model_ini_path", format_string("模型配置文件, 默认: %s", ArgsConfig.model_ini_path.c_str()),
        {"model_ini_path"}, ArgsConfig.model_ini_path);

    args::ValueFlag<std::string> model_path(parser, "model_path", "模型路径", {"model_path"}, "");

    args::ValueFlag<int32> device(parser, "device", format_string("使用的设备编号; 默认: %d", ArgsConfig.device),
                                  {"device"}, ArgsConfig.device);

    args::ValueFlag<std::string> test_image_path(parser, "test_image_path", "测试图片路径", {"test_image_path"}, "");

    try
    {
        // 解析命令行参数
        // args::ParseCLI 函数会解析命令行参数, 并将结果存储在 parser 对象中
        // 如果解析成功, parser 对象将包含所有解析后的参数值
        // 如果解析失败, 将抛出异常
        parser.ParseCLI(argc, argv);

        if (args::get(version))
        {
            std::cout << "version: " << yolo::version_string() << std::endl;
            std::cout << "build time: " << yolo::build_time() << std::endl;
            std::cout << "git branch: " << yolo::git_branch() << std::endl;
            std::cout << "git hash: " << yolo::git_hash() << std::endl;
            return 1;
        }

        // ArgsConfig.cfg_path = args::get(cfg_path);
        // ArgsConfig.box = args::get(box);
        // 上面这两个是使用示例
        // 解析命令行参数
        ArgsConfig.log_ini_path = args::get(log_ini_path);
        ArgsConfig.model_bench = args::get(model_bench);
        ArgsConfig.model_ini_path = args::get(model_ini_path);
        ArgsConfig.device = args::get(device);
        ArgsConfig.test_image_path = args::get(test_image_path);

        if (ArgsConfig.model_bench == yolo::ModelBench::OpenCV)
        {
            ArgsConfig.model_path_param.onnx_path = args::get(model_path);
        }
        else
        {
            LOG_DEFAULT_ERROR("模型框架暂不支持");
            return 3;  // 返回错误码 3 表示模型框架暂不支持
        }
    }
    catch (const args::Completion& e)
    {
        std::cerr << e.what() << '\n';
        return 0;
    }
    catch (const args::Help&)
    {
        std::cout << parser;
        return 1;
    }
    catch (const args::ParseError& e)
    {
        // 处理解析错误, 输出错误信息和帮助信息
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 2;
    }

    return 0;
}

void ini_init_logger(const std::string& ini_path)
{
    if (ini_path.empty())
    {
        throw std::runtime_error("日志的配置文件路径不能为空");
    }

    try
    {
        IniParser parser;
        // 加载配置文件
        parser.load(ini_path);

        // 获取所有节, 每个节是一个日志的配置
        std::vector<std::string> sections = parser.get_sections();
        for (const std::string& section : sections)
        {
            // 实例化一个日志配置器
            LoggerConfig logger_config;
            // 日志名字
            logger_config.m_customLogName = parser.get_string(section, "custom_log_name", section);
            // 设置日志级别; Trace=0, Debug=1, Info=2, Warn=3, Error=4, Critical=5
            logger_config.m_level = intToLogLevel(parser.get_int(section, "log_level", 2));
            // 设置日志文件存放路径
            logger_config.m_folderPath =
                parser.get_string(section, "log_folder", format_string("../logs/%s", section.c_str()));
            // 设置单个日志文件的最大大小
            logger_config.m_maxFileSizeMB = parser.get_int(section, "max_file_size_mb", 10);
            // 设置最多保留的日志文件数量
            logger_config.m_maxFiles = parser.get_int(section, "max_files", 5);
            // 设置是否启用控制台输出
            logger_config.m_consoleOutput = parser.get_bool(section, "enable_console", false);
            // 设置是否启用进程安全
            logger_config.m_processSafe = parser.get_bool(section, "process_safe", false);

            // 创建日志器
            LogManager::getInstance().createLogger(logger_config.m_customLogName, logger_config);

            // 设置默认日志器
            if (section == "default")
            {
                LogManager::getInstance().setDefaultLogger(logger_config.m_customLogName);
            }
        }
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("初始化日志器失败: " + std::string(e.what()));
    }
}

int32 main(int argc, char* argv[])
{
    // 初始化命令行参数
    int32 return_code = parser_args(argc, argv);
    if (return_code > 0)
    {
        if (return_code == 1)  // Help was requested
            return 0;          // Help was requested, exit gracefully

        return return_code;
    }

    // 初始化日志器
    if (!ArgsConfig.log_ini_path.empty())
    {
        ini_init_logger(ArgsConfig.log_ini_path);
    }
    else
    {
        throw std::runtime_error("日志的配置文件路径不能为空");
    }

    // 输出版本信息
    LOG_DEFAULT_INFO("version: %s", yolo::version_string());
    LOG_DEFAULT_INFO("build time: %s", yolo::build_time());
    LOG_DEFAULT_INFO("git branch: %s", yolo::git_branch());
    LOG_DEFAULT_INFO("git hash: %s", yolo::git_hash());

    // 初始化模型运行时
    yolo::RunTime run_time(ArgsConfig.model_ini_path,    // 模型配置文件路径
                           ArgsConfig.model_path_param,  // 模型路径参数
                           ArgsConfig.model_bench,       // 模型框架
                           ArgsConfig.device);           // 使用的设备编号

    // 运行模型
    // 读图片
    cv::Mat image = cv::imread(ArgsConfig.test_image_path);
    if (image.empty())
    {
        LOG_DEFAULT_ERROR("读取图片失败");
        return 1;
    }

    // 运行模型
    std::vector<cv::Mat> images_bgr;
    images_bgr.push_back(image);
    // 保存结果的对象
    std::vector<std::vector<yolo::YoloObject>> det_results;
    for (uint32 i = 0; i < 200; ++i)
    {
        LOG_DEFAULT_INFO("运行模型");
        run_time(images_bgr, det_results);
    }

    // 绘制边界框
    run_time.draw_result(images_bgr, det_results);

    // 保存会之后的结果
    std::string save_path = myfs::path_join("../test_res_temp", myfs::path_basename(ArgsConfig.test_image_path));
    myfs::makedirs(myfs::path_dirname(save_path));
    cv::imwrite(save_path, images_bgr[0]);

    return 0;
}
