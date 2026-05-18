/***
 * @Author       : gxs
 * @Date         : 2025-08-09 14:16:08
 * @LastEditors  : gxs
 * @LastEditTime : 2025-08-09 14:16:12
 * @FilePath     : /projectTemplate/include/ini_parser.hpp
 * @Description  :
 * @
 * @Copyright (c) 2025 by gxs, All Rights Reserved.
 */
#ifndef __INI_PARSER__H__  // 如果宏 __INI_PARSER__H__ 未定义
#define __INI_PARSER__H__  // 定义宏 __INI_PARSER__H__, 防止头文件被重复包含

#include <algorithm>      // 引入算法库, 例如 for_each, find_if
#include <cctype>         // 引入字符处理函数库, 例如 isspace
#include <fstream>        // 引入文件流库, 用于文件读写
#include <regex>          // 引入正则表达式库
#include <sstream>        // 引入字符串流库, 用于字符串解析
#include <stdexcept>      // 引入标准异常库
#include <string>         // 引入字符串处理库
#include <unordered_map>  // 引入无序哈希表库
#include <vector>         // 引入动态数组库

// 定义 IniParser 类, 用于解析 INI 文件
class IniParser
{
   private:  // 私有成员
    // 模板函数: 解析形如 "{a, b, c}" 的数组字符串为 std::vector
    /***
     * @description: 解析形如 "{a, b, c}" 的数组字符串为 std::vector
     * @template T {int, double, std::string, bool} : 模板参数 T, 表示数组元素的类型
     * @const {std::string &array_str} : 输入的数组字符串, 形如 "{1, 2, 3}" 或 "{'a', 'b', 'c'}"
     * @note: 该函数使用正则表达式匹配数组元素, 并根据模板参数 T 的类型进行转换,
     *        支持 int、double、std::string 和 bool 类型的数组元素,
     *        对于 std::string 类型, 支持被单引号或双引号包围的字符串,
     *        对于 bool 类型, 支持 "true"、"false"、"yes"、"no"、"1" 和 "0" 的字符串表示,
     *        如果解析失败 (例如格式不正确或类型转换异常) , 则忽略该元素,
     * @example
     * std::vector<int> int_array = parser.parse_array<int>("{1, 2, 3}");
     * std::vector<double> double_array = parser.parse_array<double>("{1.1, 2.2, 3.3}");
     * std::vector<std::string> string_array = parser.parse_array<std::string>("{'a', 'b', 'c'}");
     * std::vector<bool> bool_array = parser.parse_array<bool>("{true, false, yes, no}");
     * std::vector<std::string> mixed_array = parser.parse_array<std::string>("{'a', 1, 2.5, true}");
     * std::vector<int> empty_array = parser.parse_array<int>("{}"); // 返回空的 vector
     * std::vector<double> invalid_array = parser.parse_array<double>("invalid"); // 返回空的 vector
     * std::vector<std::string> quoted_array = parser.parse_array<std::string>("{'a', 'b', 'c'}"); // 支持单引号
     * std::vector<std::string> double_quoted_array = parser.parse_array<std::string>("{\"a\", \"b\", \"c\"}"); //
     * 支持双引号 std::vector<bool> bool_array = parser.parse_array<bool>("{true, false, yes, no, 1, 0}");
     * std::vector<int> mixed_array = parser.parse_array<int>("{1, '2', 3.5, true}"); // 混合类型, 忽略无法转换的元素
     * std::vector<std::string> empty_array = parser.parse_array<std::string>("{}"); // 返回空的 vector
     * std::vector<double> invalid_array = parser.parse_array<double>("invalid"); // 返回空的 vector
     * std::vector<std::string> quoted_array = parser.parse_array<std::string>("{'a', 'b', 'c'}"); // 支持单引号
     * std::vector<std::string> double_quoted_array = parser.parse_array<std::string>("{\"a\", \"b\", \"c\"}"); //
     * 支持双引号 std::vector<bool> bool_array = parser.parse_array<bool>("{true, false, yes, no, 1, 0}");
     * std::vector<int> mixed_array = parser.parse_array<int>("{1, '2', 3.5, true}"); // 混合类型, 忽略无法转换的元素
     * std::vector<std::string> empty_array = parser.parse_array<std::string>("{}");    // 返回空的 vector
     * @param &array_str {string} :
     * @return {}
     */
    template <typename T>                                           // 模板参数 T, 表示数组元素的类型
    std::vector<T> parse_array(const std::string& array_str) const  // const 成员函数, 不修改对象状态
    {
        // 创建一个空的 vector 用于存放解析结果
        std::vector<T> result;
        // 定义正则表达式, 匹配非逗号和花括号的元素
        std::regex element_regex(R"(([^,{}]+))");
        // 创建正则表达式迭代器, 从字符串开始位置进行匹配
        auto elements_begin = std::sregex_iterator(array_str.begin(), array_str.end(), element_regex);
        // 创建一个默认构造的迭代器, 表示匹配结束
        auto elements_end = std::sregex_iterator();

        // 遍历所有匹配到的元素
        // std::sregex_iterator 是一个迭代器, 用于遍历正则表达式匹配的结果
        for (std::sregex_iterator i = elements_begin; i != elements_end; ++i)
        {
            std::smatch match = *i;               // 获取当前的匹配结果
            std::string match_str = match.str();  // 将匹配结果转换为字符串
            this->trim(match_str);                // 去除字符串两端的空白字符

            try  // 使用 try-catch 块处理可能的转换异常
            {
                // 如果模板参数 T 是 int32 类型
                if constexpr (std::is_same_v<T, int32>)
                {
                    // 将字符串转换为 int32 并添加到 vector
                    result.push_back(static_cast<int32>(std::stoi(match_str)));
                }
                // 如果模板参数 T 是 uint32 类型
                else if constexpr (std::is_same_v<T, uint32>)
                {
                    // 将字符串转换为 uint32 并添加到 vector
                    // 先用 stoull 防止大数在 stoul 里由于平台差异被静默截断
                    unsigned long long val = std::stoull(match_str);
                    if (val > UINT32_MAX)
                    {
                        throw std::out_of_range("stoull argument out of uint32 range");
                    }
                    result.push_back(static_cast<uint32_t>(val));
                }
                // 如果模板参数 T 是 int64 类型
                else if constexpr (std::is_same_v<T, int64>)
                {
                    // 将字符串转换为 int64 并添加到 vector
                    result.push_back(std::stoll(match_str));
                }
                // 如果模板参数 T 是 uint64 类型
                else if constexpr (std::is_same_v<T, uint64>)
                {
                    // 将字符串转换为 uint64 并添加到 vector
                    result.push_back(std::stoull(match_str));
                }
                // 如果模板参数 T 是 float32 类型
                else if constexpr (std::is_same_v<T, float>)
                {
                    // 将字符串转换为 float32 并添加到 vector
                    result.push_back(std::stof(match_str));
                }
                // 如果模板参数 T 是 double 类型
                else if constexpr (std::is_same_v<T, double>)
                {
                    // 将字符串转换为 double 并添加到 vector
                    result.push_back(std::stod(match_str));
                }
                // 如果模板参数 T 是 std::string 类型
                else if constexpr (std::is_same_v<T, std::string>)
                {
                    // 如果字符串被引号包围, 则去除引号
                    if (match_str.size() >= 2 &&                                    // 检查字符串长度是否至少为2
                        ((match_str.front() == '"' && match_str.back() == '"') ||   // 检查是否被双引号包围
                         (match_str.front() == '\'' && match_str.back() == '\'')))  // 检查是否被单引号包围
                    {
                        // 提取并添加去除引号后的子字符串
                        result.push_back(match_str.substr(1, match_str.size() - 2));
                    }
                    else  // 如果没有被引号包围
                    {
                        // 直接添加原始字符串
                        result.push_back(match_str);
                    }
                }
                // 如果模板参数 T 是 bool 类型
                else if constexpr (std::is_same_v<T, bool>)
                {
                    // 创建一个副本
                    std::string lower = match_str;
                    // 将字符串转换为小写
                    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
                    // 根据小写字符串判断布尔值并添加
                    result.push_back(lower == "true" || lower == "1" || lower == "yes");
                }
            }
            catch (...)  // 捕获所有类型的异常
            {
                // 忽略解析错误, 继续处理下一个元素
            }
        }
        return result;  // 返回解析后的 vector
    }

   public:  // 公有成员
    // 加载并解析指定的 INI 文件
    /***
     * @description: 加载并解析指定的 INI 文件
     * @param filename {std::string} : INI 文件的路径
     * @note: 该函数读取指定的 INI 文件, 解析其中的节和键值对, 并存储到 data_ 成员变量中,
     *        支持节(section)和键(key)的嵌套结构, 处理空行和注释行,
     *        键值对格式为 "key=value", 支持字符串、整数、浮点数和布尔值,
     *        支持全局节 (没有节名的键值对) 和带有节名的键值对,
     *        如果文件打开失败, 将抛出 std::runtime_error 异常,
     *        解析过程中会去除行首尾的空白字符, 并处理被引号包围的字符串,
     *        支持多种数据类型的转换, 包括 int、double、std::string 和 bool,
     *        对于布尔值, 支持 "true"、"false"、"yes"、"no"、"1" 和 "0" 的字符串表示,
     * @example
     * IniParser parser; // 创建 IniParser 实例
     * parser.load("config.ini"); // 加载并解析 config.ini 文件
     * std::string value = parser.get_string("section", "key", "default_value"); // 获取指定节和键的字符串值
     * int int_value = parser.get_int("section", "key", 42); // 获取指定节和键的整数值, 默认值为 42
     * double double_value = parser.get_double("section", "key", 3.14); // 获取指定节和键的浮点数值, 默认值为 3.14
     * bool bool_value = parser.get_bool("section", "key", true); // 获取指定节和键的布尔值, 默认值为 true
     * std::vector<int> int_array = parser.get_array1d<int>("section", "key"); // 获取一维整数数组
     * std::vector<std::string> string_array = parser.get_array1d<std::string>("section", "key"); // 获取一维字符串数组
     * std::vector<std::vector<double>> double_array = parser.get_array2d<double>("section", "key"); //
     * 获取二维浮点数数组 std::vector<std::string> sections = parser.get_sections(); // 获取所有节的名称
     * std::vector<std::string> keys = parser.get_keys("section"); // 获取指定节中的所有键的名称
     * @throws std::runtime_error : 如果文件打开失败, 将抛出运行时错误,
     * @throws std::invalid_argument : 如果解析过程中遇到无法识别的格式或类型转换错误, 将抛出无效参数异常,
     * @throws std::out_of_range : 如果访问的键或节不存在, 将抛出越界异常,
     * @example
     * IniParser parser; // 创建 IniParser 实例
     * try
     * {
     *     parser.load("config.ini"); // 加载并解析 config.ini 文件
     * }
     * catch (const std::runtime_error &e)
     * {
     *     std::cerr << "Error loading INI file: " << e.what() << std::endl; // 处理文件打开失败的异常
     * }
     * // 获取指定节和键的字符串值
     * std::string value = parser.get_string("section", "key", "default_value");
     */
    void load(const std::string& filename)
    {
        // 创建一个输入文件流对象并尝试打开文件
        std::ifstream file(filename);
        if (!file.is_open())  // 检查文件是否成功打开
        {
            // 如果打开失败, 抛出运行时错误
            throw std::runtime_error("Failed to open file: " + filename);
        }

        // 用于存储从文件中读取的每一行
        std::string line;
        // 用于存储当前正在解析的节(section)的名称
        std::string current_section;

        while (std::getline(file, line))  // 循环读取文件的每一行
        {
            // 去除行首尾的空白字符
            this->trim(line);  // 调用 trim 函数清理行数据

            // 跳过空行和注释行 (以 ';' 或 '#' 开头)
            if (line.empty() || line[0] == ';' || line[0] == '#')  // 检查是否为空行或注释
            {
                continue;  // 如果是, 则跳过当前循环, 处理下一行
            }

            // 处理节(section), 形如 [section_name]
            if (line[0] == '[' && line.back() == ']')  // 检查行是否以 '[' 开头并以 ']' 结尾
            {
                // 提取节名 (去除方括号)
                current_section = line.substr(1, line.size() - 2);
                // 去除节名两端的空白
                this->trim(current_section);
                // 处理完节名后, 继续下一行
                continue;
            }

            // 处理键值对, 形如 key=value
            size_t delimiter_pos = line.find('=');   // 查找等号 '=' 的位置
            if (delimiter_pos != std::string::npos)  // 如果找到了等号
            {
                std::string key = line.substr(0, delimiter_pos);     // 提取键 (等号之前的部分)
                std::string value = line.substr(delimiter_pos + 1);  // 提取值 (等号之后的部分)

                this->trim(key);    // 去除键两端的空白
                this->trim(value);  // 去除值两端的空白

                // 如果值被引号包围, 则移除它们
                if (value.size() >= 2 &&                                // 检查值的长度是否至少为2
                    ((value.front() == '"' && value.back() == '"') ||   // 检查是否被双引号包围
                     (value.front() == '\'' && value.back() == '\'')))  // 检查是否被单引号包围
                {
                    // 提取去除引号后的子字符串
                    value = value.substr(1, value.size() - 2);
                }

                // 将键值对存储到数据结构中, 使用节名进行组织
                // 如果当前节名为空, 则使用空字符串表示全局节
                std::string section = current_section.empty() ? "" : current_section;

                // 对于全局值 (不在任何节下) , 存储等号后的原始行内容
                if (section.empty())
                {
                    // 存储原始值
                    this->data_[section][key] = line.substr(delimiter_pos + 1);
                }
                else  // 对于节内的值
                {
                    // 存储处理过的值
                    this->data_[section][key] = value;
                }
            }
        }
    }

    /***
     * @description: 检查指定的节是否存在
     * @param &section {string} :
     * @return {}
     */
    bool has_section(const std::string& section) const  // const 成员函数
    {
        // 在 data_ 中查找节, 如果找到则返回 true
        return this->data_.find(section) != this->data_.end();
    }

    /***
     * @description: 检查指定节中的键是否存在
     * @param &section {string} :
     * @param &key {string} :
     * @return {}
     */
    bool has_key(const std::string& section, const std::string& key) const
    {
        auto it = this->data_.find(section);              // 查找指定的节
        if (it == this->data_.end())                      // 如果节不存在
            return false;                                 // 返回 false
        return it->second.find(key) != it->second.end();  // 在该节的 map 中查找键, 如果找到则返回 true
    }

    /***
     * @description: 获取指定节和键的字符串值
     * @param &section {string} :
     * @param &key {string} :
     * @param &default_value {string} :
     * @return {}
     */
    std::string get_string(const std::string& section, const std::string& key,
                           const std::string& default_value = "") const  // 提供一个默认值
    {
        // 处理空节名 (全局值)
        std::string search_section = section.empty() ? "" : section;  // 确定要搜索的节名

        auto it = this->data_.find(search_section);  // 查找节
        if (it == this->data_.end())                 // 如果节不存在
            return default_value;                    // 返回默认值

        auto key_it = it->second.find(key);  // 在节中查找键
        if (key_it == it->second.end())      // 如果键不存在
            return default_value;            // 返回默认值

        // 对于全局值, 我们存储了原始行内容, 需要再次处理
        if (search_section.empty())
        {
            std::string value = key_it->second;  // 获取存储的值
            this->trim(value);                   // 去除空白

            // 如果有引号, 则去除
            if (value.size() >= 2 &&
                ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\'')))
            {
                value = value.substr(1, value.size() - 2);  // 提取去除引号后的子字符串
            }
            return value;  // 返回处理后的值
        }

        return key_it->second;  // 对于节内值, 直接返回存储的值
    }

    /***
     * @description: 获取指定节和键的整数值
     * @param &section {string} :
     * @param &key {string} :
     * @param default_value {int} :
     * @return {}
     */
    int get_int(const std::string& section, const std::string& key,
                int default_value = 0) const  // 提供默认值 0
    {
        std::string value = get_string(section, key, "");  // 首先获取字符串形式的值
        if (value.empty())                                 // 如果字符串为空 (即键不存在)
            return default_value;                          // 返回默认值

        try  // 使用 try-catch 块处理转换异常
        {
            return std::stoi(value);  // 尝试将字符串转换为整数
        }
        catch (...)  // 捕获所有类型的异常
        {
            return default_value;  // 如果转换失败, 返回默认值
        }
    }

    /***
     * @description: 获取指定节和键的浮点数值
     * @param &section {string} :
     * @param &key {string} :
     * @param default_value {double} :
     * @return {}
     */
    double get_double(const std::string& section, const std::string& key,
                      double default_value = 0.0) const  // 提供默认值 0.0
    {
        std::string value = get_string(section, key, "");  // 首先获取字符串形式的值
        if (value.empty())                                 // 如果字符串为空
            return default_value;                          // 返回默认值

        try  // 使用 try-catch 块处理转换异常
        {
            return std::stod(value);  // 尝试将字符串转换为 double
        }
        catch (...)  // 捕获所有类型的异常
        {
            return default_value;  // 如果转换失败, 返回默认值
        }
    }

    /***
     * @description: 获取指定节和键的布尔值
     * @param &section {string} :
     * @param &key {string} :
     * @param default_value {bool} :
     * @return {}
     */
    bool get_bool(const std::string& section, const std::string& key,
                  bool default_value = false) const  // 提供默认值 false
    {
        std::string value = get_string(section, key, "");  // 首先获取字符串形式的值
        if (value.empty())                                 // 如果字符串为空
            return default_value;                          // 返回默认值

        // 将获取到的值转换为小写以进行不区分大小写的比较
        std::string lower_value = value;                                             // 创建一个副本
        std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(),  // 使用 transform 算法
                       [](unsigned char c)                                           // lambda 表达式
                       { return std::tolower(c); });                                 // 将每个字符转换为小写

        if (lower_value == "true" || lower_value == "yes" || lower_value == "1")  // 检查表示 true 的字符串
        {
            return true;  // 返回 true
        }
        else if (lower_value == "false" || lower_value == "no" || lower_value == "0")  // 检查表示 false 的字符串
        {
            return false;  // 返回 false
        }
        else  // 如果是其他无法识别的字符串
        {
            return default_value;  // 返回默认值
        }
    }

    /***
     * @description: 获取 INI 文件中所有节的名称
     * @return {}
     */
    std::vector<std::string> get_sections() const  // const 成员函数
    {
        std::vector<std::string> sections;    // 创建一个空的 vector 用于存放节名
        for (const auto& pair : this->data_)  // 遍历 data_ map 中的所有键值对
        {
            if (!pair.first.empty())  // 检查节名是否为空 (跳过全局节)
            {
                sections.push_back(pair.first);  // 将非空节名添加到 vector
            }
        }
        return sections;  // 返回包含所有节名的 vector
    }

    /***
     * @description: 获取指定节中所有键的名称
     * @param &section {string} :
     * @return {}
     */
    std::vector<std::string> get_keys(const std::string& section) const  // const 成员函数
    {
        std::vector<std::string> keys;        // 创建一个空的 vector 用于存放键名
        auto it = this->data_.find(section);  // 查找指定的节
        if (it != this->data_.end())          // 如果找到了节
        {
            for (const auto& pair : it->second)  // 遍历该节的 map 中的所有键值对
            {
                keys.push_back(pair.first);  // 将键名添加到 vector
            }
        }
        return keys;  // 返回包含所有键名的 vector
    }

    /***
     * @description: 获取一维数组 (vector)
     * @param &section {string} :
     * @param &key {string} :
     * @param &default_value {vector<T>} :
     * @return {}
     */
    template <typename T>                                                       // 模板参数 T, 表示数组元素的类型
    std::vector<T> get_array1d(const std::string& section,                      // 节名
                               const std::string& key,                          // 键名
                               const std::vector<T>& default_value = {}) const  // 提供一个空的 vector 作为默认值
    {
        std::string value = get_string(section, key, "");                  // 首先获取字符串形式的值
        if (value.empty() || value.front() != '{' || value.back() != '}')  // 检查值是否为空或不是以花括号包围
            return default_value;                                          // 如果不满足数组格式, 返回默认值

        return this->parse_array<T>(value);  // 调用 parse_array 函数解析字符串并返回结果
    }

    /***
     * @description: 获取二维数组 (vector of vectors)
     * @param &section {string} :
     * @param &key {string} :
     * @param &default_value {vector<std::vector<T>>} :
     * @return {}
     */
    template <typename T>  // 模板参数 T, 表示数组元素的类型
    std::vector<std::vector<T>> get_array2d(
        const std::string& section,                                   // 节名
        const std::string& key,                                       // 键名
        const std::vector<std::vector<T>>& default_value = {}) const  // 提供一个空的二维 vector 作为默认值
    {
        std::string value = get_string(section, key, "");                  // 首先获取字符串形式的值
        if (value.empty() || value.front() != '{' || value.back() != '}')  // 检查值是否为空或不是以花括号包围
            return default_value;                                          // 如果不满足格式, 返回默认值

        std::vector<std::vector<T>> result;             // 创建一个空的二维 vector 用于存放结果
        std::regex inner_array_regex(R"(\{[^{}]+\})");  // 定义正则表达式, 匹配内部的花括号数组, 例如 "{1,2}"
        auto arrays_begin = std::sregex_iterator(value.begin(), value.end(),
                                                 inner_array_regex);  // 创建正则表达式迭代器, 查找所有内部数组
        auto arrays_end = std::sregex_iterator();                     // 创建结束迭代器

        for (std::sregex_iterator i = arrays_begin; i != arrays_end; ++i)  // 遍历所有匹配到的内部数组字符串
        {
            std::smatch match = *i;  // 获取当前匹配
            result.push_back(
                this->parse_array<T>(match.str()));  // 调用 parse_array 解析内部数组字符串, 并将结果添加到二维 vector
        }

        return result;  // 返回解析后的二维 vector
    }

   private:  // 公有成员 (这里重复了 public, 但语法上允许)
    /***
     * @description: 静态成员函数: 去除字符串首尾的空白字符
     * @param &s {string} :
     * @return {}
     */
    static void trim(std::string& s)  // 接受一个字符串的引用, 直接修改原字符串
    {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch)    // 从字符串开头删除空白
                                        { return !std::isspace(ch); }));  // find_if 找到第一个非空白字符
        s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch)             // 从字符串末尾删除空白
                             { return !std::isspace(ch); })               // find_if 从反向迭代器找到第一个非空白字符
                    .base(),                                              // .base() 将反向迭代器转换为正向迭代器
                s.end());                                                 // erase 删除从该位置到末尾的字符
    }

    // 存储 INI 文件数据的核心数据结构
    // 结构是: 节名 -> (键名 -> 值) 的嵌套哈希表
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> data_;

   public:
    /***
     * @description: 设置指定节和键的值
     * @param &section {string} : 节名
     * @param &key {string} : 键名
     * @param &value {string} : 值
     * @return {}
     */
    void set_value(const std::string& section, const std::string& key, const std::string& value)
    {
        std::string search_section = section.empty() ? "" : section;
        this->data_[search_section][key] = value;
    }

    /***
     * @description: 设置指定节和键的整数值
     * @param &section {string} : 节名
     * @param &key {string} : 键名
     * @param value {int} : 整数值
     * @return {}
     */
    void set_int(const std::string& section, const std::string& key, int value)
    {
        this->set_value(section, key, std::to_string(value));
    }

    /***
     * @description: 设置指定节和键的浮点数值
     * @param &section {string} : 节名
     * @param &key {string} : 键名
     * @param value {double} : 浮点数值
     * @return {}
     */
    void set_double(const std::string& section, const std::string& key, double value)
    {
        this->set_value(section, key, std::to_string(value));
    }

    /***
     * @description: 设置指定节和键的布尔值
     * @param &section {string} : 节名
     * @param &key {string} : 键名
     * @param value {bool} : 布尔值
     * @return {}
     */
    void set_bool(const std::string& section, const std::string& key, bool value)
    {
        this->set_value(section, key, value ? "true" : "false");
    }

    /***
     * @description: 设置一维数组
     * @param &section {string} : 节名
     * @param &key {string} : 键名
     * @param &array {vector<T>} : 一维数组
     * @return {}
     */
    template <typename T>
    void set_array1d(const std::string& section, const std::string& key, const std::vector<T>& array)
    {
        std::stringstream ss;
        ss << "{";
        for (size_t i = 0; i < array.size(); ++i)
        {
            if constexpr (std::is_same_v<T, std::string>)
            {
                ss << "'" << array[i] << "'";
            }
            else
            {
                ss << array[i];
            }
            if (i < array.size() - 1)
            {
                ss << ", ";
            }
        }
        ss << "}";
        this->set_value(section, key, ss.str());
    }

    /***
     * @description: 设置二维数组
     * @param &section {string} : 节名
     * @param &key {string} : 键名
     * @param &array {vector<vector<T>>} : 二维数组
     * @return {}
     */
    template <typename T>
    void set_array2d(const std::string& section, const std::string& key, const std::vector<std::vector<T>>& array)
    {
        std::stringstream ss;
        ss << "{";
        for (size_t i = 0; i < array.size(); ++i)
        {
            ss << "{";
            for (size_t j = 0; j < array[i].size(); ++j)
            {
                if constexpr (std::is_same_v<T, std::string>)
                {
                    ss << "'" << array[i][j] << "'";
                }
                else
                {
                    ss << array[i][j];
                }
                if (j < array[i].size() - 1)
                {
                    ss << ", ";
                }
            }
            ss << "}";
            if (i < array.size() - 1)
            {
                ss << ", ";
            }
        }
        ss << "}";
        this->set_value(section, key, ss.str());
    }

    /***
     * @description: 删除指定节
     * @param &section {string} : 节名
     * @return {}
     */
    void remove_section(const std::string& section) { this->data_.erase(section); }

    /***
     * @description: 删除指定节中的键
     * @param &section {string} : 节名
     * @param &key {string} : 键名
     * @return {}
     */
    void remove_key(const std::string& section, const std::string& key)
    {
        auto it = this->data_.find(section);
        if (it != this->data_.end())
        {
            it->second.erase(key);
        }
    }

    /***
     * @description: 保存数据到 INI 文件
     * @param &filename {string} : 文件名
     * @return {}
     */
    void save(const std::string& filename) const
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open file for writing: " + filename);
        }

        // 首先处理全局节 (空节名)
        auto global_it = this->data_.find("");
        if (global_it != this->data_.end())
        {
            for (const auto& key_value : global_it->second)
            {
                file << key_value.first << " = " << key_value.second << std::endl;
            }
            file << std::endl;  // 在全局节后添加空行
        }

        // 然后处理其他节
        for (const auto& section_pair : this->data_)
        {
            const std::string& section = section_pair.first;
            if (section.empty())  // 跳过全局节, 已经处理过了
                continue;

            file << "[" << section << "]" << std::endl;
            for (const auto& key_value : section_pair.second)
            {
                file << key_value.first << " = " << key_value.second << std::endl;
            }
            file << std::endl;  // 在每个节后添加空行
        }
    }

    /***
     * @description: 清空所有数据
     * @return {}
     */
    void clear() { this->data_.clear(); }

    /***
     * @description: 获取所有数据 (用于调试)
     * @return {}
     */
    const std::unordered_map<std::string, std::unordered_map<std::string, std::string>>& get_all_data() const
    {
        return this->data_;
    }
};

#endif  // !__INI_PARSER__H__ // 结束 #ifndef 的条件编译块
