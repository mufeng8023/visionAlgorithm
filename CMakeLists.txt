# 指定cmake最低版本
cmake_minimum_required(VERSION 3.10)

# 项目名称:project(MAIN)
project(yolo)

# 查找OpenCV库:find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED)

# 有些变量可能和add_compile_options中的重复,要注意
# CMAKE_C_FLAGS:gcc编译选项
# CMAKE_CXX_FLAGS:g++编译选项;"CMAKE_CXX_FLAGS -std=c++17":设置编译标准为c++17
# CMAKE_BUILD_TYPE:编译类型;(Debug:调试;Release:发布)
# CMAKE_C_COMPILER:指定c编译器
# CMAKE_CXX_COMPILER:指定c++编译器
# EXECUTABLE_OUTPUT_PATH:可执行文件输出存放路径
# LIBRARY_OUTPUT_PATH:库文件输出存放的路径
# set(CMAKE_CXX_FLAGS "CMAKE_CXX_FLAGS -std=c++17")
# set(CMAKE_BUILD_TYPE Debug or Release)
set(CMAKE_CXX_COMPILER /usr/bin/g++-11)

# 添加编译选项:add_compile_options(-Wall -g)
# -g:输出带调试信息的可执行文件
# -O[n]:优化代码, 一般为-O2
# -Wall:输出警告信息
# -std=c++17:设置编译标准
# -D:定义宏
# add_compile_options(-Wall -g)
add_compile_options(-Wall -g -std=c++17 -O2)

# 添加头文件搜索路径:include_directories(./include)
# include_directories(include_PATH)
include_directories(${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/lib
    /usr/include/
)

# 添加多个特定的库文件搜索路径:link_directories(LIB_PATH1 LIB_PATH2 ...)
# link_directories(LIB_PATH)
link_directories(${OPENCV_ROOT}/lib)

# find_package(OpenVINO COMPONENTS Runtime)

# 相当前工程添加存放源文件的子目录, 并可以指定中间二进制和目标二进制存放的位置:add_subdirectory(source_dir [binary_dir])
# add_subdirectory(source_dir)

# 生成库文件:add_library(libname MODE source1 source2 ...)
# add_library(libname MODE source1 source2 ...)
# libname:生成库的名称
# MODE:库的模式:{SHARED:动态库; STATIC:静态库}
# source:源文件,可以使用set设置的变量代替
# add_library(libname MODE source)

# 设置变量名称:set(VALUE_NAME ./src/test01.cpp ./src/test02.cpp);${}:获取变量的值
# 就是使用一个变量名称代替多个cpp文件,方便调用
# set(VALUE_NAME ./src/test01.cpp)

# 发现一个目录下所有的源代码文件放置到一个列表中, 并将该列表存储到一个变量中:aux_source_directory(PATH VALUE_NAME)
# PATH:源文件的路径
# VALUE_NAME:生成的变量名
# 如果main函数的源文件也在该路经中保存,那么直接可用add_executable(target VALUE_NAME)生成可执行的目标文件
# aux_source_directory(PATH VALUE_NAME)

# 编译可执行文件:add_executable(target main.cpp VALUE_NAME);${}:获取变量的值
# 遍历获取src中所有cpp文件:file(GLOB My_Source_Files src/*.cpp)
# file(GLOB My_Source_Files src/*.cpp)
# add_executable(yolo main.cpp VALUE_NAME)
file(GLOB CPP_Files src/*.cpp lib/detector/src/*.cpp)
add_executable(${PROJECT_NAME}
    ${CPP_Files}
)

# 为可执行文件链接共享库:target_link_libraries(target library1 library2 ...)
# target_link_libraries(yolo library1)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
