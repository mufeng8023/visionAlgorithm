# ============================================================
# 独立编译跟踪器库 - 专为 Python ctypes 调用而设计
# 不修改项目根目录的 CMakeLists.txt
# ============================================================
cmake_minimum_required(VERSION 3.10)
project(tracker_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----- 编译器选项 -----
# 不使用 -Wall -Wextra, 因为 lapjv.hpp 包含 const 限定符错误;
add_compile_options(-O2 -g -fPIC)

# ----- 编译时日志级别 (Info) -----
add_compile_options(-DLOG_COMPILE_LEVEL=2)

# ----- Eigen3 -----
set(EIGEN3_INCLUDE_DIR "/usr/local/Program/eigen-5.0.0/include/eigen3")
include_directories(${EIGEN3_INCLUDE_DIR})

# ----- 项目头文件路径 -----
include_directories(
    ${CMAKE_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/../lib
    ${CMAKE_SOURCE_DIR}/../lib/detector
)

# ----- 宽松编译 (处理 lapjv.hpp 的有符号/无符号类型问题) -----
add_compile_options(-fpermissive)

# ----- OpenCV (YoloObject 等依赖) -----
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ----- 定义编译宏 (版本/时间戳等) -----
string(TIMESTAMP TRACK_BUILD_TIME "%Y%m%d_%H%M%S")
execute_process(
    COMMAND git -C ${CMAKE_SOURCE_DIR}/.. rev-parse HEAD
    OUTPUT_VARIABLE TRACK_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
execute_process(
    COMMAND git -C ${CMAKE_SOURCE_DIR}/.. rev-parse --abbrev-ref HEAD
    OUTPUT_VARIABLE TRACK_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT TRACK_GIT_HASH)
    set(TRACK_GIT_HASH "unknown")
endif()
if(NOT TRACK_GIT_BRANCH)
    set(TRACK_GIT_BRANCH "unknown")
endif()

add_compile_options(
    -DTRACK_BUILD_TIME="${TRACK_BUILD_TIME}"
    -DTRACK_GIT_HASH="${TRACK_GIT_HASH}"
    -DTRACK_GIT_BRANCH="${TRACK_GIT_BRANCH}"
    -DTRACK_PROJECT_NAME="${PROJECT_NAME}"
    -DPROJECT_ROOT="${CMAKE_SOURCE_DIR}/.."
)

# ----- 源文件 (单编译单元避免 logging.hpp 重复定义) -----
set(TRACKER_SRC
    all_api.cpp
)

add_library(tracker SHARED ${TRACKER_SRC})

target_link_libraries(tracker
    ${OpenCV_LIBS}
)

# ----- 输出路径: 默认在 build/lib 下 -----
set_target_properties(tracker PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

message(STATUS "Tracker library build configuration:")
message(STATUS "  Eigen3 include: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "  OpenCV: ${OpenCV_VERSION}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")