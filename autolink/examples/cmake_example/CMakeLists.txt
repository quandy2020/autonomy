# CMake 示例项目 - 使用 Autolink

cmake_minimum_required(VERSION 3.10)
project(AutolinkCMakeExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加 FindAutolink.cmake 路径
# 假设 FindAutolink.cmake 在项目根目录的 cmake/ 目录下
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")

# 查找 Autolink
find_package(Autolink REQUIRED)

# 创建可执行文件
add_executable(autolink_cmake_example
    main.cpp
)

# 链接 Autolink
target_link_libraries(autolink_cmake_example PRIVATE Autolink::Autolink)

# 打印找到的信息
if(Autolink_FOUND)
    message(STATUS "Found Autolink:")
    message(STATUS "  Version: ${Autolink_VERSION}")
    message(STATUS "  Include dir: ${Autolink_INCLUDE_DIRS}")
    message(STATUS "  Library: ${Autolink_LIBRARIES}")
endif()

