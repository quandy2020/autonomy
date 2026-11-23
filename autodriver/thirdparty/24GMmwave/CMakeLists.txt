cmake_minimum_required(VERSION 3.10)

# 项目名称
project(MmwaveController)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

find_package(Boost REQUIRED COMPONENTS system)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

# 源文件列表
set(SOURCES
    src/CmdPacket.cpp
    src/CmdPacketTransmitter.cpp
    src/Mmwave.cpp
    src/SerialPort.cpp
    src/SerialStreamProcessor.cpp
    src/RangeDataProcessor.cpp
    src/RangeData.cpp
    src/Utils.cpp
    src/Chirp.cpp
)

add_library(mmwave STATIC ${SOURCES})
target_include_directories(mmwave PUBLIC include)
target_link_libraries(mmwave PUBLIC ${Boost_LIBRARIES})

# 查找并链接线程库
find_package(Threads REQUIRED)
target_link_libraries(mmwave PUBLIC Threads::Threads)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE mmwave)
# 根据不同平台链接额外的库
if(WIN32)
    # Windows平台可能需要链接额外的库
    target_link_libraries(${PROJECT_NAME} PRIVATE setupapi)
elseif(UNIX AND NOT APPLE)
    # Linux平台
    target_link_libraries(${PROJECT_NAME} PRIVATE pthread)
endif()

# 安装规则
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# 打印配置信息
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Sources: ${SOURCES}")
message(STATUS "Headers: ${HEADERS}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
