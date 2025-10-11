cmake_minimum_required(VERSION 3.10)
project(HuaweiUAVScheduler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件路径
include_directories(include)

# 源文件列表
file(GLOB SOURCES src/*.cpp)

add_executable(uav_scheduler ${SOURCES})
