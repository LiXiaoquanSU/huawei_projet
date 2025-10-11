cmake_minimum_required(VERSION 3.10)
project(HuaweiUAVScheduler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件目录
include_directories(include)

# 明确列出所有源文件（手动管理，清晰可靠）
add_executable(uav_scheduler
    src/main.cpp
    src/UAV.cpp
    src/Flow.cpp
    src/Network.cpp
    src/Scheduler.cpp
    src/Utils.cpp
)
