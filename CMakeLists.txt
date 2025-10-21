cmake_minimum_required(VERSION 3.10)
project(HuaweiUAVScheduler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件路径
include_directories(include)

# 主程序源文件列表（排除测试相关文件）
file(GLOB MAIN_SOURCES src/*.cpp)

# 创建主程序可执行文件
add_executable(uav_scheduler ${MAIN_SOURCES})

# ================ Google Test 配置 ================

# 查找Google Test
find_package(GTest QUIET)

if(GTest_FOUND)
    # 如果系统中已安装Google Test，直接使用
    message(STATUS "Found Google Test: ${GTest_VERSION}")
    enable_testing()
    
    # 创建测试库（不包含main.cpp）
    file(GLOB LIB_SOURCES src/*.cpp)
    list(REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
    
    if(LIB_SOURCES)
        add_library(uav_scheduler_lib STATIC ${LIB_SOURCES})
        target_include_directories(uav_scheduler_lib PUBLIC include)
    endif()
    
    # 测试文件
    file(GLOB TEST_SOURCES tests/*.cpp)
    
    if(TEST_SOURCES AND LIB_SOURCES)
        # 创建测试可执行文件
        add_executable(uav_scheduler_tests ${TEST_SOURCES})
        target_link_libraries(uav_scheduler_tests 
            uav_scheduler_lib 
            GTest::gtest 
            GTest::gtest_main
        )
        target_include_directories(uav_scheduler_tests PRIVATE include)
        
        # 添加测试到CTest
        include(GoogleTest)
        gtest_discover_tests(uav_scheduler_tests)
        
        message(STATUS "Google Test configuration completed")
        message(STATUS "Run tests with: make test or ctest")
    else()
        message(WARNING "No test sources or library sources found")
    endif()
    
else()
    message(STATUS "Google Test not found. Install Google Test to enable unit tests.")
    message(STATUS "On Ubuntu/Debian: sudo apt-get install libgtest-dev")
    message(STATUS "Then build and install Google Test manually or use package manager")
endif()

# ================ 构建信息 ================
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Main executable: uav_scheduler")
if(GTest_FOUND AND TEST_SOURCES AND LIB_SOURCES)
    message(STATUS "Test executable: uav_scheduler_tests")
endif()
