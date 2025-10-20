# 测试专用的CMakeLists.txt
# 如果系统没有安装Google Test，可以使用这个版本

cmake_minimum_required(VERSION 3.10)
project(HuaweiUAVScheduler_Tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件路径
include_directories(../include)

# 获取库源文件（排除main.cpp）
file(GLOB LIB_SOURCES ../src/*.cpp)
list(REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../src/main.cpp)

# 创建静态库
add_library(uav_scheduler_lib STATIC ${LIB_SOURCES})
target_include_directories(uav_scheduler_lib PUBLIC ../include)

# 尝试使用FetchContent下载Google Test
include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)

# 设置Google Test选项
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(BUILD_GTEST ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

# 启用测试
enable_testing()

# 测试文件
file(GLOB TEST_SOURCES *.cpp)

# 创建测试可执行文件
add_executable(uav_scheduler_tests ${TEST_SOURCES})

# 链接库
target_link_libraries(uav_scheduler_tests
    uav_scheduler_lib
    gtest
    gtest_main
)

target_include_directories(uav_scheduler_tests PRIVATE ../include)

# 添加测试
include(GoogleTest)
gtest_discover_tests(uav_scheduler_tests)

# 提供便捷的测试目标
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --verbose
    DEPENDS uav_scheduler_tests
    COMMENT "Running all tests"
)