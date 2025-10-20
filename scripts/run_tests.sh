#!/bin/bash

# 运行Google Test单元测试

set -e

echo "运行Google Test单元测试..."

# 获取项目根目录
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# 构建测试
BUILD_DIR="build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)

# 检查是否生成了测试可执行文件
if [[ -f "uav_scheduler_tests" ]]; then
    echo ""
    echo "运行测试..."
    echo "=========================================="
    ./uav_scheduler_tests
    echo "=========================================="
    echo "测试完成！"
elif [[ -f "tests/uav_scheduler_tests" ]]; then
    echo ""
    echo "运行测试..."
    echo "=========================================="
    ./tests/uav_scheduler_tests
    echo "=========================================="
    echo "测试完成！"
else
    echo ""
    echo "警告：未找到测试可执行文件。可能Google Test未正确安装。"
    echo "尝试使用tests目录下的独立构建..."
    
    cd ../tests
    
    if [[ -d "build" ]]; then
        rm -rf "build"
    fi
    
    mkdir -p "build"
    cd "build"
    
    echo "使用独立的测试构建配置..."
    cmake .. -DCMAKE_BUILD_TYPE=Debug
    
    echo "编译测试..."
    make -j$(nproc)
    
    if [[ -f "uav_scheduler_tests" ]]; then
        echo ""
        echo "运行测试..."
        echo "=========================================="
        ./uav_scheduler_tests
        echo "=========================================="
        echo "测试完成！"
    else
        echo "错误：无法构建测试。请检查Google Test是否正确安装。"
        echo ""
        echo "安装Google Test (Fedora):"
        echo "  sudo dnf install gtest-devel gmock-devel cmake gcc-c++"
        echo ""
        echo "安装Google Test (Ubuntu/Debian):"
        echo "  sudo apt-get update"
        echo "  sudo apt-get install libgtest-dev cmake"
        echo ""
        echo "安装Google Test (CentOS/RHEL):"
        echo "  sudo yum install gtest-devel gmock-devel cmake gcc-c++"
        exit 1
    fi
fi

cd - > /dev/null

echo ""
echo "所有操作完成！"