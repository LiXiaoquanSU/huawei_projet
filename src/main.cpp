#include <iostream>
#include <fstream>
#include "Network.h"
#include "Scheduler.h"
#include "Utils.h"

int main() {
    std::cout << "=== UAV Network Scheduler Simulation ===" << std::endl;

    const std::string inputDir = "../data";
    const std::string outputDir = "../output";

    // 1. 获取输入文件列表
    auto files = Utils::listInputFiles(inputDir);
    if (files.empty()) {
        std::cerr << "No input files found in " << inputDir << std::endl;
        return 1;
    }

    // 2. 对每个文件执行调度
    for (const auto& inputPath : files) {
        std::cout << "\nProcessing: " << inputPath << std::endl;

        Network network;
        if (!Utils::loadNetworkFromFile(inputPath, network))
            continue;

        std::string outputPath = Utils::makeOutputPath(inputPath, inputDir, outputDir);
        Utils::runSchedulerAndSave(network, outputPath);
    }

    std::cout << "\n=== All scheduling tasks completed. ===" << std::endl;
    return 0;
}
