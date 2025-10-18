#include <iostream>
#include <fstream>
#include "Network.h"
#include "Scheduler.h"
#include "Utils.h"

int main() {
    std::cout << "=== PathFinder A* 测试程序 ===" << std::endl;

    const std::string inputDir = "../input";
    const std::string outputDir = "../output";

    auto files = Utils::listInputFiles(inputDir);
    if (files.empty()) {
        std::cerr << "❌ 未找到输入文件！请在 input 文件夹中放测试文件。\n";
        return 1;
    }

    for (const auto& inputPath : files) {
        std::cout << "\n📂 测试文件：" << inputPath << std::endl;

        Network network;
        if (!Utils::loadNetworkFromFile(inputPath, network))
            continue;

        Scheduler scheduler(network);
        scheduler.run(); // 调用测试模式
        
        std::string outputPath = Utils::makeOutputPath(inputPath, inputDir, outputDir);
        std::ofstream fout(outputPath);
        if (!fout.is_open()) {
            std::cerr << "❌ Cannot open output file: " << outputPath << std::endl;
            return false;
        }

        scheduler.outputResult(fout);
        std::cout << "✅ Result saved to: " << outputPath << std::endl;

    }

    return 0;
    // std::cout << "=== UAV Network Scheduler Simulation ===" << std::endl;

    // const std::string inputDir = "../intput";
    // const std::string outputDir = "../output";

    // // 1. 获取输入文件列表
    // auto files = Utils::listInputFiles(inputDir);
    // if (files.empty()) {
    //     std::cerr << "No input files found in " << inputDir << std::endl;
    //     return 1;
    // }

    // // 2. 对每个文件执行调度
    // for (const auto& inputPath : files) {
    //     std::cout << "\nProcessing: " << inputPath << std::endl;

    //     Network network;
    //     if (!Utils::loadNetworkFromFile(inputPath, network))
    //         continue;

    //     std::string outputPath = Utils::makeOutputPath(inputPath, inputDir, outputDir);
    //     Utils::runSchedulerAndSave(network, outputPath);
    // }

    // std::cout << "\n=== All scheduling tasks completed. ===" << std::endl;
    // return 0;
}
