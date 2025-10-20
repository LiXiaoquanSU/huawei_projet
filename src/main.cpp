#include <iostream>
#include <fstream>
#include "Network.h"
#include "Scheduler.h"
#include "Utils.h"
    #include "LigneFinder.h"
    #include <map>
    #include <set>

int main() {
    // std::cout << "=== PathFinder A* 测试程序 ===" << std::endl;

    // const std::string inputDir = "../input";
    // const std::string outputDir = "../output";

    // auto files = Utils::listInputFiles(inputDir);
    // if (files.empty()) {
    //     std::cerr << "❌ 未找到输入文件！请在 input 文件夹中放测试文件。\n";
    //     return 1;
    // }

    // for (const auto& inputPath : files) {
    //     std::cout << "\n📂 测试文件：" << inputPath << std::endl;

    //     Network network;
    //     if (!Utils::loadNetworkFromFile(inputPath, network))
    //         continue;

    //     Scheduler scheduler(network);
    //     scheduler.run(); // 调用测试模式
        
    //     std::string outputPath = Utils::makeOutputPath(inputPath, inputDir, outputDir);
    //     std::ofstream fout(outputPath);
    //     if (!fout.is_open()) {
    //         std::cerr << "❌ Cannot open output file: " << outputPath << std::endl;
    //         return false;
    //     }

    //     scheduler.outputResult(fout);
    //     std::cout << "✅ Result saved to: " << outputPath << std::endl;

    // }

        std::cout << "=== LigneFinder 测试 ===" << std::endl;
    
        // 构造一个简单网络 4x4
        Network net;
        net.M = 4; net.N = 4; net.T = 10; net.FN = 1;
    
        // 构造16个UAV，峰值10，相位3
        for (int y = 0; y < 4; ++y){
            for (int x = 0; x < 4; ++x){
                if(x==1 && y==1)
                    net.uavs.emplace_back(y * 3 + x, x, y, 10, 6);
                else
                    net.uavs.emplace_back(y * 3 + x, x, y, 10, 3);
            }
        }
            
    
        // 构造一个简单流：从 (0,0) 出发，目标区域 (2,2)
        Flow f(1, 0, 0, 0, 200, 3, 2, 3, 3);
        net.flows.push_back(f);
    
        // 构造当前时刻 t=3 的带宽矩阵
        std::map<LigneFinder::XY, double> bw;
        for (auto& u : net.uavs)
            bw[{u.x, u.y}] = u.bandwidthAt(3);
    
        LigneFinder finder(net, f, 3, bw);
        auto results = finder.runAStarOnce();
    
        if (results.empty()) {
            std::cout << "❌ 未找到任何可行路径\n";
        } else {
            std::cout << "✅ 找到 " << results.size() << " 条候选路径\n";
            for (const auto& L : results) {
                std::cout << "Flow " << L.flowId
                          << " Score=" << L.score
                          << " q=" << L.q
                          << " Dist=" << L.distance
                          << " Path:";
                for (auto [x, y] : L.pathXY)
                    std::cout << " (" << x << "," << y << ")";
                std::cout << "\n";
            }
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
