#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <cmath>
#include "Network.h"
#include "Flow.h"
#include "UAV.h"
#include "LigneFinder.h"
#include "SlicePlanner.h"
#include "Scheduler.h"
#include "Utils.h"

int runCube(){
    std::cout << "=== PathFinder A* 测试程序 ===" << std::endl;

    const std::string inputDir = "../input";
    const std::string outputDir = "../output";

    auto files = Utils::listInputFiles(inputDir);
    if (files.empty()) {
        std::cerr << "❌ 未找到输入文件！请在 input 文件夹中放测试文件。\n";
        return -1;
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
            return -1;
        }

        scheduler.outputResult(fout);
        std::cout << "✅ Result saved to: " << outputPath << std::endl;

    }
    return 0;
}

using XY = std::pair<int,int>;

void testLigneFinderSingleFlow() {
    std::cout << "\n=== 🔍 LigneFinder 单流测试 ===\n";

    // ===== 构造简单网络 =====
    Network net;
    net.M = 3;
    net.N = 3;
    net.T = 10;

    // UAV 带宽初始化：峰值 10，φ=3（周期相同）
    for (int x = 0; x < net.M; ++x) {
        for (int y = 0; y < net.N; ++y) {
            if(x==1 && y==2)
                net.uavs.push_back(UAV(x * net.N + y, x, y, 9, 3));
            else
                net.uavs.push_back(UAV(x * net.N + y, x, y, 10, 3));
        }
    }

    // 添加一条流：Flow1 从(0,0)出发，目标区域为右下角 (2,2)
    Flow f1(1, 0, 2, 0, 40, 2, 1, 2, 2);
    net.flows.push_back(f1);

    // ===== 构造当前时刻带宽矩阵 =====
    std::map<std::pair<int,int>, double> bw;
    int t = 0;
    for (const auto& uav : net.uavs) {
        bw[{uav.x, uav.y}] = uav.bandwidthAt(t);
    }

    // ===== 创建 LigneFinder 并运行 A* =====
    LigneFinder finder(net, f1, t, bw,{-1,-1},{-1,-1},0,1,40);
    auto candidates = finder.runAStarOnce();

    if (candidates.empty()) {
        std::cout << "❌ 未找到任何候选路径\n";
        return;
    }

    // ===== 输出所有候选路径 =====
    std::cout << "\n=== 候选路径结果 ===\n";
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& L = candidates[i];
        std::cout << "#" << i + 1 << " Flow=" << L.flowId
                  << " Score=" << L.score
                  << " q=" << L.q
                  << " dist=" << L.distance
                  << " Path: ";
        for (auto [x, y] : L.pathXY)
            std::cout << "(" << x << "," << y << ")->";
        std::cout << "\n";
    }
    std::cout << "=== LigneFinder 测试结束 ===\n";
}

void testSlicePlanner(){
    // ✅ 1. 初始化网络参数
    Network net;
    net.M = 5;
    net.N = 5;
    int idCounter = 0;
    net.T = 10;
    // // ✅ 2. 构建 UAV 网格
    // for (int x = 0; x < net.M; ++x) {
    //     for (int y = 0; y < net.N; ++y) {
    //         if(x==0 && y==1)
    //             net.uavs.push_back(UAV(idCounter++, x, y, 10, 3));
    //         else
    //             net.uavs.push_back(UAV(idCounter++, x, y, 10, 3));
    //     }
    // }

net.uavs.push_back(UAV(0, 0, 0, 10, 1));
net.uavs.push_back(UAV(1, 1, 0, 10, 2));
net.uavs.push_back(UAV(2, 2, 0, 10, 3));
net.uavs.push_back(UAV(3, 3, 0, 10, 4));
net.uavs.push_back(UAV(4, 4, 0, 10, 5));
net.uavs.push_back(UAV(5, 0, 1, 10, 2));
net.uavs.push_back(UAV(6, 1, 1, 9, 3));
net.uavs.push_back(UAV(7, 2, 1, 10, 4));
net.uavs.push_back(UAV(8, 3, 1, 9, 5));
net.uavs.push_back(UAV(9, 4, 1, 10, 1));
net.uavs.push_back(UAV(10, 0, 2, 10, 3));
net.uavs.push_back(UAV(11, 1, 2, 8, 4));
net.uavs.push_back(UAV(12, 2, 2, 10, 5));
net.uavs.push_back(UAV(13, 3, 2, 9, 1));
net.uavs.push_back(UAV(14, 4, 2, 10, 2));
net.uavs.push_back(UAV(15, 0, 3, 10, 4));
net.uavs.push_back(UAV(16, 1, 3, 10, 5));
net.uavs.push_back(UAV(17, 2, 3, 9, 1));
net.uavs.push_back(UAV(18, 3, 3, 10, 2));
net.uavs.push_back(UAV(19, 4, 3, 10, 3));
net.uavs.push_back(UAV(20, 0, 4, 10, 5));
net.uavs.push_back(UAV(21, 1, 4, 10, 1));
net.uavs.push_back(UAV(22, 2, 4, 9, 2));
net.uavs.push_back(UAV(23, 3, 4, 10, 3));
net.uavs.push_back(UAV(24, 4, 4, 10, 4));
    // ✅ 3. 构建带宽矩阵
    std::map<XY,double> bw;
    for (const auto& u : net.uavs)
        bw[{u.x, u.y}] = u.bandwidthAt(0);

    // ✅ 4. 打印带宽地图
    std::cout << "[DEBUG] 带宽矩阵 (t=0):\n";
    for (int y = net.N-1; y >= 0; --y) {
        for (int x = 0; x < net.M; ++x)
            std::cout << std::setw(6) << std::fixed << std::setprecision(1) << bw[{x,y}] << " ";
        std::cout << "\n";
    }

    // ✅ 5. 构建多个测试 Flow，设计不同的难度和分数
    Flow flow1, flow2, flow3;
    
    Flow f1(1, 0, 0, 0, 40, 4, 4, 4, 4);
    Flow f2(2, 0, 1, 2, 25, 3, 3, 4, 4);
    Flow f3(3, 2, 4, 1, 30, 0, 0, 2, 2);
    net.flows = {f1, f2, f3};

    // ✅ 7. 测试新的 SlicePlanner
    std::cout << "\n=== SlicePlanner 测试（优化后） ===\n";
    
    // 构建剩余数据映射
    std::map<int, double> remainingData = {{1, 40.0}, {2, 25.0}, {3, 30.0}};
    std::map<int, XY> lastLandingMap = {{1, {-1,-1}}, {2, {-1,-1}}, {3, {-1,-1}}};
    std::map<int, XY> nextLandingMap = {{1, {-1,-1}}, {2, {-1,-1}}, {3, {-1,-1}}};
    std::map<int, int> changeCountMap = {{1, 0}, {2, 0}, {3, 0}};
    std::map<int, int> neighborStateMap = {{1, 0}, {2, 0}, {3, 0}};
    
    int currentTime = 0;
    
    SlicePlanner planner(net, remainingData, lastLandingMap, nextLandingMap, 
                        changeCountMap, neighborStateMap, currentTime, bw);
    
    std::cout << "\n>>> 执行 SlicePlanner::planAllSlices() ...\n";
    auto slices = planner.planAllSlices();
    
    std::cout << "\n✅ SlicePlanner 结果总结：\n";
    std::cout << "  找到 " << slices.size() << " 个可能的 Slice 组合\n";
    
    if (!slices.empty()) {
        std::cout << "\n前几个 Slice 的详细信息：\n";
        for (size_t i = 0; i < std::min((size_t)5, slices.size()); ++i) {
            const auto& slice = slices[i];
            std::cout << "\n  === Slice #" << (i+1) << " (t=" << slice.t << ") ===\n";
            std::cout << "    包含 " << slice.lignes.size() << " 条 Ligne：\n";
            
            double totalScore = 0.0;
            for (const auto& ligne : slice.lignes) {
                auto [ex, ey] = ligne.pathXY.back();
                std::cout << "      Flow#" << ligne.flowId
                          << "  q=" << std::fixed << std::setprecision(3) << ligne.q
                          << "  score=" << std::setw(7) << std::setprecision(3) << ligne.score
                          << "  end=(" << ex << "," << ey << ")"
                          << "  path: ";
                for (const auto& [x, y] : ligne.pathXY)
                    std::cout << "(" << x << "," << y << ")->";
                std::cout << "\n";
                totalScore += ligne.score;
            }
            std::cout << "    *** Slice总分: " << std::fixed << std::setprecision(3) << totalScore << " ***\n";
        }
    }
}
int main() {

//    int test = runCube();

    testSlicePlanner();

//    testLigneFinderSingleFl();
    return 0;
}
