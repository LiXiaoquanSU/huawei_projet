#include <iostream>
#include <iomanip>
#include <map>
#include <cmath>
#include "Network.h"
#include "Flow.h"
#include "UAV.h"
#include "LigneFinder.h"
#include "SlicePlanner.h"

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

int main() {
    std::cout << "=== LigneFinder 单独测试 ===\n";

    // ✅ 1. 初始化网络参数
    Network net;
    net.M = 5;
    net.N = 5;
    int idCounter = 0;

    // ✅ 2. 构建 UAV 网格
    for (int x = 0; x < net.M; ++x) {
        for (int y = 0; y < net.N; ++y) {
            if(x==0 && y==1)
                net.uavs.push_back(UAV(idCounter++, x, y, 10, 3));
            else
                net.uavs.push_back(UAV(idCounter++, x, y, 10, 3));
        }
    }

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
    
    // Flow1: 近距离，应该得高分
    flow1.id = 1;
    flow1.size = 20.0;  // 较小数据量
    flow1.x = 1; flow1.y = 1;  // 中心位置
    flow1.startTime = 0;
    flow1.m1 = 1; flow1.n1 = 3;  // 近距离落地区域
    flow1.m2 = 1; flow1.n2 = 3;
    
    // Flow2: 中距离，中等分数
    flow2.id = 2;
    flow2.size = 30.0;
    flow2.x = 0; flow2.y = 0;  // 角落位置
    flow2.startTime = 0;
    flow2.m1 = 0; flow2.n1 = 3;  // 中等距离落地区域
    flow2.m2 = 2; flow2.n2 = 3;
    
    // Flow3: 远距离，应该得低分
    flow3.id = 3;
    flow3.size = 40.0;  // 较大数据量
    flow3.x = 4; flow3.y = 2;  // 角落位置
    flow3.startTime = 0;
    flow3.m1 = 0; flow3.n1 = 2;  // 远距离落地区域（对角）
    flow3.m2 = 0; flow3.n2 = 2;

    net.flows = {flow1, flow2, flow3};

    // ✅ 6. 测试单个 LigneFinder
    std::cout << "\n=== 单个 LigneFinder 测试 ===\n";
    XY lastLanding = {-1, -1};
    XY nextLanding = {-1, -1};
    int changeCount = 0;
    double remain = flow1.size;
    int neighborState = 1;

    LigneFinder finder(net, flow1, 0, bw,
                       lastLanding,
                       nextLanding,
                       changeCount,
                       neighborState,
                       remain);

    std::cout << "\n>>> 执行 LigneFinder::runAStarOnce() ...\n";
    auto lignes = finder.runAStarOnce();

    if (lignes.empty()) {
        std::cout << "❌ 未找到任何路径！请检查落地区域或带宽设置。\n";
    } else {
        std::cout << "✅ 找到 " << lignes.size() << " 条可行路径：\n";
        int idx = 1;
        for (const auto& L : lignes) {
            auto [ex, ey] = L.pathXY.back();
            std::cout << "  #" << idx++
                      << "  q=" << std::setw(6) << std::setprecision(3) << L.q
                      << "  score=" << std::setw(7) << std::setprecision(3) << L.score
                      << "  end=(" << ex << "," << ey << ")\n    path: ";
            for (auto& [x, y] : L.pathXY)
                std::cout << "(" << x << "," << y << ")->";
            std::cout << "\n";
        }
    }

    // ✅ 7. 测试新的 SlicePlanner
    std::cout << "\n=== SlicePlanner 测试（优化后） ===\n";
    
    // 构建剩余数据映射
    std::map<int, double> remainingData = {{1, 20.0}, {2, 30.0}, {3, 40.0}};
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

    // // ============ 验证排序逻辑 ============
    // std::cout << "\n=== 验证流排序逻辑 ===\n";
    // std::cout << "预期：Flow1(近距离)应该分数最高，Flow3(远距离)分数最低\n";
    
    // // 手动检查每个流的候选分数
    // for (auto flowId : {1, 2, 3}) {
    //     const Flow* flowPtr = nullptr;
    //     for (const auto& f : net.flows) {
    //         if (f.id == flowId) { flowPtr = &f; break; }
    //     }
    //     if (!flowPtr) continue;
        
    //     double remain = remainingData[flowId];
    //     LigneFinder finder(net, *flowPtr, currentTime, bw,
    //                       {-1,-1}, {-1,-1}, 0, 0, remain);
    //     auto lignes = finder.runAStarOnce();
        
    //     double avgScore = 0.0;
    //     if (!lignes.empty()) {
    //         double totalScore = 0.0;
    //         for (const auto& ligne : lignes) {
    //             totalScore += ligne.score;
    //         }
    //         avgScore = totalScore / lignes.size();
    //     }
        
    //     std::cout << "Flow#" << flowId 
    //               << " - 起点:(" << flowPtr->x << "," << flowPtr->y << ")"
    //               << " 落地区域:[(" << flowPtr->m1 << "," << flowPtr->n1 << ")-(" 
    //               << flowPtr->m2 << "," << flowPtr->n2 << ")]"
    //               << " 数据量:" << flowPtr->size
    //               << " → 候选数:" << lignes.size()
    //               << " 平均分:" << std::fixed << std::setprecision(3) << avgScore << "\n";
    // }

    // std::cout << "\n=== 测试完成 ===\n";
    // //testLigneFinderSingleFlow();

    return 0;
}
