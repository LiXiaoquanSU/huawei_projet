#include "Network.h"
#include "Flow.h"
#include "LigneFinder.h"
#include <iostream>
#include <map>

int main() {
    std::cout << "=== LigneFinder 非首次寻路测试 ===" << std::endl;

    // ---------- 构造网络 ----------
    Network net;
    net.M = 3; net.N = 3; net.T = 10; net.FN = 1;

    // 每个 UAV 的带宽设为 10，相位 3
    for (int y = 0; y < 3; ++y) {
        for (int x = 0; x < 3; ++x) {
            if(x==1 &&y==0)
                net.uavs.emplace_back(y * 3 + x, x, y, 5, 3);
            else
                net.uavs.emplace_back(y * 3 + x, x, y, 10, 3);
        }
    }

    // ---------- 构造流 ----------
    // 从 (0,0) 出发，目标区域 (2,2)
    Flow f(1, 0, 0, 0, 30, 2, 0, 2, 0);
    net.flows.push_back(f);

    // ---------- 构造带宽表 ----------
    std::map<std::pair<int,int>, double> bw;
    for (int y = 0; y < 3; ++y)
        for (int x = 0; x < 3; ++x)
            bw[{x,y}] = (x == 1 && y == 0) ? 5.0 : 10.0;

    // ---------- 模拟上一次落点 ----------
    LigneFinder::XY lastLanding = {-1,-1};   // 上次落点
    int landingChangeCount = 0;             // 已经发生过一次落点变化

    // ---------- 创建 LigneFinder ----------
    double remainingData = 8.0; 
    int current_t = 3;
    LigneFinder finder(net, f, current_t, bw, lastLanding, landingChangeCount,remainingData);

    // ---------- 运行测试 ----------
    auto result = finder.runAStarOnce();

    // ---------- 输出候选结果 ----------
    std::cout << "========== [Test Result] ==========" << std::endl;
    if (result.empty()) {
        std::cout << "❌ 未找到任何路径\n";
    } else {
        for (size_t i = 0; i < result.size(); ++i) {
            const auto& L = result[i];
            std::cout << "#" << i+1
                      << " score=" << L.score
                      << " q=" << L.q
                      << " dist=" << L.distance
                      << " end=(" << L.pathXY.back().first << "," << L.pathXY.back().second << ")"
                      << " path=";
            for (auto& [x,y] : L.pathXY) std::cout << "("<<x<<","<<y<<")->";
            std::cout << "\n";
        }
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> 7abb832 (限制剩余流量)
