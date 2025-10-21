#include <iostream>
#include <iomanip>
#include <map>
#include <cmath>
#include "Network.h"
#include "Flow.h"
#include "UAV.h"
#include "LigneFinder.h"

using XY = std::pair<int,int>;

int main() {
    std::cout << "=== LigneFinder 单独测试 ===\n";

    // ✅ 1. 初始化网络参数
    Network net;
    net.M = 3;
    net.N = 3;
    int idCounter = 0;

    // ✅ 2. 构建 UAV 网格
    for (int x = 0; x < net.M; ++x) {
        for (int y = 0; y < net.N; ++y) {
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

    // ✅ 5. 构建一个测试 Flow
    Flow flow;
    flow.id = 1;
    flow.size = 40.0;
    flow.x = 0;
    flow.y = 0;
    flow.startTime = 0;
    flow.m1 = 1; flow.n1 = 2;   // 落地区域 (3,3) 到 (4,4)
    flow.m2 = 2; flow.n2 = 2;

    // ✅ 6. 构造 LigneFinder
    XY lastLanding = {-1, -1};
    XY nextLanding = {-1, -1};
    int changeCount = 0;
    double remain = flow.size;
    int neighborState = 0;

    LigneFinder finder(net, flow, 0, bw,
                       lastLanding,
                       nextLanding,
                       changeCount,
                       neighborState,
                       remain);
    // ✅ 7. 执行寻路
    std::cout << "\n>>> 执行 LigneFinder::runAStarOnce() ...\n";
    auto lignes = finder.runAStarOnce();

    // ✅ 8. 输出结果
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

    std::cout << "\n=== 测试完成 ===\n";
    return 0;
}
