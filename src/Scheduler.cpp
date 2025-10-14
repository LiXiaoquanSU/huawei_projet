#include "Scheduler.h"
#include <algorithm>
#include <iomanip>
#include <cmath>
#include "PathFinder.h"

Scheduler::Scheduler(Network& net) : network(net) {}

void Scheduler::run() {
    std::cout << "\n=== PathFinder A* 全时刻测试模式启动 ===\n";

    PathFinder pathFinder;

    for (int t = 0; t < network.T; ++t) {
        // 1️⃣ 筛选当前时刻已开始的 flow
        std::vector<Flow> activeFlows;
        for (const auto& f : network.flows) {
            if (f.startTime <= t)
                activeFlows.push_back(f);
        }

        // 2️⃣ 如果当前时刻没有流，跳过
        if (activeFlows.empty()) {
            std::cout << "\n--- 时刻 t = " << t << " --- 无活动流\n";
            continue;
        }

        // 3️⃣ 调用 PathFinder
        auto lignes = pathFinder.findCandidatePaths(network, activeFlows, t);

        // 4️⃣ 打印输出
        std::cout << "\n--- 时刻 t = " << t << " ---\n";
        std::cout << "活动流数量: " << activeFlows.size()
                  << " | 路径数量: " << lignes.size() << "\n\n";

        for (const auto& l : lignes) {
            std::cout << "Flow #" << l.flowId
                      << " | Score=" << std::fixed << std::setprecision(2) << l.score
                      << " | Bw=" << l.bandwidth
                      << " | UAV Path: ";

            for (size_t i = 0; i < l.pathUavIds.size(); ++i) {
                std::cout << l.pathUavIds[i];
                if (i < l.pathUavIds.size() - 1) std::cout << "->";
            }
            std::cout << "\n";
        }
    }

    std::cout << "\n=== PathFinder 全时刻测试结束 ===\n";
}

void Scheduler::outputResult(std::ostream& out) const {
    for (const auto& [flowId, records] : scheduleRecords) {
        out << flowId << " " << records.size() << "\n";
        for (const auto& record : records) {
            out << record.t << " " << record.x << " " << record.y << " " << record.z << "\n";
        }
    }
}
