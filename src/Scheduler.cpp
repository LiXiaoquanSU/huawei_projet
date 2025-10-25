#include "Scheduler.h"
#include "DTCube.h"
#include "CubeOptimizer.h"
#include <algorithm>
#include <iomanip>
#include <map>
#include <sstream>
#include <cmath>
#include <tuple>
#include <vector>

Scheduler::Scheduler(Network& net)
    : network(net) {}

/**
 * @brief 主调度入口（当前仅做空实现）
 */
void Scheduler::run() {
    std::cout << "\n=== PathFinder 调度启动 ===\n";
    std::cout << "网络尺寸: " << network.M << " x " << network.N
              << "，流数量: " << network.FN
              << "，时长 T=" << network.T << "\n";

    if (network.T <= 0) {
        std::cerr << "⚠️ 网络未配置有效的时间长度，跳过调度。\n";
        resultCube.reset();
        std::cout << "=== 调度未执行 ===\n";
        return;
    }

    // Step 1: 构建基础 DTCube
    DTCubeBuilder builder(network);
    Cube best = builder.build();
    resultCube = std::move(best);

    // Step 2: 优化 Cube
    CubeOptimizer optimizer(network, *resultCube);
    Cube optimized = optimizer.optimize();

    // ✅ 用优化结果覆盖原始 Cube
    resultCube = std::move(optimized);

    // Step 3: 打印最终统计
    std::cout << "\n================= 📊 Scoring Summary =================\n";
    std::cout << resultCube->summary() << std::endl;
    std::cout << "=====================================================\n";

    std::cout << "=== 调度完成 ===\n";
}

/**
 * @brief 将调度结果输出为标准表格格式
 */
void Scheduler::outputResult(std::ostream& out) const {
    if (!out) {
        std::cerr << "❌ 输出流无效，无法写入结果\n";
        return;
    }

    if (!resultCube) {
        std::cerr << "❌ 尚未执行调度，缺少可输出的 Cube\n";
        return;
    }

    auto oldFlags = out.flags();
    auto oldPrecision = out.precision();

    std::map<int, std::vector<std::tuple<int, int, int, double>>> flowRecords;
    for (const auto& slice : resultCube->slices) {
        for (const auto& ligne : slice.lignes) {
            if (ligne.pathXY.empty()) {
                continue;
            }
            auto [endX, endY] = ligne.pathXY.back();
            flowRecords[ligne.flowId].emplace_back(slice.t, endX, endY, ligne.q);
        }
    }

    for (auto& [flowId, records] : flowRecords) {
        std::sort(records.begin(), records.end(),
                  [](const auto& a, const auto& b) {
                      return std::get<0>(a) < std::get<0>(b);
                  });
    }

    static const std::vector<std::tuple<int, int, int, double>> emptyRecords;

    auto formatQ = [](double value) {
        std::ostringstream oss;
        const double rounded = std::round(value * 10.0) / 10.0;
        const bool isInteger = std::abs(rounded - std::round(rounded)) < 1e-6;
        oss << std::fixed << std::setprecision(isInteger ? 0 : 1) << rounded;
        return oss.str();
    };

    for (const auto& flow : network.flows) {
        const auto it = flowRecords.find(flow.id);
        const auto& records = (it != flowRecords.end()) ? it->second : emptyRecords;

        out << flow.id << ' ' << records.size() << '\n';
        std::cout << "Flow " << flow.id << " records: " << records.size() << '\n';

        for (const auto& [t, x, y, q] : records) {
            const auto formattedQ = formatQ(q);
            out << t << ' '
                << x << ' '
                << y << ' '
                << formattedQ << '\n';
            std::cout << "  t=" << t << ", UAV(" << x << "," << y << "), q="
                      << formattedQ << " Mbps\n";
        }
    }
    out.flags(oldFlags);
    out.precision(oldPrecision);
}
