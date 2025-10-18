#include "Scheduler.h"

#include <iomanip>

// 构造函数：保存网络引用并初始化决策树构建器
Scheduler::Scheduler(Network& net)
    : network(net), treeBuilder(net) {}

// 主调度入口：调用决策树构建器生成全局最优 Cube
void Scheduler::run() {
    std::cout << "\n=== PathFinder A* 决策树调度启动 ===\n";
    std::cout << "网络尺寸: " << network.M << " x " << network.N
              << "，流数量: " << network.FN
              << "，时长 T=" << network.T << "\n";

    // 基础参数校验
    if (network.T <= 0) {
        std::cerr << "⚠️ 网络未配置有效的时间长度，跳过调度。\n";
        resultCube.reset();
        std::cout << "=== 调度未执行 ===\n";
        return;
    }

    resultCube = treeBuilder.build();

    // 理论上 resultCube 一定有值，但这里仍做防御式检查
    if (!resultCube) {
        std::cerr << "❌ 决策树构建失败，未生成 Cube。\n";
        std::cout << "=== 调度失败 ===\n";
        return;
    }

    std::cout << "生成的 Slice 数量: " << resultCube->slices.size() << "\n";
    std::cout << "Cube 总得分: " << std::fixed << std::setprecision(2)
              << resultCube->totalScore << "\n";
    std::cout << "=== 调度完成 ===\n";
}

// 将调度结果输出为标准表格格式
void Scheduler::outputResult(std::ostream& out) const {
    if (!out) {
        std::cerr << "❌ 输出流无效，无法写入结果\n";
        return;
    }

    // 确保先执行 run()
    if (!resultCube) {
        std::cerr << "❌ 尚未执行调度，缺少可输出的 Cube\n";
        return;
    }

    auto oldFlags = out.flags();
    auto oldPrecision = out.precision();

    auto table = resultCube->exportOutput(network.M);
    out << "# t x y q\n";
    for (const auto& [t, x, y, q] : table) {
        out << t << ' '
            << x << ' '
            << y << ' '
            << std::fixed << std::setprecision(2) << q << '\n';
        std::cout << "t=" << t << ", UAV(" << x << "," << y << "), q=" << std::fixed << std::setprecision(2) << q << " Mbps\n";
    }

    out.flags(oldFlags);
    out.precision(oldPrecision);
}
