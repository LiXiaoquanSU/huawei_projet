#include "Scheduler.h"
#include <iomanip>

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

    // 如果 T 无效
    if (network.T <= 0) {
        std::cerr << "⚠️ 网络未配置有效的时间长度，跳过调度。\n";
        resultCube.reset();
        std::cout << "=== 调度未执行 ===\n";
        return;
    }

    // 构造一个空 Cube（防止报错）
    Cube cube(network.T);
    resultCube = cube;

    std::cout << "⚙️ 暂未启用 DTCubeBuilder，已生成空 Cube。\n";
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

    auto table = resultCube->exportOutput(network.M);
    out << "# t x y q\n";
    for (const auto& [t, x, y, q] : table) {
        out << t << ' '
            << x << ' '
            << y << ' '
            << std::fixed << std::setprecision(2) << q << '\n';
        std::cout << "t=" << t
                  << ", UAV(" << x << "," << y << "), q="
                  << std::fixed << std::setprecision(2) << q << " Mbps\n";
    }

    out.flags(oldFlags);
    out.precision(oldPrecision);
}
