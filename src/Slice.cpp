#include "Slice.h"
#include <sstream>
#include <iomanip>
#include <map>

Slice::Slice() : t(0), totalScore(0) {}

Slice::Slice(int t, const std::vector<Ligne>& lignes)
    : t(t), lignes(lignes), totalScore(0) {}

/**
 * @brief 计算总得分（所有 Ligne 的得分求和）
 */
double Slice::computeTotalScore() {
    totalScore = 0.0;
    for (auto& l : lignes)
        totalScore += l.score;
    return totalScore;
}

/**
 * @brief 获取某个流在该时刻的总传输量（Σq）
 */
double Slice::getFlowTotalQ(int flowId) const {
    double sum = 0.0;
    for (const auto& l : lignes)
        if (l.flowId == flowId)
            sum += l.q;
    return sum;
}

/**
 * @brief 获取某个流的终点 UAV 坐标
 * @param M 网络宽度（用于索引换算）
 */
std::pair<int, int> Slice::getFlowEndPoint(int flowId, int M) const {
    for (const auto& l : lignes) {
        if (l.flowId == flowId && !l.pathUavIds.empty()) {
            int lastId = l.pathUavIds.back();
            int x = lastId % M;
            int y = lastId / M;
            return {x, y};
        }
    }
    return {-1, -1}; // 未找到
}

/**
 * @brief 导出标准输出表
 * @return vector<tuple<t, x, y, q>>
 */
std::vector<std::tuple<int, int, int, double>> Slice::exportOutputTable(int M) const {
    std::vector<std::tuple<int, int, int, double>> table;
    for (const auto& l : lignes) {
        table.push_back(l.exportOutput(M)); 
    }
    return table;
}

/**
 * @brief 调试输出
 */
std::string Slice::summary() const {
    std::ostringstream oss;
    oss << "--- Slice t=" << t << " ---\n";
    oss << std::setw(8) << "FlowID"
        << std::setw(12) << "Q (Mbps)"
        << std::setw(12) << "Score" << "\n";
    oss << "--------------------------------\n";
    for (const auto& l : lignes) {
        oss << std::setw(8) << l.flowId
            << std::setw(12) << std::fixed << std::setprecision(2) << l.q
            << std::setw(12) << std::fixed << std::setprecision(2) << l.score
            << "\n";
    }
    oss << "--------------------------------\n";
    oss << "Total Score: " << std::fixed << std::setprecision(2)
        << totalScore << "\n";
    return oss.str();
}
