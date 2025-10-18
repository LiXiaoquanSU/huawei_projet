#include "Cube.h"
#include <iostream>
#include <sstream>
#include <iomanip>

Cube::Cube(int T)
    : T(T), totalScore(0.0) {}

/**
 * @brief 添加一个时刻切片
 */
void Cube::addSlice(const Slice& slice) {
    if (slice.t < 0 || slice.t >= T) {
        std::cerr << "⚠️ Slice t=" << slice.t << " 超出时间范围 [0," << T-1 << "]\n";
        return;
    }
    if (slices.size() <= static_cast<size_t>(slice.t))
        slices.resize(slice.t + 1);
    slices[slice.t] = slice;
}

/**
 * @brief 计算 Cube 的总得分（所有 Slice.totalScore 之和）
 */
double Cube::computeTotalScore() {
    totalScore = 0.0;
    for (auto& s : slices)
        totalScore += s.totalScore;
    return totalScore;
}

/**
 * @brief 导出完整的输出表
 */
std::vector<std::tuple<int,int,int,double>> Cube::exportOutput(int M) const {
    std::vector<std::tuple<int,int,int,double>> table;
    for (const auto& s : slices) {
        auto sub = s.exportOutputTable(M);
        table.insert(table.end(), sub.begin(), sub.end());
    }
    return table;
}

/**
 * @brief 打印 Cube 的摘要
 */
std::string Cube::summary() const {
    std::ostringstream oss;
    oss << "\n===== Cube Summary =====\n";
    oss << "Total Time: " << T << " s\n";
    oss << "Total Slices: " << slices.size() << "\n";
    oss << "------------------------\n";

    for (const auto& s : slices) {
        oss << "t=" << std::setw(2) << s.t
            << "  SliceScore=" << std::fixed << std::setprecision(2)
            << s.totalScore << "\n";
    }

    oss << "------------------------\n";
    oss << "Cube Total Score: " << std::fixed << std::setprecision(2)
        << totalScore << "\n";
    return oss.str();
}
