#include "Cube.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <map>
#include <numeric>
#include <cmath>

Cube::Cube(int T)
    : T(T), totalScore(0.0) {}

/**
 * @brief 添加一个时刻切片
 */
void Cube::addSlice(const Slice& slice) {
    if (slice.t < 0 || slice.t >= T) {
        std::cerr << "Slice t=" << slice.t << " 超出时间范围 [0," << T-1 << "]\n";
        return;
    }
    if (slices.size() <= static_cast<size_t>(slice.t))
        slices.resize(slice.t + 1);
    slices[slice.t] = slice;
}


std::string Cube::summary() const {
    std::ostringstream oss;
    oss << "Scoring Calculation\n";

    // === 汇总每个 flow 的 Ligne ===
    std::map<int, std::vector<Ligne>> flowMap;
    for (const auto& s : slices) {
        for (const auto& L : s.lignes) {
            flowMap[L.flowId].push_back(L);
        }
    }

    double totalWeighted = 0.0;
    double totalSize = 0.0;

    // === 按 flow 输出 ===
    for (const auto& [fid, lignes] : flowMap) {
        if (lignes.empty()) continue;

        const double Q_total = lignes.front().Q_total;
        totalSize += Q_total;

        double U2G_Score = 1.0;  // 所有流已全部传输

        // 平均时延得分估计（参考老师例）
        double delaySum = 0.0;
        for (const auto& L : lignes) {
            int delayFactor = std::max(0, L.t - L.t_start);
            delaySum += (L.Tmax / (delayFactor + L.Tmax)) * (L.q / Q_total);
        }
        double TrafficDelayScore = delaySum;

        // 距离得分
        double DistScore = 0.0;
        for (const auto& L : lignes) {
            DistScore += (L.q / Q_total) * std::pow(2.0, -0.1 * L.distance);
        }

        // 落点变化数 k：若所有 Ligne 落点相同则 k=1，否则按落点变化+1
        int k = 1;
        std::pair<int,int> lastEnd = lignes.front().pathXY.back();
        for (const auto& L : lignes) {
            auto end = L.pathXY.back();
            if (end != lastEnd) {
                k++;
                lastEnd = end;
            }
        }
        double U2GPointScore = 1.0 / k;

        // 计算总分
        double totalScore = 100.0 * (0.4 * U2G_Score +
                                     0.2 * TrafficDelayScore +
                                     0.3 * DistScore +
                                     0.1 * U2GPointScore);

        // 打印细节
        oss << "\nFlow " << fid << ":\n";
        oss << "• Total U2G Traffic Score = "
            << std::fixed << std::setprecision(1)
            << Q_total << "/" << Q_total << " = 1.0\n\n";

        oss << "• Traffic Delay Score = ";
        for (const auto& L : lignes) {
            int d = std::max(0, L.t - L.t_start);
            oss << L.q << "/" << Q_total
                << "*1/" << d << "+10 ";
        }
        oss << "= " << std::fixed << std::setprecision(4)
            << TrafficDelayScore << "\n\n";

        oss << "• Transmission Distance Score = ";
        oss << std::fixed << std::setprecision(4) << DistScore << "\n\n";

        oss << "• U2G Point Score: k=" << k
            << " => " << std::fixed << std::setprecision(1)
            << U2GPointScore << "\n\n";

        oss << "• Total Score = 100(0.4*" << U2G_Score
            << " + 0.2*" << TrafficDelayScore
            << " + 0.3*" << DistScore
            << " + 0.1*" << U2GPointScore
            << ") = " << std::fixed << std::setprecision(3)
            << totalScore << "\n";

        totalWeighted += Q_total * totalScore;
    }

    if (totalSize > 0.0) {
        double overall = totalWeighted / totalSize;
        oss << "\nTotal score: "
            << std::fixed << std::setprecision(3)
            << overall << "\n";
    }

    return oss.str();
}
