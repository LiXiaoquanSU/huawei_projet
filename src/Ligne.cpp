#include "Ligne.h"
#include <cmath>
#include <algorithm>

/**
 * @brief 计算当前位置到落地区域最近点的曼哈顿距离
 */
static double predictRemainingDistance(int currentX, int currentY,
                                       int m1, int n1, int m2, int n2)
{
    if (currentX >= m1 && currentX <= m2 &&
        currentY >= n1 && currentY <= n2)
        return 0.0;

    double dx = 0.0;
    if (currentX < m1) dx = m1 - currentX;
    else if (currentX > m2) dx = currentX - m2;

    double dy = 0.0;
    if (currentY < n1) dy = n1 - currentY;
    else if (currentY > n2) dy = currentY - n2;

    return std::abs(dx) + std::abs(dy);
}

// ======================== 构造函数 ========================
Ligne::Ligne()
    : flowId(0), t(0), t_start(0),
      bandwidth(0), q(0), Q_total(0),
      distance(0), Tmax(10.0),
      landed(false), score(0.0),
      prevLandingId(-1), landingChangeCount(1),
      LandingPenalty(0.0) {}

// ======================== 得分计算函数 ========================
void Ligne::computeScore(int currentX, int currentY,
                         int landingX1, int landingY1,
                         int landingX2, int landingY2,
                         double alpha, double beta)
{
    if (Q_total <= 0) {
        score = 0.0;
        return;
    }

    // ===== 1️⃣ U2G Traffic Score =====
    double U2G = q / Q_total;

    // ===== 2️⃣ Traffic Delay Score =====
    double delayFactor = std::max(0, t - t_start);
    double Delay = (Tmax / (delayFactor + Tmax)) * (q / Q_total);

    // ===== 3️⃣ Transmission Distance Score =====
    double Dist = 0.0;
    if (landed) {
        Dist = (q / Q_total) * std::pow(2.0, -alpha * distance);
    } else {
        double Dremain = predictRemainingDistance(currentX, currentY,
                                                  landingX1, landingY1,
                                                  landingX2, landingY2);
        double effectiveQ = std::min(Q_total, q + Dremain);
        double effectiveDist = distance + beta * Dremain;
        Dist = (effectiveQ / Q_total) * std::pow(2.0, -alpha * effectiveDist);
    }

    // ===== 4️⃣ 落点变化惩罚 =====
    LandingPenalty = 0.0;
    if (landed && !pathUavIds.empty()) {
        int currentLandingId = pathUavIds.back();

        if (prevLandingId != -1 && currentLandingId != prevLandingId) {
            ++landingChangeCount;
            // 计算惩罚项 Δpenalty(k) = 10 * (1/(k-1) - 1/k)
            LandingPenalty = 10.0 * ((1.0 / (landingChangeCount - 1)) - (1.0 / landingChangeCount));
        }
    }

    // ===== ✅ 综合得分（含落点惩罚） =====
    score = 100.0 * (0.4 * U2G + 0.2 * Delay + 0.3 * Dist) - LandingPenalty;
}

// ======================== 导出输出函数 ========================
std::tuple<int, int, int, double> Ligne::exportOutput(int M) const {
    if (pathUavIds.empty()) {
        return std::make_tuple(t, -1, -1, 0.0);
    }

    int lastId = pathUavIds.back();
    int endX = lastId % M;
    int endY = lastId / M;
    return std::make_tuple(t, endX, endY, q);
}
