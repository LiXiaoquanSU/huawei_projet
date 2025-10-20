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
      landed(false), score(0.0) {}

// ======================== 得分计算函数 ========================
int Ligne::computeScore(int currentX, int currentY,
                         int landingX1, int landingY1,
                         int landingX2, int landingY2,
                         double alpha)
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
        double effectiveDist = distance + Dremain;
        Dist = (effectiveQ / Q_total) * std::pow(2.0, -alpha * effectiveDist);
    }

    // ===== ✅ 综合得分（外部传入落点惩罚） =====
    score = 100.0 * (0.4 * U2G + 0.2 * Delay + 0.3 * Dist);
    return score;
}

double addPathUav(int x, int y, double q_u,
    int landingX1, int landingY1,
    int landingX2, int landingY2,
    double alpha = 0.1)
{
    // 1) 新点带宽为 0 → 失败
    if (q_u <= 0.0) return -1;

    if (pathXY.empty()) {
    // 2) 不允许重复出现
    if (pathContains(pathXY, x, y)) return -1;

    // 3) 必须与“最后一个节点”4邻接（如果已有路径）
    if (!pathXY.empty()) {
        auto [lx, ly] = pathXY.back();
        if (!isNeighbor4(lx, ly, x, y)) return -1;
    }

    // 4) 不允许与“除最后一个节点外”的任意旧节点 4 邻接（避免贴边/绕回）
    if (adjacentToAnyExceptLast(pathXY, x, y)) return -1;

    // 5) 通过校验：插入节点，更新距离（= pathXY.size() - 1）
    pathXY.emplace_back(x, y);
    distance = std::max(0, (int)pathXY.size() - 1);

    // 6) 更新 q（瓶颈），以新增点的带宽 q_u 约束
    if (q <= 0.0) q = q_u;
    else if (q_u < q) q = q_u;

    // 可选：保持 bandwidth 与 q 同步为瓶颈
    if (bandwidth <= 0.0) bandwidth = q;
    else bandwidth = std::min(bandwidth, q);

    // 是否落地（仅做 landed 标记，具体得分仍由 computeScore 处理）
    if (!pathXY.empty()) {
    auto [ex, ey] = pathXY.back();
    landed = (ex >= landingX1 && ex <= landingX2 &&
    ey >= landingY1 && ey <= landingY2);
    }

    // 立刻刷新分数并返回
    return computeScore(currentX, currentY,
        landingX1, landingY1, landingX2, landingY2,
        alpha, beta);
}


// ======================== 导出输出函数 ========================
std::tuple<int, int, int, double> Ligne::exportOutput(int M) const {
    if (pathXY.empty()) {
        return std::make_tuple(t, -1, -1, 0.0);
    }
    auto [endX, endY] = pathXY.back();
    return std::make_tuple(t, endX, endY, q);
}
// —— 工具：4邻接判断
static inline bool isNeighbor4(int ax, int ay, int bx, int by) {
    int dx = std::abs(ax - bx);
    int dy = std::abs(ay - by);
    return (dx + dy == 1); // 上下左右
}
// —— 工具：路径中是否已包含该点
static inline bool pathContains(const std::vector<std::pair<int,int>>& path, int x, int y) {
    for (const auto& p : path) if (p.first == x && p.second == y) return true;
    return false;
}
// —— 工具：新点是否与除最后一个节点外的任何旧节点 4 邻接（避免贴边/绕回）
static inline bool adjacentToAnyExceptLast(const std::vector<std::pair<int,int>>& path,
    int x, int y)
{
if (path.size() <= 1) return true;
// 最后一个节点允许相邻，其余不允许
for (size_t i = 0; i + 1 < path.size(); ++i) {
if (isNeighbor4(path[i].first, path[i].second, x, y)) return true;
}
return false;
}
