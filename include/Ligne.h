#ifndef LIGNE_H
#define LIGNE_H

#include <vector>
#include <tuple>
#include <utility>
#include <cmath>
#include <algorithm>

/**
 * @brief 单时刻内的一条候选路径（直接作为 A* 的“Item”）
 * - 路径用 (x,y) 座标序列存储：pathXY
 * - 通过 score 作为优先级（priority_queue 将按 score 高者优先）
 * - q 表示瓶颈带宽（或本秒可传量上界），append 时按 min 规则维护
 */
struct Ligne {
    // ======= 基本上下文 =======
    int   flowId{0};
    int   t{0};                  // 当前时刻
    int   t_start{0};            // 流开始时间
    double Q_total{0.0};         // 流总量 (Mbits)
    double Tmax{10.0};           // 最大时延(用于 Delay 评分)

    // ======= 路径与性能 =======
    std::vector<std::pair<int,int>> pathXY;  // (x,y) 座标路径
    double distance{0.0};         // 跳数 = pathXY.size()-1
    double bandwidth{0.0};        // 当前已知瓶颈
    double remainingD{std::numeric_limits<double>::infinity()};       // 剩余待传流量
    double q{0.0};                // 实际用于评分的“已分配速率/可传量上限”
    bool   landed{false};         // 是否已到达落地区域
    double score{0.0};            // 评分（A* 的优先级关键）

    // 声明构造函数（实现见 .cpp）
    Ligne();

    // ========== 评分（作为 A* 的估值/优先级）==========
    // alpha 控制距离项衰减强度
    double computeScore(int currentX, int currentY,
                     int landingX1, int landingY1,
                     int landingX2, int landingY2,
                     double alpha = 0.1);

    // ========== 追加节点 ==========
    // 只做合法性检查与状态更新（不计算分数）。返回 -1 表示非法，返回 (int)score 表示成功（但分数未刷新）
    double addPathUav(int x, int y, double q_u);

    // 追加节点并立刻 computeScore，返回最新分数；非法返回 -1
    double addPathUav(int x, int y, double q_u,
                   int currentX, int currentY,
                   int landingX1, int landingY1,
                   int landingX2, int landingY2,
                   double alpha = 0.1);

    // ======= 导出标准输出 (t, endX, endY, q) =======
    std::tuple<int,int,int,double> exportOutput() const;

    // ======= 供 priority_queue 使用：score 高者优先 =======
    // 默认 priority_queue 取“最大”的元素在顶，因此这里定义为 score < other.score
    bool operator<(const Ligne& other) const {
        return score < other.score;
    }

private:
    // 4 邻接判断
    static inline bool isNeighbor4(int ax, int ay, int bx, int by) {
        int dx = std::abs(ax - bx), dy = std::abs(ay - by);
        return (dx + dy == 1);
    }
    // 路径包含该点？
    static inline bool pathContains(const std::vector<std::pair<int,int>>& path, int x, int y) {
        for (const auto& p : path) if (p.first == x && p.second == y) return true;
        return false;
    }
    // 是否与除最后一个以外的任一点 4 邻接（避免贴边/绕回）
    static inline bool adjacentToAnyExceptLast(const std::vector<std::pair<int,int>>& path,
                                               int x, int y) {
        if (path.size() <= 1) return false;
        for (size_t i = 0; i + 1 < path.size(); ++i)
            if (isNeighbor4(path[i].first, path[i].second, x, y)) return true;
        return false;
    }
};

#endif // LIGNE_H
