#ifndef LIGNEFINDER_H
#define LIGNEFINDER_H

#include "Network.h"
#include "Ligne.h"
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <utility>

/**
 * @brief LigneFinder
 * 在给定时刻 t，利用 A* 算法在 UAV 网格中搜索从起点到落地区域的最优路径。
 * 可选择 4 邻接 或 8 邻接 搜索模式。
 */
class LigneFinder {
public:
    enum NeighborMode { FOUR_WAY, EIGHT_WAY };

    LigneFinder(const Network& net, int currentT, NeighborMode mode = EIGHT_WAY);

    /**
     * @brief 搜索单个流的最优路径
     * @param flow 目标流对象
     * @return 返回最优路径对应的 Ligne 对象
     */
    Ligne findBestPath(const Flow& flow);

private:
    const Network& network;
    int t;                    ///< 当前时刻
    NeighborMode neighborMode; ///< 邻接模式

    struct Node {
        int x, y;
        double g;  ///< 实际代价
        double h;  ///< 启发函数
        double f;  ///< f = g + h
        int parentX, parentY;
        bool operator<(const Node& other) const { return f > other.f; }
    };

    // ===== 内部函数 =====
    std::vector<std::pair<int,int>> getNeighbors(int x, int y) const;
    double heuristic(int x1, int y1, int x2, int y2) const;
    double bandwidthCost(int x, int y) const;
};

#endif // LIGNEFINDER_H


