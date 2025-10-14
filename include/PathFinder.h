#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "Network.h"
#include "Flow.h"
#include "Ligne.h"
#include <vector>
#include <queue>
#include <map>

class PathFinder {
public:
    PathFinder();

    // 主入口：在时刻 t 为所有流寻找可行路径
    std::vector<Ligne> findCandidatePaths(const Network& network,
                                          const std::vector<Flow>& flows,
                                          int t);

private:
    std::vector<std::vector<double>> bandwidthMatrix; // 当前时刻带宽矩阵

    void buildBandwidthMatrix(const Network& network, int t);

    std::vector<Ligne> findPathsForFlow(const Network& network,
                                        const Flow& flow,
                                        int t,
                                        int maxPaths);

    std::vector<int> getNeighborIds(const Network& network, int uavId) const;

    double computePathScore(double bandwidth, int hopCount, bool lands) const;

    int computePathBudget(const Network& network) const; // 动态调整K值
};

#endif // PATHFINDER_H
