#include "PathFinder.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <iostream>

PathFinder::PathFinder() {}

/**
 * 在时刻 t 为所有流寻找可行路径
 */
std::vector<Ligne> PathFinder::findCandidatePaths(const Network& network,
                                                  const std::vector<Flow>& flows,
                                                  int t) {
    std::vector<Ligne> allLignes;

    // 1️⃣ 构建当前时刻的带宽矩阵
    buildBandwidthMatrix(network, t);

    // 2️⃣ 根据网络规模和流数计算要保留的路径数量（动态K）
    int maxPaths = computePathBudget(network);

    // 3️⃣ 为每个flow寻找候选路径
    for (const auto& f : flows) {
        auto lignes = findPathsForFlow(network, f, t, maxPaths);
        allLignes.insert(allLignes.end(), lignes.begin(), lignes.end());
    }

    return allLignes;
}

/**
 * 构建当前时刻的带宽矩阵 bandwidthMatrix[y][x]
 */
void PathFinder::buildBandwidthMatrix(const Network& network, int t) {
    bandwidthMatrix.assign(network.N, std::vector<double>(network.M, 0.0));
    for (const auto& uav : network.uavs) {
        bandwidthMatrix[uav.y][uav.x] = uav.bandwidthAt(t);
    }
}

std::vector<Ligne> PathFinder::findPathsForFlow(const Network& network,
                                                const Flow& flow,
                                                int t,
                                                int maxPaths)
{
    std::vector<Ligne> results;
    int startId = flow.y * network.M + flow.x;
    if (bandwidthMatrix[flow.y][flow.x] <= 0)
        return results;

    const int totalNodes = network.M * network.N;
    auto idxToXY = [&](int id, int& x, int& y) {
        x = id % network.M;
        y = id / network.M;
    };

    // ------------------------
    // 定义A*节点结构
    // ------------------------
    struct Node {
        int id;
        double g;        // 已走代价
        double f;        // 启发式总代价
        double minBw;    // 当前路径瓶颈带宽
        std::vector<int> path;
        bool operator>(const Node& other) const { return f > other.f; }
    };

    auto heuristic = [&](int x, int y, const Flow& f) {
        // 到落地区域中心的欧式距离
        double midx = (f.m1 + f.m2) / 2.0;
        double midy = (f.n1 + f.n2) / 2.0;
        return std::sqrt((x - midx)*(x - midx) + (y - midy)*(y - midy));
    };

    auto computeCost = [&](double bw) {
        // 宽带越大代价越低
        return (bw > 0) ? 1.0 / bw : 1e9;
    };
    struct NodeCmp {
        bool operator()(const std::pair<double, Node>& a,
                        const std::pair<double, Node>& b) const {
            return a.first > b.first; // 仅比较 f 值（越小优先）
        }
    };
    std::priority_queue<std::pair<double, Node>,
                    std::vector<std::pair<double, Node>>,
                    NodeCmp> open;

    Node start;
    start.id = startId;
    start.g = 0;
    start.minBw = bandwidthMatrix[flow.y][flow.x];
    start.path = { startId };

    int sx = flow.x, sy = flow.y;
    double h0 = heuristic(sx, sy, flow);
    start.f = start.g + h0;
    open.push({ start.f, start });

    std::vector<double> bestCost(totalNodes, 1e9);
    bestCost[startId] = 0;

    double bestScore = -1e9;
    Ligne bestLigne;
    std::vector<Ligne> candidates; // 存相近高分路线

    std::set<int> visitedGoals; // 已探索落点避免重复

    while (!open.empty()) {
        auto [_, node] = open.top();
        open.pop();

        int cx, cy;
        idxToXY(node.id, cx, cy);

        // 已到达落地区域
        bool landed = (cx >= flow.m1 && cx <= flow.m2 &&
                       cy >= flow.n1 && cy <= flow.n2);

        if (landed) {
            // 计算路径得分
            double score = computePathScore(node.minBw, node.path.size(), true);

            Ligne l;
            l.flowId = flow.id;
            l.t = t;
            l.pathUavIds = node.path;
            l.bandwidth = node.minBw;
            l.score = score;

            // 更新最优或接近最优路径
            if (score > bestScore) {
                bestScore = score;
                bestLigne = l;
            }
            // 保留接近最优的附近落点路径
            if (score >= bestScore * 0.85 && !visitedGoals.count(node.id)) {
                candidates.push_back(l);
                visitedGoals.insert(node.id);
            }

            // 继续搜索相邻落点
            continue;
        }

        // 扩展邻居
        auto neighbors = getNeighborIds(network, node.id);
        for (int nid : neighbors) {
            int nx, ny;
            idxToXY(nid, nx, ny);

            double bw = bandwidthMatrix[ny][nx];
            if (bw <= 0) continue;

            double gNew = node.g + computeCost(bw);
            if (gNew >= bestCost[nid]) continue; // 已有更优路径

            // 启发式估计
            double fNew = gNew + heuristic(nx, ny, flow);
            double minBwNew = std::min(node.minBw, bw);

            // 动态剪枝：如果潜在分数明显低于当前最优，跳过
            double potential = computePathScore(minBwNew, node.path.size() + 1, false);
            if (bestScore > -1e8 && potential < bestScore * 0.3)
                continue;

            Node next = node;
            next.id = nid;
            next.g = gNew;
            next.f = fNew;
            next.minBw = minBwNew;
            next.path.push_back(nid);

            bestCost[nid] = gNew;
            open.push({ fNew, next });
        }
    }

    // 汇总最终结果
    if (bestScore > -1e8) {
        // 检查 bestLigne 是否已经在 candidates 中
        bool exists = false;
        for (auto& c : candidates) {
            if (c.flowId == bestLigne.flowId &&
                c.t == bestLigne.t &&
                c.pathUavIds == bestLigne.pathUavIds) {
                exists = true;
                break;
            }
        }
    
        // 如果不存在，再插入
        results = candidates;
        if (!exists)
            results.push_back(bestLigne);
    }

    // 排序并截断
    std::sort(results.begin(), results.end(),
              [](const Ligne& a, const Ligne& b) {
                  return a.score > b.score;
              });

    if ((int)results.size() > maxPaths)
        results.resize(maxPaths);

    return results;
}


/**
 * 获取 UAV 邻居编号（4邻域）
 */
std::vector<int> PathFinder::getNeighborIds(const Network& network, int uavId) const {
    std::vector<int> ids;
    int x = uavId % network.M;
    int y = uavId / network.M;

    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    for (int k = 0; k < 4; ++k) {
        int nx = x + dx[k];
        int ny = y + dy[k];
        if (nx >= 0 && nx < network.M && ny >= 0 && ny < network.N) {
            ids.push_back(ny * network.M + nx);
        }
    }
    return ids;
}

/**
 * 启发式路径评分函数
 * 可调整权重以平衡带宽/距离/落地优先级
 */
double PathFinder::computePathScore(double bandwidth, int hopCount, bool lands) const {
    double w_bw = 1.0;
    double w_hop = 0.3;
    double w_land = 0.5;

    double score = w_bw * bandwidth - w_hop * hopCount;
    if (lands) score += w_land; // 到达地面区域的奖励

    return score;
}

/**
 * 根据网络规模和流数量动态确定要保留的路径数量
 */
int PathFinder::computePathBudget(const Network& network) const {
    int F = network.flows.size();
    int MN = network.M * network.N;
    int K_base = 5;
    double alpha = 0.8, beta = 0.5;
    int K = K_base + int(alpha * std::sqrt(F) + beta * std::log(MN));
    return std::min(K, 30);
}
