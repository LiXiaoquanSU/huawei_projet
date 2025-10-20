#include "LigneFinder.h"
#include <cmath>
#include <algorithm>
#include <iostream>

LigneFinder::LigneFinder(const Network& net, int currentT)
    : network(net), t(currentT) {}

/**
 * @brief 计算 UAV 带宽的代价（带宽越低代价越高）
 */
double LigneFinder::bandwidthCost(int x, int y) const {
    const UAV* uav = network.getUAV(x, y);
    if (!uav) return 1e6; // 无效节点惩罚
    double bw = uav->bandwidthAt(t);
    if (bw <= 1e-6) return 1e5; // 零带宽节点惩罚
    return 1.0 / bw; // 反比代价
}

/**
 * @brief 计算启发函数 h
 *        - FOUR_WAY 使用曼哈顿距离
 *        - EIGHT_WAY 使用欧几里得距离
 */
double LigneFinder::heuristic(int x1, int y1, int x2, int y2) const {
        return std::abs(x1 - x2) + std::abs(y1 - y2);
}

/**
 * @brief 获取邻接节点坐标
 */
std::vector<std::pair<int,int>> LigneFinder::getNeighbors(int x, int y) const {
    std::vector<std::pair<int,int>> dirs;
        dirs = {{1,0},{-1,0},{0,1},{0,-1}};
    std::vector<std::pair<int,int>> res;
    for (auto [dx,dy] : dirs) {
        int nx = x + dx, ny = y + dy;
        if (nx >= 0 && nx < network.M && ny >= 0 && ny < network.N)
            res.emplace_back(nx, ny);
    }
    return res;
}

/**
 * @brief 在给定时刻 t 下，为一个流执行 A* 搜索，找到最优路径
 */
Ligne LigneFinder::findBestPath(const Flow& flow) {
    Ligne result;
    result.flowId = flow.id;
    result.t = t;
    result.t_start = flow.startTime;
    result.Q_total = flow.size;

    // 如果流还没开始则直接返回空路径
    if (t < flow.startTime) return result;

    const int startX = flow.x;
    const int startY = flow.y;

    // ========== A* 初始化 ==========
    std::priority_queue<Node> openSet;
    std::map<std::pair<int,int>, Node> allNodes;
    std::set<std::pair<int,int>> closedSet;

    Node start {startX, startY, 0.0, 0.0, 0.0, -1, -1};
    start.h = heuristic(startX, startY, flow.m1, flow.n1); // 先用左上角估计
    start.f = start.g + start.h;
    openSet.push(start);
    allNodes[{startX, startY}] = start;

    bool found = false;
    std::pair<int,int> goal;

    // ========== 主循环 ==========
    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (closedSet.count({current.x, current.y})) continue;
        closedSet.insert({current.x, current.y});

        // 判断是否到达落地区域
        if (flow.inLandingRange(current.x, current.y)) {
            found = true;
            goal = {current.x, current.y};
            break;
        }

        // 遍历邻居
        for (auto [nx, ny] : getNeighbors(current.x, current.y)) {
            if (closedSet.count({nx, ny})) continue;

            double stepCost = 1.0 + bandwidthCost(nx, ny);
            double tentative_g = current.g + stepCost;

            // 对落地区域中心的启发估计
            double hx = std::clamp((flow.m1 + flow.m2) / 2.0, 0.0, (double)network.M);
            double hy = std::clamp((flow.n1 + flow.n2) / 2.0, 0.0, (double)network.N);
            double h = heuristic(nx, ny, (int)hx, (int)hy);

            double f = tentative_g + h;

            auto key = std::make_pair(nx, ny);
            if (!allNodes.count(key) || tentative_g < allNodes[key].g) {
                Node next {nx, ny, tentative_g, h, f, current.x, current.y};
                openSet.push(next);
                allNodes[key] = next;
            }
        }
    }

    // ========== 若找到路径则回溯 ==========
    std::vector<std::pair<int,int>> path;
    if (found) {
        std::pair<int,int> cur = goal;
        while (true) {
            path.push_back(cur);
            auto nodeIt = allNodes.find(cur);
            if (nodeIt == allNodes.end()) break;
            Node n = nodeIt->second;
            if (n.parentX == -1) break;
            cur = {n.parentX, n.parentY};
        }
        std::reverse(path.begin(), path.end());
    } else {
        // 没找到路径则返回原地
        path.push_back({startX, startY});
    }

    // ========== 构造 Ligne ==========
    result.pathUavIds.clear();
    result.gridWidth = network.M;
    double minBandwidth = 1e9;
    for (auto [x, y] : path) {
        int uavId = y * network.M + x;
        result.pathUavIds.push_back(uavId);
        UAV* uav = network.getUAV(x, y);
        if (uav)
            minBandwidth = std::min(minBandwidth, uav->bandwidthAt(t));
    }

    result.distance = path.size() - 1;
    result.bandwidth = (minBandwidth < 1e9 ? minBandwidth : 0.0);
    result.q = result.bandwidth;  // 简化：一秒内按瓶颈带宽传输
    result.landed = flow.inLandingRange(path.back().first, path.back().second);

    // 计算得分
    result.computeScore(flow.x, flow.y, flow.m1, flow.n1, flow.m2, flow.n2);

    return result;
}
