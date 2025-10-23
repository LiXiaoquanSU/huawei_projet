#include "DTCube.h"

#include <limits>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>

using XY = std::pair<int,int>;

// ====== 日志开关（需要静默时改为 false 即可，不影响逻辑）======
static constexpr bool LF_DEBUG = true;

DTCubeBuilder::DTCubeBuilder(Network& net)
    : network(net), T(net.T) {}

Cube DTCubeBuilder::build() {
    if (LF_DEBUG) std::cout << "=== 开始构建 DTCube ===" << std::endl;

    // 初始全局状态
    std::map<int,double> remaining;
    std::map<int,XY>     lastLanding;
    std::map<int,XY>     nextLanding;
    std::map<int,int>    changeCount;
    std::map<int,int>    neighborState;

    for (const auto& f : network.flows) {
        remaining[f.id]   = f.size;
        lastLanding[f.id] = {-1,-1};
        nextLanding[f.id] = {-1,-1};
        changeCount[f.id] = 0;
        neighborState[f.id] = 1;
    }

    std::vector<Slice> currentPath;
    std::vector<Slice> bestPath;
    double bestScore = -std::numeric_limits<double>::infinity();

    dfs(0, currentPath, 0.0, bestScore, bestPath,
        remaining, lastLanding, nextLanding, changeCount, neighborState);

    Cube cube(T);
    for (const auto& s : bestPath) cube.addSlice(s);
    return cube;
}

void DTCubeBuilder::dfs(int t,
                        std::vector<Slice>& currentPath,
                        double currentScore,
                        double& bestScore,
                        std::vector<Slice>& bestPath,
                        std::map<int,double> remaining,
                        std::map<int,XY>    lastLanding,
                        std::map<int,XY>    nextLanding,
                        std::map<int,int>   changeCount,
                        std::map<int,int>   neighborState)
{
    if (LF_DEBUG) std::cout << "[递归] 时刻 t=" << t << " 开始运行" << std::endl;

    // 终止
    if (t >= T || allFinished(remaining)) {
        if (currentScore > bestScore || bestPath.empty()) {
            bestScore = currentScore;
            bestPath  = currentPath;
        }
        return;
    }

    // 1) 构造带宽图
    auto bw = makeBandwidthMap(t);

    // 2) 生成候选切片
    SlicePlanner planner(network, remaining, lastLanding, nextLanding,
                         changeCount, neighborState, t, bw);
    auto candidates = planner.planAllSlices();

    if (LF_DEBUG)
        std::cout << "  → SlicePlanner 返回了 " << candidates.size() << " 个 Slice" << std::endl;

    if (candidates.empty()) {
        Slice empty = makeEmptySlice(t);
        currentPath.push_back(empty);
        dfs(t+1, currentPath, currentScore, bestScore, bestPath,
            remaining, lastLanding, nextLanding, changeCount, neighborState);
        currentPath.pop_back();
        return;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Slice& a, const Slice& b){
                  return computeSliceScore(a) > computeSliceScore(b);
              });
    const int BEAM = 20;
    if ((int)candidates.size() > BEAM) candidates.resize(BEAM);

    // 3) 遍历候选
    for (const auto& s : candidates) {
        double sliceScore = computeSliceScore(s);

        auto rem2 = remaining;
        auto last2 = lastLanding;
        auto chg2  = changeCount;
        updateStateWithSlice(s, rem2, last2, chg2);

        currentPath.push_back(s);
        dfs(t+1, currentPath, currentScore + sliceScore,
            bestScore, bestPath, rem2, last2, nextLanding, chg2, neighborState);
        currentPath.pop_back();
    }
}

std::map<XY,double> DTCubeBuilder::makeBandwidthMap(int t) const {
    std::map<XY,double> bw;
    for (const auto& u : network.uavs) {
        bw[{u.x, u.y}] = u.bandwidthAt(t);
    }
    return bw;
}

double DTCubeBuilder::computeSliceScore(const Slice& s) {
    double total = 0.0;
    for (const auto& L : s.lignes) total += L.score;
    return total;
}

void DTCubeBuilder::updateStateWithSlice(const Slice& s,
                                         std::map<int,double>& remaining,
                                         std::map<int,XY>&    lastLanding,
                                         std::map<int,int>&   changeCount)
{
    for (const auto& L : s.lignes) {
        remaining[L.flowId] = std::max(0.0, remaining[L.flowId] - L.q);
        if (!L.pathXY.empty()) {
            auto [ex, ey] = L.pathXY.back();
            auto last = lastLanding[L.flowId];
            if (last.first != -1 && (last.first != ex || last.second != ey)) {
                changeCount[L.flowId] += 1;
            }
            lastLanding[L.flowId] = {ex, ey};
        }
    }
}

bool DTCubeBuilder::allFinished(const std::map<int,double>& remaining) {
    for (auto& kv : remaining) if (kv.second > 1e-9) return false;
    return true;
}

Slice DTCubeBuilder::makeEmptySlice(int t) const {
    Slice empty(t);
    return empty;
}
