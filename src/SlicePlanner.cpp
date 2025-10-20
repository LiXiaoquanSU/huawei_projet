#include "SlicePlanner.h"
#include <algorithm>
#include <iostream>

SlicePlanner::SlicePlanner(Network& net, int currentT)
    : network(net), t(currentT) {
    initBandwidthMatrix();
}

/**
 * @brief 初始化带宽矩阵
 */
void SlicePlanner::initBandwidthMatrix() {
    bandwidthMatrix.clear();
    for (const auto& uav : network.uavs) {
        bandwidthMatrix[uav.id] = uav.bandwidthAt(t);
    }
}

/**
 * @brief 应用一条路径占用带宽
 */
void SlicePlanner::applyLigneUsage(const Ligne& ligne) {
    // 简化：暂不真正修改带宽，只做占位
    (void)ligne;
}

/**
 * @brief 生成可行路径候选（空实现，不报错即可）
 */
std::vector<Ligne> SlicePlanner::generateCandidateLignes() {
    std::vector<Ligne> candidates;

    // 示例：生成一个空 Ligne 用于防止报错
    if (!network.flows.empty()) {
        Ligne L;
        L.flowId = network.flows.front().id;
        L.t = t;
        L.score = 0.0;
        candidates.push_back(L);
    }

    return candidates;
}

/**
 * @brief 规划所有可能的 Slice
 */
std::vector<Slice> SlicePlanner::planSlices() {
    std::vector<Slice> slices;
    auto candidates = generateCandidateLignes();

    if (candidates.empty()) return slices;

    // 暂时：每个 Ligne 生成一个 Slice
    for (auto& l : candidates) {
        Slice s(t, {l});
        s.computeTotalScore();
        slices.push_back(s);
    }

    // 按得分排序
    std::sort(slices.begin(), slices.end(),
              [](const Slice& a, const Slice& b) {
                  return a.totalScore > b.totalScore;
              });

    return slices;
}

/**
 * @brief 获取当前带宽矩阵
 */
const std::map<int, double>& SlicePlanner::getBandwidthMatrix() const {
    return bandwidthMatrix;
}

/**
 * @brief 计算单个 Slice 的得分（暂时返回其 totalScore）
 */
double SlicePlanner::computeSliceScore(const Slice& slice) const {
    return slice.totalScore;
}
