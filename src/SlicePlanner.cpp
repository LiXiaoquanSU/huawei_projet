#include "SlicePlanner.h"
#include "LigneFinder.h"   
#include <algorithm>
#include <iostream>


SlicePlanner::SlicePlanner(Network& net, int currentT)
    : network(net), t(currentT) {
    initBandwidthMatrix();
}

void SlicePlanner::initBandwidthMatrix() {
    bandwidthMatrix.clear();
    for (const auto& uav : network.uavs) {
        bandwidthMatrix[uav.id] = uav.bandwidthAt(t);
    }
}

void SlicePlanner::applyLigneUsage(const Ligne& ligne) {
    for (int uavId : ligne.pathUavIds) {
        bandwidthMatrix[uavId] = std::max(0.0, bandwidthMatrix[uavId] - ligne.q);
    }
}



std::vector<Ligne> SlicePlanner::generateCandidateLignes() {
    std::vector<Ligne> candidates;

    // 创建路径搜索器（8邻接模式，可改为 FOUR_WAY）
    LigneFinder finder(network, t, LigneFinder::EIGHT_WAY);

    for (const auto& flow : network.flows) {
        if (t < flow.startTime) continue;  // 未开始的流跳过

        // ✅ 调用 LigneFinder 生成最优路径
        Ligne l = finder.findBestPath(flow);

        // ✅ 扣减路径上 UAV 带宽占用
        applyLigneUsage(l);

        // ✅ 保存到候选集
        candidates.push_back(l);
    }

    return candidates;
}


std::vector<Slice> SlicePlanner::planSlices() {
    std::vector<Slice> slices;

    auto candidates = generateCandidateLignes();
    if (candidates.empty()) return slices;

    // 简化：先将每个单条Ligne作为一个Slice，后续再扩展为组合优化
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

const std::map<int, double>& SlicePlanner::getBandwidthMatrix() const {
    return bandwidthMatrix;
}

double SlicePlanner::computeSliceScore(const Slice& slice) const {
    return slice.totalScore;
}
