#include "SlicePlanner.h"
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

    for (const auto& flow : network.flows) {
        if (t < flow.startTime) continue; // 尚未开始的流跳过

        // 这里将来可以调用 PathFinder 或自定义算法来生成路径
        Ligne l;
        l.flowId = flow.id;
        l.t = t;
        l.t_start = flow.startTime;
        l.Q_total = flow.size;
        l.pathUavIds.push_back(flow.y * network.M + flow.x); // 暂定访问点
        l.q = 0.0;
        l.computeScore(flow.x, flow.y, flow.m1, flow.n1, flow.m2, flow.n2);
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
