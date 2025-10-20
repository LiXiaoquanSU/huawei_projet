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
        std::pair<int,int> pos = {uav.x, uav.y};
        bandwidthMatrix[pos] = uav.bandwidthAt(t);
    }
}

void SlicePlanner::applyLigneUsage(const Ligne& ligne) {
    // 从路径上每个节点扣减使用的带宽
    for (const auto& [x, y] : ligne.pathXY) {
        std::pair<int,int> pos = {x, y};
        if (bandwidthMatrix.count(pos)) {
            bandwidthMatrix[pos] = std::max(0.0, bandwidthMatrix[pos] - ligne.q);
        }
    }
}

std::vector<Ligne> SlicePlanner::generateCandidateLignes(
    const Flow& flow,
    const std::pair<int,int>& lastLanding,
    int landingChangeCount) {
    
    // 创建 LigneFinder 并使用 runAStarOnce() 生成候选路径
    LigneFinder finder(network, flow, t, bandwidthMatrix, lastLanding, landingChangeCount);
    
    return finder.runAStarOnce();
}


std::vector<Slice> SlicePlanner::planSlices(
    const std::map<int, std::pair<int,int>>& lastLandings,
    const std::map<int, int>& landingChangeCounts) {
    
    std::vector<Slice> slicesCandidates;
    
    // 1. 为每个流生成候选路径
    std::map<int, std::vector<Ligne>> flowCandidates;
    
    for (const auto& flow : network.flows) {
        if (t < flow.startTime) continue;  // 未开始的流跳过
        
        // 获取历史信息
        std::pair<int,int> lastLanding = {-1, -1};
        int changeCount = 0;
        
        auto lastIt = lastLandings.find(flow.id);
        if (lastIt != lastLandings.end()) {
            lastLanding = lastIt->second;
        }
        
        auto countIt = landingChangeCounts.find(flow.id);
        if (countIt != landingChangeCounts.end()) {
            changeCount = countIt->second;
        }
        
        // 生成该流的候选路径
        auto candidates = generateCandidateLignes(flow, lastLanding, changeCount);
        if (!candidates.empty()) {
            flowCandidates[flow.id] = std::move(candidates);
        }
    }
    
    if (flowCandidates.empty()) return slicesCandidates;
    
    // 2. 枚举所有可能的路径组合生成 Slice
    // 简化实现：为每个流选择最佳路径组成一个 Slice
    std::vector<Ligne> bestLignes;
    
    // 备份带宽矩阵
    auto originalBandwidth = bandwidthMatrix;
    
    // 按流 ID 顺序依次选择最佳路径
    for (const auto& [flowId, candidates] : flowCandidates) {
        if (!candidates.empty()) {
            // 选择得分最高的路径
            const Ligne& bestLigne = candidates[0];  // 已按得分排序
            bestLignes.push_back(bestLigne);
            
            // 从带宽矩阵中扣减占用
            applyLigneUsage(bestLigne);
        }
    }
    
    // 3. 创建 Slice 并计算得分
    if (!bestLignes.empty()) {
        Slice slice(t, bestLignes);
        slice.computeTotalScore();
        slicesCandidates.push_back(slice);
    }
    
    // 恢复原始带宽矩阵
    bandwidthMatrix = originalBandwidth;
    
    // 4. 按得分排序，限制候选集数量
    std::sort(slicesCandidates.begin(), slicesCandidates.end(),
              [](const Slice& a, const Slice& b) {
                  return a.totalScore > b.totalScore;
              });
    
    if (slicesCandidates.size() > beamSlices) {
        slicesCandidates.resize(beamSlices);
    }
    
    return slicesCandidates;
}

const std::map<std::pair<int,int>, double>& SlicePlanner::getBandwidthMatrix() const {
    return bandwidthMatrix;
}

double SlicePlanner::computeSliceScore(const Slice& slice) const {
    return slice.totalScore;
}
