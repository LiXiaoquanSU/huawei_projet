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
        std::pair<int,int> pos = {uav.x, uav.y};
        bandwidthMatrix[pos] = uav.bandwidthAt(t);
    }
}

/**
 * @brief 应用一条路径占用带宽
 */
void SlicePlanner::applyLigneUsage(const Ligne& ligne) {
<<<<<<< HEAD
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
=======
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
>>>>>>> main
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

<<<<<<< HEAD
/**
 * @brief 获取当前带宽矩阵
 */
const std::map<int, double>& SlicePlanner::getBandwidthMatrix() const {
=======
const std::map<std::pair<int,int>, double>& SlicePlanner::getBandwidthMatrix() const {
>>>>>>> main
    return bandwidthMatrix;
}

/**
 * @brief 计算单个 Slice 的得分（暂时返回其 totalScore）
 */
double SlicePlanner::computeSliceScore(const Slice& slice) const {
    return slice.totalScore;
}
