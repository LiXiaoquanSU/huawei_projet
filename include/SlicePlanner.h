#ifndef SLICEPLANNER_H
#define SLICEPLANNER_H

#include "Network.h"
#include "Slice.h"
#include "Ligne.h"
#include "LigneFinder.h" 
#include <vector>
#include <map>

/**
 * @brief SlicePlanner 负责在单个时刻 t 上生成所有可能的 Slice 组合
 * 
 * 输入：当前时刻 t、Network、当前带宽矩阵
 * 输出：若干可能的 Slice（按得分排序）
 */
class SlicePlanner {
public:
    SlicePlanner(Network& net, int currentT);

    /**
     * @brief 初始化带宽矩阵（由 Network.uavs 计算）
     */
    void initBandwidthMatrix();

    /**
     * @brief 确认一条路径使用后，更新带宽矩阵
     */
    void applyLigneUsage(const Ligne& ligne);

    /**
     * @brief 主规划函数：生成若干可能的 Slice（按得分排序）
     * @param lastLandings 上一时刻各流的落点位置 (flowId -> (x,y))
     * @param landingChangeCounts 各流落点变化次数 (flowId -> count)
     */
    std::vector<Slice> planSlices(
        const std::map<int, std::pair<int,int>>& lastLandings = {},
        const std::map<int, int>& landingChangeCounts = {}
    );

    /**
     * @brief 获取当前带宽矩阵（坐标形式）
     */
    const std::map<std::pair<int,int>, double>& getBandwidthMatrix() const;

private:
    Network& network;
    int t; ///< 当前时刻

    // 带宽矩阵：(x,y) -> 可用带宽(Mbps)
    std::map<std::pair<int,int>, double> bandwidthMatrix;

    // 候选集上限
    static constexpr int beamSlices = 10;

    // 使用 LigneFinder 生成单个流的候选路径
    std::vector<Ligne> generateCandidateLignes(
        const Flow& flow,
        const std::pair<int,int>& lastLanding,
        int landingChangeCount
    );

    // 计算单个Slice得分
    double computeSliceScore(const Slice& slice) const;
};

#endif // SLICEPLANNER_H
