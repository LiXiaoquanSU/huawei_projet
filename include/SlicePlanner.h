#ifndef SLICEPLANNER_H
#define SLICEPLANNER_H

#include "Network.h"
#include "Slice.h"
#include "Ligne.h"
#include "LigneFinder.h"
#include <vector>
#include <map>
#include <set>

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
     */
    std::vector<Slice> planSlices();

    /**
     * @brief 获取当前带宽矩阵
     */
    const std::map<int, double>& getBandwidthMatrix() const;

private:
    Network& network;
    int t; ///< 当前时刻

    // UAV带宽矩阵：uavId -> 可用带宽(Mbps)
    std::map<int, double> bandwidthMatrix;

    // 计算当前时刻所有可行路径候选
    std::vector<Ligne> generateCandidateLignes();

    // 计算单个Slice得分
    double computeSliceScore(const Slice& slice) const;
};

#endif // SLICEPLANNER_H
