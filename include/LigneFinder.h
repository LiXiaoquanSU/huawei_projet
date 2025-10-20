#ifndef LIGNEFINDER_H
#define LIGNEFINDER_H

#include "Network.h"
#include "Flow.h"
#include "Ligne.h"
#include <set>
#include <map>
#include <vector>

/**
 * @brief LigneFinder：负责在给定时刻 t 搜索某个流的最优路径集合（A* + 动态阈值剪枝）
 * 
 * 主要功能：
 *  - 从网络的带宽矩阵中搜索所有可能落地的路径；
 *  - 结合带宽、路径长度、历史落点变化惩罚，筛选高分候选；
 *  - 提供单次 runAStarOnce() 接口返回候选集合；
 */
class LigneFinder {
public:
    using XY = std::pair<int,int>;

    LigneFinder(const Network& net,
                const Flow& flow,
                int t,
                const std::map<XY,double>& bw,
                const XY& lastLanding = {-1,-1},
                int landingChangeCount = 0,
                double remainingData = -1)
        : network_(net), flow_(flow), t_(t),
          bw_(bw), lastLanding_(lastLanding),
          landingChangeCount_(landingChangeCount),
          remainingData_(remainingData) {}

    /**
     * @brief 运行一次 A* 搜索，返回候选路径集合
     */
    std::vector<Ligne> runAStarOnce(const std::set<XY>& banSet = {}) const;

private:
    const Network& network_;
    const Flow& flow_;
    int t_;
    const std::map<XY,double>& bw_;

    XY lastLanding_;          // 上一次的落点
    int landingChangeCount_;  // 落点变化次数
    double remainingData_; 
    // 取指定坐标处的临时带宽
    double bwAt(int x, int y) const;

    // 获取上下左右四邻居
    std::vector<XY> neighbors4(int x, int y) const;

    // 判断是否在网格范围内
    bool inGrid(int x, int y) const;

    // 根据第k次落点变化计算惩罚
    double deltaPenaltyForK(int k) const;

    // 对已落地路径施加奖惩
    void applyLandingAdjustment(Ligne& L) const;

    // 根据当前最佳路径计算动态阈值
    double computeThresholdFromBest(const Ligne& best) const;
};

#endif // LIGNEFINDER_H
