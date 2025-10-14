#ifndef SLICERESOLVER_H
#define SLICERESOLVER_H

#include "PathFinder.h"
#include <vector>
#include <map>

struct Slice {
    int t;
    std::vector<Ligne> lignes; // 同时调度的多条路径
    double totalScore;
};

class SliceResolver {
public:
    SliceResolver();

    /**
     * @brief 将多条路径（ligne）整合为无冲突的切片（slice）
     * @param lignes 当前时刻所有流的候选路径
     * @param t 当前时刻
     * @return 无冲突的调度切片集合
     */
    std::vector<Slice> resolveConflicts(const std::vector<Ligne>& lignes, int t);

private:
    // 检查两个路径是否冲突（共享同一 UAV 带宽）
    bool hasConflict(const Ligne& a, const Ligne& b) const;

    // 分配带宽，调整冲突路径速率
    void adjustForBandwidth(std::vector<Ligne>& lignes);

    // 计算切片总得分
    double computeSliceScore(const Slice& slice) const;
};

#endif // SLICERESOLVER_H
