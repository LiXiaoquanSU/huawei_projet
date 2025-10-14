#ifndef CUBEOPTIMIZER_H
#define CUBEOPTIMIZER_H

#include "SliceResolver.h"
#include <vector>
#include <map>

/**
 * Cube 代表整个时间周期的调度结果。
 * 每个 Slice 表示一个时刻或时间段的调度方案。
 */
struct Cube {
    std::vector<Slice> slices;
    double totalScore;
};

class CubeOptimizer {
public:
    CubeOptimizer();

    /**
     * @brief 将某一时刻的 slice 组合加入全局优化模型
     */
    void integrateSlice(const std::vector<Slice>& slices, int t);

    /**
     * @brief 在所有时间片结束后，选择最优组合输出结果
     */
    Cube finalizeResults(std::ostream& out) const;

private:
    std::map<int, std::vector<Slice>> timeSlices; // 每秒的切片候选
    Cube bestCube; // 最优解

    // 动态规划 / 回溯拼接
    void buildBestCube();

    // 计算整体得分
    double computeCubeScore(const Cube& cube) const;
};

#endif // CUBEOPTIMIZER_H
