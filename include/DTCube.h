#ifndef DTCUBE_H
#define DTCUBE_H

#include <vector>
#include "Cube.h"
#include "Network.h"
#include "SlicePlanner.h"

/**
 * @brief 负责生成完整Slice决策树（逐时刻添加 Slice到树上），并将slice树从叶子节点向上逐层提取为Cube。
 *
 * 说明：
 * - 通过递归方式构建决策树，每个节点代表某个时刻的 Slice 选择；
 * - DTCubeBuilder 使用 SlicePlanner 在某个 slice 节点每个时刻生成多个候选 Slice 并将其添加到该节点下；
 * - 添加完成后，DTCubeBuilder 会从叶子节点开始，逐层向上提取父节点的 Slice 并将其添加到子节点的 Cube 中；
 * - 比较各个子节点的 Cube 得分，选择得分最高的 Cube 作为该父节点的最佳 Cube；
 * - 最终形成一个完整的 Cube；
 * - 最终生成的 Cube 包含所有时刻的最优 Slice 组合。
 */
class DTCubeBuilder {
public:
    explicit DTCubeBuilder(Network& net);

    /**
     * @brief 构建一个覆盖 [0, T) 所有时刻的 Cube
     */
    Cube build();

private:
    Network& network;
    int T;

    /**
     * @brief 深度优先搜索整棵决策树，记录得分最高的 Slice 组合
     *
     * @param t             当前处理的时刻索引
     * @param currentPath   递归路径上已有的 Slice 列表（长度等于 t）
     * @param currentScore  currentPath 中所有 Slice 的得分累计
     * @param bestScore     全局最佳得分（引用参数，用于原地更新）
     * @param bestPath      全局最佳 Slice 列表（引用参数，用于原地更新）
     */
    void buildDecisionTree(int t,
                           std::vector<Slice>& currentPath,
                           double currentScore,
                           double& bestScore,
                           std::vector<Slice>& bestPath);

    Slice makeEmptySlice(int t) const;
};

#endif // DTCUBE_H
