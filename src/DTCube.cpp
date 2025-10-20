// #include "DTCube.h"

// #include <limits>
// #include <vector>

// DTCubeBuilder::DTCubeBuilder(Network& net)
//     : network(net), T(net.T) {}

// Cube DTCubeBuilder::build() {
//     // currentPath 随着递归深入按顺序记录每个时刻的 Slice
//     std::vector<Slice> currentPath;
//     // bestPath 用于存储迄今为止得分最高的 Slice 序列
//     std::vector<Slice> bestPath;
//     double bestScore = -std::numeric_limits<double>::infinity();

//     // 从 t=0 开始展开整棵决策树
//     buildDecisionTree(0, currentPath, 0.0, bestScore, bestPath);

//     Cube cube(T);
//     // 将最佳路径中每个 Slice 写入 Cube
//     for (const auto& slice : bestPath) {
//         cube.addSlice(slice);
//     }
//     // 汇总得分，方便后续直接读取 Cube.totalScore
//     cube.computeTotalScore();
//     return cube;
// }

// void DTCubeBuilder::buildDecisionTree(int t,
//                                       std::vector<Slice>& currentPath,
//                                       double currentScore,
//                                       double& bestScore,
//                                       std::vector<Slice>& bestPath) {
//     // 当所有时刻均已处理时，根据累计得分更新全局最优解
//     if (t >= T) {
//         if (currentScore > bestScore || bestPath.empty()) {
//             bestScore = currentScore;
//             bestPath = currentPath;
//         }
//         return;
//     }

//     // 针对时刻 t 生成候选 Slice
//     SlicePlanner planner(network, t);
//     auto candidates = planner.planSlices();

//     // 若没有任何候选，创建一个空 Slice 占位，以保持时间轴连续
//     if (candidates.empty()) {
//         Slice emptySlice = makeEmptySlice(t);
//         currentPath.push_back(emptySlice);
//         buildDecisionTree(t + 1, currentPath, currentScore + emptySlice.totalScore,
//                           bestScore, bestPath);
//         currentPath.pop_back();
//         return;
//     }

//     // 依次尝试每个候选 Slice，并继续向下递归
//     for (auto& slice : candidates) {
//         // slice 可能尚未显式计算得分，这里确保 totalScore 可用
//         if (slice.totalScore == 0.0 && !slice.lignes.empty()) {
//             slice.computeTotalScore();
//         }
//         currentPath.push_back(slice);
//         buildDecisionTree(t + 1, currentPath, currentScore + slice.totalScore,
//                           bestScore, bestPath);
//         currentPath.pop_back();
//     }
// }

// Slice DTCubeBuilder::makeEmptySlice(int t) const {
//     // 创建一个不包含任何路径的 Slice，用于没有业务的时刻
//     Slice empty(t, {});
//     empty.computeTotalScore();
//     return empty;
// }
