#ifndef LIGNE_H
#define LIGNE_H

#include <vector>

/**
 * @brief 表示单个时间刻 (t) 内的一条可行传输路径
 * 
 * 一条数据流可以在一个时刻内通过多个 UAV (多跳)，直到落地。
 * 
 * 示例：flow1 在 t=5 时刻
 *   UAV路径序列: [12, 13, 18] （编号）
 *   带宽瓶颈: 8.5 Mbps
 */
struct Ligne {
    int flowId;                   // 所属数据流编号
    int t;                        // 当前时刻
    std::vector<int> pathUavIds;  // 经过的 UAV 编号序列（例如 [3, 5, 8]）
    double bandwidth;             // 路径瓶颈带宽（最小链路带宽）
    double score;                 // 综合得分（启发式计算）
};

#endif // LIGNE_H
