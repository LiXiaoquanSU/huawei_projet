#ifndef LIGNE_H
#define LIGNE_H

#include <vector>
#include <cmath>
#include <tuple>

/**
 * @brief 表示单个时间刻 (t) 内的一条 UAV 传输路径
 * 
 * Tmax 为流的最大传输时延（单位：秒）
 * 
 * 说明：
 *  - Ligne 不再记录落点变化状态；
 *  - 由 SlicePlanner 计算落点惩罚并传入；
 *  - computeScore() 可接收外部落点惩罚参数。
 */
struct Ligne {
    // ================= 基本属性 =================
    int flowId;                   // 所属流编号
    int t;                        // 当前时刻
    int t_start;                  // 流的起始时间
    std::vector<int> pathUavIds;  // UAV 节点路径（编号序列）

    // ================= 路径性能 =================
    double bandwidth;             // 瓶颈带宽
    double q;                     // 实际传输流量（Mbps）
    double Q_total;               // 流的总数据量（Mbps）
    double distance;              // 路径跳数（=pathUavIds.size() - 1）
    double Tmax;                  // ⏱️ 最大允许传输时延（秒）

    // ================= 状态与评分 =================
    bool landed;                  // 是否已到达落地点
    double score;                 // 本路径得分

    // ================= 构造与函数 =================
    Ligne();

    /**
     * @brief 计算路径得分（支持外部落点惩罚参数）
     * @param landingPenalty 落点变化惩罚值（由 SlicePlanner 计算）
     */
    void computeScore(int currentX, int currentY,
                      int landingX1, int landingY1,
                      int landingX2, int landingY2,
                      double alpha = 0.1,
                      double beta = 1.0);

    /**
     * @brief 导出当前路径的标准输出项 (t, endX, endY, q)
     * @param M 网络宽度，用于将 UAV 编号转换为坐标
     */
    std::tuple<int, int, int, double> exportOutput(int M) const;
};

#endif // LIGNE_H
