#ifndef LIGNE_H
#define LIGNE_H

#include <vector>
#include <cmath>
#include <tuple>
//把落点记录换到SlicePlanner
/**
 * @brief 表示单个时间刻 (t) 内的一条 UAV 传输路径
 * 
 * Tmax 为流的最大传输时延（单位：秒）
 * 
 * 新增：
 *  - prevLandingId：上一落点 UAV 的 id（-1 表示无）
 *  - landingChangeCount：累计落点变化次数（至少为1）
 *  - LandingPenalty：本次变化带来的惩罚值（仅在落点变化时更新）
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
    int prevLandingId;            // 上一落点 UAV id
    int landingChangeCount;       // 落点变化次数
    double LandingPenalty;        // 本次变化的惩罚值

    // ================= 构造与函数 =================
    Ligne();

    /**
     * @brief 计算路径得分（包含落点变化惩罚）
     */
    void computeScore(int currentX, int currentY,
                      int landingX1, int landingY1,
                      int landingX2, int landingY2,
                      double alpha = 0.1,
                      double beta = 1.0);

    /**
     * @brief 导出当前路径的标准输出项 (t, endX, endY, q)
     */
    std::tuple<int, int, int, double> exportOutput(int M) const;
};

#endif // LIGNE_H
