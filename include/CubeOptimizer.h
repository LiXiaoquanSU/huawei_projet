#ifndef CUBE_OPTIMIZER_H
#define CUBE_OPTIMIZER_H

#include "Cube.h"
#include "Network.h"
#include "LigneFinder.h"
#include "SlicePlanner.h"
#include <map>
#include <vector>
#include <tuple>
#include <optional>
#include <utility>
#include <iostream>

/**
 * @brief CubeOptimizer：对 DTCubeBuilder 产出的 Cube 做“单位得分效率”再平衡优化
 *
 * 核心步骤：
 *  1) 从 Cube 提取“确定表”（C 表）：记录每时刻每流的 q/score/eff/endXY
 *  2) 构建“潜力表”（P 表）：在扣除了其它流已定占用的临时带宽上，对每个 (t,flow)
 *     用 LigneFinder 搜最优路线（带入 last/next/changeCount/neighborState/remaining），取单位得分最高者
 *  3) 选择 Δeff 最大的流与时刻，从低效时刻 t_low 让出流量到高效时刻 t_high
 *     用 SlicePlanner 在 t_high 层做一次“整层重排”（保留其它流上限/意愿可按需要扩展）
 *  4) 迭代直到没有 Δeff>0
 */
class CubeOptimizer {
public:
    CubeOptimizer(const Network& net, const Cube& inputCube);

    /// 执行优化过程，返回优化后的 Cube
    Cube optimize();

private:
    const Network& network_;
    Cube cube_;  // 工作副本

    struct CellData {
        double q{0.0};
        double score{0.0};
        double eff{0.0};
        bool   valid{false};
        std::pair<int,int> endXY{-1,-1};
    };
    using Table = std::map<int, std::map<int, CellData>>; // flowId -> (t -> cell)

    Table confirmedTable_;   // C 表
    Table baselineConfirmedTable_; // 初始 C 表（用于保持原始效率排序）
    Table potentialTable_;   // P 表

    // ====== 日志开关（需要静默时改为 false 即可，不影响逻辑）======
    static constexpr bool LF_DEBUG = true;
    static constexpr double EPS = 1e-9;

private:
    // -------- 构表/查询工具 --------
    void buildConfirmedTable();
    void buildPotentialTable();

    // 计算在时刻 t、流 fid 的：
    //  - lastLanding(t-1)、nextLanding(t+1)
    //  - landingChangeCount（从最早到 t-1 的变化次数）
    //  - remaining（到 t 之前未传的数据量）
    //  - neighborState（按 0/1/2 规则）
    std::pair<int,int> getLastLanding(int fid, int t) const;
    std::pair<int,int> getNextLanding(int fid, int t) const;
    int  getLandingChangeCountGlobal(int fid) const;
    double getRemainingUntil(int fid, int t) const;
    int  getNeighborState(int fid, int t) const;

    // 在时刻 t 构造“扣除了其它流占用”的带宽矩阵（但对 fid 自身不扣）
    std::map<std::pair<int,int>, double> makeMaskedBwForPotential(int fid, int t) const;

    // 选择 Δeff 最大者：返回 (flowId, t_high, gap, hasCapacityGain)
    std::tuple<int,int,double,bool> findMaxEfficiencyGap() const;

    // 执行一次“从 t_low → t_high 的流量再分配 + t_high 层重排”
    void rebalanceFlow(int fid, int t_high, int t_low);

    // 打印表摘要
    void logTableSummary(const std::string& name, const Table& tbl) const;
    void visualPrintCube(const std::string& title) const;
    void visualPrintTableC() const;
    void visualPrintTableP() const;

    // --------- 内部辅助 ---------
    const Flow* findFlow(int fid) const;
    std::optional<Ligne> computeBestPotentialLigne(int fid, int t) const;
    bool applyDirectReplacement(int fid, int t, double desiredQ);
};

#endif // CUBE_OPTIMIZER_H
