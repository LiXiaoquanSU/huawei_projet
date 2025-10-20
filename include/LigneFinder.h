#ifndef LIGNEFINDER_H
#define LIGNEFINDER_H

#include <vector>
#include <map>
#include <set>
#include <utility>
#include "Network.h"
#include "Ligne.h"

/**
 * @brief 单时刻A*搜索器（按score最大化）
 *
 * 使用说明：
 *  - 构造时注入 network、flow、带宽矩阵、当前时刻t、上次落点与变化次数；
 *  - 调用 search() 返回若干 Ligne 候选（已按你的逻辑留好挂点）。
 *
 * 约定：
 *  - 路径坐标用 (x,y) 存在 Ligne::pathXY 中；
 *  - 带宽矩阵以 (x,y) 为键：std::map<std::pair<int,int>, double>；
 *  - 仅允许 4 邻接；
 *  - A* 的优先级直接使用 Ligne::computeScore() 得到的“乐观得分估计”（最大堆思想）。
 */
class LigneFinder {
public:
    using XY = std::pair<int,int>;
    using BWMatrix = std::map<XY, double>;

    LigneFinder(const Network& net,
                const Flow& flow,
                const BWMatrix& bw,
                int t,
                XY lastLanding,
                int landingChangeCount);

    /**
     * @brief 运行搜索，返回候选路径集合
     * @param maxAttempts 为了挖掘多条不同路径的尝试次数（通过ban中间节点）
     * @return 若干候选 Ligne（按score从高到低排序的建议由调用方处理或内部处理）
     */
    std::vector<Ligne> search(int maxAttempts = 8);

private:
    // ===== 输入上下文 =====
    const Network& network_;
    const Flow&    flow_;
    const BWMatrix& bw_;        // 临时带宽矩阵 (x,y) -> 可用带宽
    int t_;                     // 当前时刻
    XY  lastLanding_;           // 上一次落点 (-1,-1 表首次)
    int landingChangeCount_;    // 落点变化次数（历史）

    // ===== 内部工具：网格/邻接 =====
    inline bool inGrid(int x, int y) const;
    std::vector<XY> neighbors4(int x, int y) const;

    // ===== 内部工具：带宽与路径 =====
    double bwAt(int x, int y) const; // 从 bw_ 获取 (x,y) 带宽（默认0）
    double pathBottleneckBW(const std::vector<XY>& path) const;

    // 用已有 (x,y) 路径构造 Ligne（设置 pathXY/q/bandwidth/distance/landed 等，调用 computeScore）
    Ligne buildLigneFromPath(const std::vector<XY>& path,
                             double Q_remaining /*可按需使用*/) const;

    // 对 Ligne.score 施加 “首次落地奖励 / 落点变化惩罚”
    void applyLandingBonusOrPenalty(Ligne& L) const;

    // ===== A* 一次运行（返回一条完整落地路径；失败返回空）=====
    // banSet：禁止作为“中间节点”的坐标（终点若在落区直接返回，不受ban约束）
    std::vector<XY> runAStarOnce(const std::set<XY>& banSet) const;

    // ===== 候选集合策略（双条件筛选）=====
    // candidateMap：key=落点坐标，value=该落点的若干优选Ligne
    using CandidateMap = std::map<XY, std::vector<Ligne>>;
    void tryInsertCandidate(CandidateMap& cmap,
                            const Ligne& L,
                            const Ligne& bestLigne,
                            double bestScore) const;

    // 下一次落点变化罚分，用于阈值 (bestScore - nextPenalty)
    double nextChangePenalty() const;
};

#endif // LIGNEFINDER_H
