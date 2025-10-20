#include "LigneFinder.h"
#include <queue>
#include <map>
#include <limits>

// 起点/网格/邻居工具函数同之前

// 取 (x,y) 的临时可用带宽
double LigneFinder::bwAt(int x, int y) const {
    auto it = bw_.find({x,y});
    return (it == bw_.end() ? 0.0 : it->second);
}

// 仅上下左右
std::vector<LigneFinder::XY> LigneFinder::neighbors4(int x, int y) const {
    std::vector<XY> res;
    if (inGrid(x+1,y)) res.emplace_back(x+1,y);
    if (inGrid(x-1,y)) res.emplace_back(x-1,y);
    if (inGrid(x,y+1)) res.emplace_back(x,y+1);
    if (inGrid(x,y-1)) res.emplace_back(x,y-1);
    return res;
}

bool LigneFinder::inGrid(int x, int y) const {
    return x >= 0 && x < network_.M && y >= 0 && y < network_.N;
}

// ============ 工具：第 k 次变化的扣分 ============
double LigneFinder::deltaPenaltyForK(int k) const {
    if (k <= 1) return 0.0;
    return 10.0 * ( (1.0 / (k - 1)) - (1.0 / k) );  // 与你图中一致
}

// ============ 工具：对“已落地”的路径施加奖惩 ============
void LigneFinder::applyLandingAdjustment(Ligne& L) const {
    if (L.pathXY.empty()) return;
    // 首次落地：+10
    if (lastLanding_.first == -1 && lastLanding_.second == -1) {
        L.score += 10.0;
        return;
    }
    // 如果落点与上次不同：扣“第 (changeCount+1) 次变化”的分
    auto [ex, ey] = L.pathXY.back();
    if (!(ex == lastLanding_.first && ey == lastLanding_.second)) {
        int k = landingChangeCount_ + 1;   // 本次就是第 k 次变化
        L.score -= deltaPenaltyForK(k);
    }
    // 相同落点：不加不扣
}

// ============ 工具：由当前 best 计算动态阈值 ============
double LigneFinder::computeThresholdFromBest(const Ligne& best) const {
    if (best.pathXY.empty()) return -std::numeric_limits<double>::infinity();

    auto [bx, by] = best.pathXY.back();
    int nextK;

    if (lastLanding_.first == -1 && lastLanding_.second == -1) {
        // 首次落地，下一次变化将是第 (changeCount+1) 次（通常 k=1 → Δ=0）
        nextK = landingChangeCount_ + 1;
    } else {
        // 若 best 与 lastLanding 相同 -> 下一次变化是 (changeCount+1)
        // 若 best 与 lastLanding 不同 -> 本次已发生一次变化，下一次是 (changeCount+2)
        bool bestChanged = !(bx == lastLanding_.first && by == lastLanding_.second);
        nextK = landingChangeCount_ + (bestChanged ? 2 : 1);
    }
    return best.score - deltaPenaltyForK(nextK);
}

// ============ 单次 A*：一次性产出“已筛选的候选集” ============
std::vector<Ligne> LigneFinder::runAStarOnce(const std::set<XY>& banSet) const {
    std::vector<Ligne> candidates;                  // 结果：候选集合（已筛过）
    std::map<XY, std::vector<Ligne>> cmap;         // 落点 -> 该落点候选
    Ligne bestLigne;                                // 当前最佳
    double bestScore = -std::numeric_limits<double>::infinity();
    double threshold = -std::numeric_limits<double>::infinity();

    auto inBan = [&](int x,int y){
        if (flow_.inLandingRange(x,y)) return false; // 落区不受 ban
        return banSet.count({x,y}) > 0;
    };

    const int sx = flow_.x, sy = flow_.y;
    if (t_ < flow_.startTime) return candidates;

    // ---------- 开放集：score 高优先 ----------
    std::priority_queue<Ligne> open;

    // ---------- 初始化首条 Ligne ----------
    Ligne L0;
    L0.flowId  = flow_.id;
    L0.t       = t_;
    L0.t_start = flow_.startTime;
    L0.Q_total = flow_.size;
    //L0.setScoringContext(flow_.x, flow_.y, flow_.m1, flow_.n1, flow_.m2, flow_.n2, 0.1);

    double bw_start = bwAt(sx, sy);
    if (bw_start <= 0.0) return candidates;  // 如需允许从 0 带宽起步，可放宽此处

    if (L0.addPathUav(sx, sy, bw_start) < 0) return candidates;
    open.push(L0);

    // ---------- 访问剪枝：到达坐标的最高已见 score ----------
    auto key64 = [](int x,int y)->long long { return ( (long long)y << 32 ) | (unsigned int)x; };
    std::map<long long, double> bestSeenScore;
    {
        auto [ex,ey] = L0.pathXY.back();
        bestSeenScore[key64(ex,ey)] = L0.score;
    }

    while (!open.empty()) {
        Ligne cur = open.top(); open.pop();

        // 全局阈值剪枝（队列有序，堆顶都小于阈值则直接收工）
        if (cur.score < threshold) break;

        // 已落地：施加“落点历史奖惩”，并按规则尝试加入候选
        if (cur.landed && !cur.pathXY.empty()) {
            Ligne landed = cur;         // 拷贝，避免污染 cur（如果你想继续扩展未必要扩）
            applyLandingAdjustment(landed);

            // 刷新最佳 → 直接加入、重算阈值
            if (landed.score > bestScore) {
                bestScore = landed.score;
                bestLigne = landed;
                auto key = landed.pathXY.back();
                cmap[key].push_back(landed);

                threshold = computeThresholdFromBest(bestLigne);
            } else if (landed.score >= threshold) {
                // 非最佳：双条件（分数≥阈值 + 同落点更短）
                auto key = landed.pathXY.back();
                auto& vec = cmap[key];
                if (!vec.empty()) {
                    int minDist = (int)vec.front().distance;
                    for (auto& c : vec) minDist = std::min(minDist, (int)c.distance);
                    if ((int)landed.distance <= minDist) vec.push_back(landed);
                } else {
                    vec.push_back(landed);
                }
            }
            // 不再从已落地节点继续扩展（避免产生环和冗余）
            continue;
        }

        // 末端坐标
        auto [cx, cy] = cur.pathXY.back();

        // 扩展四邻居
        for (auto [nx, ny] : neighbors4(cx, cy)) {
            if (inBan(nx, ny)) continue;

            double bw_xy = bwAt(nx, ny);
            if (bw_xy <= 0.0) continue;

            Ligne nxt = cur;
            if (nxt.addPathUav(nx, ny, bw_xy) < 0) continue;

            // 阈值剪枝（无论落没落地）
            if (nxt.score < threshold) continue;

            // 访问剪枝：同坐标若已有更高估值则跳过
            long long k = key64(nx, ny);
            auto it = bestSeenScore.find(k);
            if (it != bestSeenScore.end() && nxt.score <= it->second) continue;
            bestSeenScore[k] = nxt.score;

            open.push(nxt);
        }
    }

    // 展平 cmap 为结果
    for (auto& [end, vec] : cmap) {
        candidates.insert(candidates.end(), vec.begin(), vec.end());
    }
    // 可选：按得分降序
    std::sort(candidates.begin(), candidates.end(),
              [](const Ligne& a, const Ligne& b){ return a.score > b.score; });

    return candidates;
}

