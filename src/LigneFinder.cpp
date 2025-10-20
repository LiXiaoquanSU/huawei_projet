#include "LigneFinder.h"
#include <queue>
#include <map>
#include <limits>
#include <iostream>
#include <iomanip>
#include <sstream>

// ====== 日志开关（需要静默时改为 false 即可，不影响逻辑）======
static constexpr bool LF_DEBUG = false;

// 小工具：把 path 序列打印成 "(x,y)->(x,y)..."
static std::string pathToStr(const std::vector<LigneFinder::XY>& path) {
    std::ostringstream oss;
    for (size_t i = 0; i < path.size(); ++i) {
        oss << "(" << path[i].first << "," << path[i].second << ")";
        if (i + 1 < path.size()) oss << "->";
    }
    return oss.str();
}

// 小工具：把 Ligne 内部 pathXY 打印
static std::string lignePathToStr(const Ligne& L) {
    std::ostringstream oss;
    for (size_t i = 0; i < L.pathXY.size(); ++i) {
        oss << "(" << L.pathXY[i].first << "," << L.pathXY[i].second << ")";
        if (i + 1 < L.pathXY.size()) oss << "->";
    }
    return oss.str();
}

// 取 (x,y) 的临时可用带宽
double LigneFinder::bwAt(int x, int y) const {
    auto it = bw_.find({x,y});
    double val = (it == bw_.end() ? 0.0 : it->second);
    if (LF_DEBUG) {
        std::cout << "[bwAt] (" << x << "," << y << ") -> " << val << " Mbps\n";
    }
    return val;
}

// 仅上下左右
std::vector<LigneFinder::XY> LigneFinder::neighbors4(int x, int y) const {
    std::vector<XY> res;
    if (inGrid(x+1,y)) res.emplace_back(x+1,y);
    if (inGrid(x-1,y)) res.emplace_back(x-1,y);
    if (inGrid(x,y+1)) res.emplace_back(x,y+1);
    if (inGrid(x,y-1)) res.emplace_back(x,y-1);
    if (LF_DEBUG) {
        std::cout << "[neighbors4] from (" << x << "," << y << ") -> ";
        for (auto& p : res) std::cout << "(" << p.first << "," << p.second << ") ";
        std::cout << "\n";
    }
    return res;
}

bool LigneFinder::inGrid(int x, int y) const {
    bool ok = (x >= 0 && x < network_.M && y >= 0 && y < network_.N);
    if (LF_DEBUG) {
        std::cout << "[inGrid] (" << x << "," << y << ") -> " << (ok ? "OK" : "OUT") << "\n";
    }
    return ok;
}

// ============ 工具：第 k 次变化的扣分 ============
double LigneFinder::deltaPenaltyForK(int k) const {
    if (k <= 1) return 0.0;
    double v = 10.0 * ( (1.0 / (k - 1)) - (1.0 / k) );  // 与你图中一致
    if (LF_DEBUG) {
        std::cout << "[deltaPenaltyForK] k=" << k << " -> " << v << "\n";
    }
    return v;
}

// ============ 工具：对“已落地”的路径施加奖惩 ============
void LigneFinder::applyLandingAdjustment(Ligne& L) const {
    if (L.pathXY.empty()) return;

    if (LF_DEBUG) {
        auto [ex, ey] = L.pathXY.back();
        std::cout << "[applyLandingAdjustment] before score=" << L.score
                  << "  end=(" << ex << "," << ey << ")  lastLanding=("
                  << lastLanding_.first << "," << lastLanding_.second << ")"
                  << "  changeCount=" << landingChangeCount_ << "\n";
    }

    // 如果落点与上次不同：扣“第 (changeCount+1) 次变化”的分
    auto [ex, ey] = L.pathXY.back();
    if (!(ex == lastLanding_.first && ey == lastLanding_.second)) {
        int k = landingChangeCount_ + 1;   // 本次就是第 k 次变化
        double d = deltaPenaltyForK(k);
        L.score -= d;
        if (LF_DEBUG) {
            std::cout << "  -> landing changed: -" << d << ", score=" << L.score << "\n";
        }
    } else {
        if (LF_DEBUG) {
            std::cout << "  -> same landing: no change\n";
        }
    }
}

// ============ 工具：由当前 best 计算动态阈值 ============
double LigneFinder::computeThresholdFromBest(const Ligne& best) const {
    if (best.pathXY.empty()) {
        if (LF_DEBUG) {
            std::cout << "[computeThresholdFromBest] best has empty path -> -inf\n";
        }
        return -std::numeric_limits<double>::infinity();
    }

    auto [bx, by] = best.pathXY.back();
    int nextK;

    if (lastLanding_.first == -1 && lastLanding_.second == -1) {
        nextK = landingChangeCount_ + 2;
    } else {
        bool bestChanged = !(bx == lastLanding_.first && by == lastLanding_.second);
        nextK = landingChangeCount_ + (bestChanged ? 2 : 1);
    }
    double thr = best.score - deltaPenaltyForK(nextK);
    if (LF_DEBUG) {
        std::cout << "[computeThresholdFromBest] bestScore=" << best.score
                  << " nextK=" << nextK << " -> threshold=" << thr << "\n";
    }
    return thr;
}

// ============ 单次 A*：一次性产出“已筛选的候选集” ============
std::vector<Ligne> LigneFinder::runAStarOnce(const std::set<XY>& banSet) const {
    if (LF_DEBUG) {
        std::cout << "\n========== [runAStarOnce] START ==========\n";
        std::cout << " Flow#" << flow_.id
                  << " t=" << t_
                  << " start=(" << flow_.x << "," << flow_.y << ")"
                  << " landingRect=[(" << flow_.m1 << "," << flow_.n1 << ")-("
                  << flow_.m2 << "," << flow_.n2 << ")]"
                  << " lastLanding=(" << lastLanding_.first << "," << lastLanding_.second << ")"
                  << " changeCount=" << landingChangeCount_ << "\n";
        std::cout << " Ban size=" << banSet.size() << "\n";
    }

    std::vector<Ligne> candidates;                  // 结果：候选集合（已筛过）
    std::map<XY, std::vector<Ligne>> cmap;         // 落点 -> 该落点候选
    Ligne bestLigne;                                // 当前最佳
    double bestScore = -std::numeric_limits<double>::infinity();
    double threshold = -std::numeric_limits<double>::infinity();

    auto inBan = [&](int x,int y){
        if (flow_.inLandingRange(x,y)) return false; // 落区不受 ban
        bool banned = (banSet.count({x,y}) > 0);
        if (LF_DEBUG && banned) {
            std::cout << "  [ban] (" << x << "," << y << ") is banned, skip\n";
        }
        return banned;
    };

    const int sx = flow_.x, sy = flow_.y;
    if (t_ < flow_.startTime) {
        if (LF_DEBUG) {
            std::cout << "  [early-exit] t_=" << t_ << " < startTime="
                      << flow_.startTime << ", return empty\n";
            std::cout << "========== [runAStarOnce] END (0 candidates) ==========\n";
        }
        return candidates;
    }

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
    if (LF_DEBUG) {
        std::cout << "  [init] start=(" << sx << "," << sy << ") bw_start=" << bw_start << "\n";
    }
    if (bw_start <= 0.0) {
        if (LF_DEBUG) {
            std::cout << "  [early-exit] start bw <= 0, return empty\n";
            std::cout << "========== [runAStarOnce] END (0 candidates) ==========\n";
        }
        return candidates;  // 如需允许从 0 带宽起步，可放宽此处
    }

    if (L0.addPathUav(sx, sy, bw_start) < 0) {
        if (LF_DEBUG) {
            std::cout << "  [init] addPathUav(start) failed, return empty\n";
            std::cout << "========== [runAStarOnce] END (0 candidates) ==========\n";
        }
        return candidates;
    }
    if (LF_DEBUG) {
        std::cout << "  [push-open] L0 path=" << lignePathToStr(L0)
                  << " q=" << L0.q << " dist=" << L0.distance
                  << " score=" << L0.score << " landed=" << (L0.landed?"Y":"N") << "\n";
    }
    open.push(L0);

    // ---------- 访问剪枝：到达坐标的最高已见 score ----------
    auto key64 = [](int x,int y)->long long { return ( (long long)y << 32 ) | (unsigned int)x; };
    std::map<long long, double> bestSeenScore;
    {
        auto [ex,ey] = L0.pathXY.back();
        bestSeenScore[key64(ex,ey)] = L0.score;
        if (LF_DEBUG) {
            std::cout << "  [bestSeenScore] set (" << ex << "," << ey << ")=" << L0.score << "\n";
        }
    }

    while (!open.empty()) {
        Ligne cur = open.top(); open.pop();
        if (LF_DEBUG) {
            std::cout << "\n  [pop-open] path=" << lignePathToStr(cur)
                      << " q=" << cur.q << " dist=" << cur.distance
                      << " score=" << cur.score
                      << " landed=" << (cur.landed?"Y":"N")
                      << " threshold=" << threshold << "\n";
        }

        // 全局阈值剪枝（队列有序，堆顶都小于阈值则直接收工）
        if (cur.score < threshold) {
            if (LF_DEBUG) {
        std::cout << "    [prune] cur.score=" << cur.score
                  << " < threshold=" << threshold << " -> skip\n";
    }
    continue; // 仅跳过当前分支，继续其他分支
        }

        // 已落地：施加“落点历史奖惩”，并按规则尝试加入候选
        if (cur.landed && !cur.pathXY.empty()) {
            if (LF_DEBUG) {
                auto [lx,ly] = cur.pathXY.back();
                std::cout << "    [landed] at (" << lx << "," << ly << "), apply adjustment\n";
            }
            Ligne landed = cur;         // 拷贝，避免污染 cur（如果你想继续扩展未必要扩）
            applyLandingAdjustment(landed);

            // 刷新最佳 → 直接加入、重算阈值
            if (landed.score > bestScore) {
                bestScore = landed.score;
                bestLigne = landed;
                auto key = landed.pathXY.back();
                cmap[key].push_back(landed);

                double newThr = computeThresholdFromBest(bestLigne);
                if (LF_DEBUG) {
                    std::cout << "    [best-update] new bestScore=" << bestScore
                              << "  threshold=" << newThr << "\n";
                }
                threshold = newThr;
            } else if (landed.score >= threshold) {
    // 非最佳：双条件（分数≥阈值 + 同落点更短才加入）
    auto key = landed.pathXY.back();
    auto& vec = cmap[key];

    if (!vec.empty()) {
        int minDist = static_cast<int>(vec.front().distance);
        for (auto& c : vec) {
            minDist = std::min(minDist, static_cast<int>(c.distance));
        }

        // 只在“严格更短”时加入；相同或更长一律不加
        if (static_cast<int>(landed.distance) < minDist) {
            vec.push_back(landed);
            if (LF_DEBUG) {
                std::cout << "    [candidate-keep] score>=threshold & strictly-shorter"
                          << "  end=(" << key.first << "," << key.second << ")"
                          << "  dist=" << landed.distance << "  kept\n";
            }
        } else {
            if (LF_DEBUG) {
                std::cout << "    [candidate-skip] not strictly shorter at same end"
                          << "  end=(" << key.first << "," << key.second << ")"
                          << "  dist=" << landed.distance
                          << "  minDist=" << minDist << "  skipped\n";
            }
        }
    } else {
        // 同落点尚无记录，直接加入
        vec.push_back(landed);
        if (LF_DEBUG) {
            std::cout << "    [candidate-new] end=(" << key.first << "," << key.second
                      << ") added\n";
        }
    }
}

            // 不再从已落地节点继续扩展（避免产生环和冗余）
            continue;
        }

        // 末端坐标
        auto [cx, cy] = cur.pathXY.back();
        if (LF_DEBUG) {
            std::cout << "    [expand] from (" << cx << "," << cy << ")\n";
        }

        // 扩展四邻居
        for (auto [nx, ny] : neighbors4(cx, cy)) {
            if (inBan(nx, ny)) {
                if (LF_DEBUG) {
                    std::cout << "      [skip] (" << nx << "," << ny << ") in banSet\n";
                }
                continue;
            }

            double bw_xy = bwAt(nx, ny);
            if (bw_xy <= 0.0) {
                if (LF_DEBUG) {
                    std::cout << "      [skip] (" << nx << "," << ny << ") bw<=0\n";
                }
                continue;
            }

            Ligne nxt = cur;
            double rc = nxt.addPathUav(nx, ny, bw_xy, flow_.x, flow_.y, flow_.m1, flow_.n1, flow_.m2, flow_.n2, 0.1);
            if (rc < 0) {
                if (LF_DEBUG) {
                    std::cout << "      [skip] addPathUav(" << nx << "," << ny << ") failed\n";
                }
                continue;
            }

            // 阈值剪枝（无论落没落地）
            if (nxt.score < threshold) {
                if (LF_DEBUG) {
                    std::cout << "      [skip] nxt.score=" << nxt.score
                              << " < threshold=" << threshold << "\n";
                }
                continue;
            }

            // // 访问剪枝：同坐标若已有更高估值则跳过
            // long long k = key64(nx, ny);
            // auto it = bestSeenScore.find(k);
            // if (it != bestSeenScore.end() && nxt.score < it->second) {
            //     if (LF_DEBUG) {
            //         std::cout << "      [skip] bestSeenScore[(" << nx << "," << ny
            //                   << ")=" << it->second << "] >= nxt.score=" << nxt.score << "\n";
            //     }
            //     continue;
            // }
            // bestSeenScore[k] = nxt.score;

            if (LF_DEBUG) {
                std::cout << "      [push-open] path=" << lignePathToStr(nxt)
                          << " q=" << nxt.q << " dist=" << nxt.distance
                          << " score=" << nxt.score
                          << " landed=" << (nxt.landed?"Y":"N") << "\n";
            }
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

    if (LF_DEBUG) {
        std::cout << "\n========== [runAStarOnce] RESULT ==========\n";
        if (candidates.empty()) {
            std::cout << "  (no candidates)\n";
        } else {
            std::cout << "  total candidates: " << candidates.size() << "\n";
            for (size_t i = 0; i < candidates.size(); ++i) {
                const auto& L = candidates[i];
                auto [ex,ey] = L.pathXY.empty() ? std::make_pair(-1,-1) : L.pathXY.back();
                std::cout << "  #" << (i+1)
                          << " score=" << L.score
                          << " q=" << L.q
                          << " dist=" << L.distance
                          << " end=(" << ex << "," << ey << ")"
                          << " path=" << lignePathToStr(L) << "\n";
            }
        }
        std::cout << "========== [runAStarOnce] END ==========\n";
    }

    return candidates;
}