#include "CubeOptimizer.h"
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <set>
#include <sstream>

// ====== 调试开关 ======
static constexpr bool OPT_DEBUG  = true;   // 逻辑级 Debug
static constexpr bool OPT_VISUAL = true;   // 可视化网格

// 为了对齐输出
static inline std::string fmt2(double v, int w=6, int p=2) {
    std::ostringstream oss;
    oss << std::setw(w) << std::fixed << std::setprecision(p) << v;
    return oss.str();
}

CubeOptimizer::CubeOptimizer(const Network& net, const Cube& inputCube)
    : network_(net), cube_(inputCube) {
    // 保障 cube_ 含有 0..T-1 的切片槽位，避免后续 t_high/t_low 超界
    if ((int)cube_.slices.size() < network_.T) {
        cube_.slices.resize(network_.T);
    }
    for (int t = 0; t < network_.T; ++t) {
        cube_.slices[t].t = t;
    }

    // 构建初始 C 表，用作效率排序的基线
    buildConfirmedTable();
    baselineConfirmedTable_ = confirmedTable_;
}

static inline double getFlowTotalSize(const Network& net, int fid) {
    for (const auto& f : net.flows) if (f.id == fid) return f.size;
    return 0.0;
}

/* ------------------- 可视化工具 ------------------- */
void CubeOptimizer::visualPrintCube(const std::string& title) const {
    if (!OPT_VISUAL) return;
    std::cout << "\n==== [Cube Visual] " << title << " ====\n";
    for (int t = 0; t < (int)cube_.slices.size(); ++t) {
        const auto& s = cube_.slices[t];
        std::cout << "t=" << t << " | ";
        std::map<int,double> flowQ;
        for (const auto& L : s.lignes) flowQ[L.flowId] += L.q;
        for (const auto& f : network_.flows) {
            double q = flowQ.count(f.id) ? flowQ.at(f.id) : 0.0;
            std::cout << "F" << f.id << ":" << fmt2(q) << "  ";
        }
        std::cout << "\n";
    }
}

void CubeOptimizer::visualPrintTableC() const {
    if (!OPT_VISUAL) return;
    std::cout << "\n---- [C-table Visual] 确定表 (q/eff) ----\n";
    // 打印列头
    std::cout << "      ";
    for (int t = 0; t < network_.T; ++t) std::cout << "  t" << std::setw(2) << t << "      ";
    std::cout << "\n";
    for (const auto& f : network_.flows) {
        int fid = f.id;
        std::cout << "F" << fid << " | ";
        for (int t = 0; t < network_.T; ++t) {
            auto itR = confirmedTable_.find(fid);
            double q=0, eff=0;
            if (itR != confirmedTable_.end()) {
                auto it = itR->second.find(t);
                if (it != itR->second.end() && it->second.valid) {
                    q = it->second.q; eff = it->second.eff;
                }
            }
            std::cout << fmt2(q) << "/" << fmt2(eff,5,3) << "  ";
        }
        std::cout << "\n";
    }
}

void CubeOptimizer::visualPrintTableP() const {
    if (!OPT_VISUAL) return;
    std::cout << "\n---- [P-table Visual] 潜力表 (q/eff) ----\n";
    std::cout << "      ";
    for (int t = 0; t < network_.T; ++t) std::cout << "  t" << std::setw(2) << t << "      ";
    std::cout << "\n";
    for (const auto& f : network_.flows) {
        int fid = f.id;
        std::cout << "F" << fid << " | ";
        for (int t = 0; t < network_.T; ++t) {
            double q=0, eff=0;
            auto itR = potentialTable_.find(fid);
            if (itR != potentialTable_.end()) {
                auto it = itR->second.find(t);
                if (it != itR->second.end() && it->second.valid) {
                    q = it->second.q; eff = it->second.eff;
                }
            }
            std::cout << fmt2(q) << "/" << fmt2(eff,5,3) << "  ";
        }
        std::cout << "\n";
    }
}

/* ------------------- 主流程 ------------------- */
Cube CubeOptimizer::optimize() {
    if (OPT_DEBUG) std::cout << "\n=== ⚙️ CubeOptimizer 启动 ===\n";

    buildConfirmedTable();
    if (OPT_DEBUG) logTableSummary("Initial Confirmed Table", confirmedTable_);
    visualPrintCube("Initial Cube");
    visualPrintTableC();

    int iter = 0;
    while (true) {
        ++iter;
        if (iter > 50) {
            std::cout << "⚠️ 达到最大迭代次数，强制结束优化。\n";
            break;
        }

        if (OPT_DEBUG) std::cout << "\n--- 🌀 优化迭代 #" << iter << " ---\n";

        buildPotentialTable();
        if (OPT_DEBUG) logTableSummary("Potential Table", potentialTable_);
        visualPrintTableP();

        auto [fid, t_high, gap, hasCapacity] = findMaxEfficiencyGap();
        (void)hasCapacity;
        if (fid == -1 || gap < 1e-6) {
            if (OPT_DEBUG) std::cout << "\n✅ 潜力效率不再高于当前效率，优化结束。\n";
            break;
        }

        // 找 C 表中最低效率时刻（有流量）
        int t_low = -1;
        double eff_min = 1e18;
        auto itRow = confirmedTable_.find(fid);
        if (itRow != confirmedTable_.end()) {
            for (const auto& [t, cell] : itRow->second) {
                if (cell.valid && cell.q > 1e-9 && cell.eff < eff_min) {
                    eff_min = cell.eff;
                    t_low = t;
                }
            }
        }
        if (t_low == -1) {
            if (OPT_DEBUG) std::cout << "⚠️ Flow#" << fid << " 无可释放的低效时刻，跳过。\n";
            break;
        }

        if (OPT_DEBUG) {
            std::cout << "🔍 发现改进：Flow#" << fid
                      << " t_high=" << t_high << " (潜力高)  t_low=" << t_low << " (当前低)"
                      << " gap=" << gap << "\n";
        }

        // 前后可视化
        visualPrintCube("Before Rebalance");
        visualPrintTableC();

        rebalanceFlow(fid, t_high, t_low);

        // 重新构建确定表和潜力表
        buildConfirmedTable();
        visualPrintCube("After Rebalance");
        visualPrintTableC();

        buildPotentialTable();
        if (OPT_DEBUG) logTableSummary("Potential Table (post-rebalance)", potentialTable_);
        visualPrintTableP();
    }

    if (OPT_DEBUG) std::cout << "\n=== ✅ CubeOptimizer 完成 ===\n";
    return cube_;
}

/* ------------------- 构建确定表 ------------------- */
void CubeOptimizer::buildConfirmedTable() {
    confirmedTable_.clear();

    for (const auto& slice : cube_.slices) {
        int t = slice.t;
        for (const auto& L : slice.lignes) {
            auto& cell = confirmedTable_[L.flowId][t];
            cell.q     = L.q;
            cell.score = L.score;
            cell.eff   = (L.q > 1e-9 ? L.score / L.q : 0.0);
            cell.valid = true;
            cell.endXY = L.pathXY.empty() ? std::make_pair(-1,-1)
                                          : L.pathXY.back();
        }
    }
}

/* ------------------- 构建潜力表 ------------------- */
void CubeOptimizer::buildPotentialTable() {
    potentialTable_.clear();
    constexpr double EPS = CubeOptimizer::EPS;

    for (const auto& flow : network_.flows) {
        int fid = flow.id;
        const double totalQ = getFlowTotalSize(network_, fid);
        if (totalQ <= EPS) continue;

        for (int t = 0; t < network_.T; ++t) {
            auto bw = makeMaskedBwForPotential(fid, t);
            auto lastXY = getLastLanding(fid, t);
            auto nextXY = getNextLanding(fid, t);
            int  kCount = getLandingChangeCountGlobal(fid);
            int  nState = getNeighborState(fid, t);
            double rem = totalQ;  // 潜力表用总量（你指定的规则）

            LigneFinder finder(network_, flow, t, bw,
                               lastXY, nextXY,
                               kCount, nState,
                               rem);

            auto lignes = finder.runAStarOnce();
            if (lignes.empty()) continue;

            const Ligne* best = nullptr;
            double bestEff = -1e18;
            for (const auto& L : lignes) {
                if (L.q <= EPS) continue;
                double eff = L.score / L.q;
                if (eff > bestEff) {
                    bestEff = eff;
                    best = &L;
                }
            }
            if (!best) continue;

            double qC = 0.0;
            auto itRowC = confirmedTable_.find(fid);
            if (itRowC != confirmedTable_.end()) {
                auto itC = itRowC->second.find(t);
                if (itC != itRowC->second.end() && itC->second.valid)
                    qC = itC->second.q;
            }

            auto& cell = potentialTable_[fid][t];
            cell.q     = best->q;
            cell.score = best->score;
            cell.eff   = bestEff;
            cell.valid = true;
            cell.endXY = best->pathXY.empty() ? std::make_pair(-1,-1)
                                              : best->pathXY.back();

            if (OPT_DEBUG) {
                std::cout << "[P-add] Flow#" << fid
                          << " t=" << t
                          << " q=" << cell.q
                          << " score=" << cell.score
                          << " eff=" << cell.eff << "\n";
            }
        }
    }
}

/* ------------------- 邻接状态/上下文 ------------------- */
std::pair<int,int> CubeOptimizer::getLastLanding(int fid, int t) const {
    if (t <= 0) return {-1,-1};
    auto itRow = confirmedTable_.find(fid);
    if (itRow == confirmedTable_.end()) return {-1,-1};
    auto it = itRow->second.find(t-1);
    if (it == itRow->second.end()) return {-1,-1};
    return it->second.valid ? it->second.endXY : std::make_pair(-1,-1);
}

std::pair<int,int> CubeOptimizer::getNextLanding(int fid, int t) const {
    if (t >= network_.T-1) return {-1,-1};
    auto itRow = confirmedTable_.find(fid);
    if (itRow == confirmedTable_.end()) return {-1,-1};
    auto it = itRow->second.find(t+1);
    if (it == itRow->second.end()) return {-1,-1};
    return it->second.valid ? it->second.endXY : std::make_pair(-1,-1);
}

int CubeOptimizer::getLandingChangeCountGlobal(int fid) const {
    auto itRow = confirmedTable_.find(fid);
    if (itRow == confirmedTable_.end()) return 0;

    int k = 0;
    std::pair<int,int> prev{-1,-1};
    bool hasPrev = false;

    for (const auto& [t, cell] : itRow->second) {
        if (!cell.valid || cell.q <= 1e-9) continue;
        if (!hasPrev) {
            prev = cell.endXY;
            hasPrev = true;
            continue;
        }
        if (cell.endXY != prev) {
            ++k;
            prev = cell.endXY;
        }
    }
    return k;
}

int CubeOptimizer::getNeighborState(int fid, int t) const {
    bool leftEdge  = (t == 0);
    bool rightEdge = (t == network_.T - 1);
    auto itRow = confirmedTable_.find(fid);
    bool hasL = false, hasR = false;
    if (!leftEdge && itRow != confirmedTable_.end()) {
        auto itL = itRow->second.find(t-1);
        hasL = (itL != itRow->second.end() && itL->second.valid && itL->second.q > 1e-9);
    }
    if (!rightEdge && itRow != confirmedTable_.end()) {
        auto itR = itRow->second.find(t+1);
        hasR = (itR != itRow->second.end() && itR->second.valid && itR->second.q > 1e-9);
    }
    if ((hasL || leftEdge) && (hasR || rightEdge)) return 0;
    if ((hasL || leftEdge) ^ (hasR || rightEdge)) return 1;
    return 2;
}

/* ------------------- 扣减其它流占用 ------------------- */
std::map<std::pair<int,int>, double>
CubeOptimizer::makeMaskedBwForPotential(int fid, int t) const {
    std::map<std::pair<int,int>, double> bw;
    for (const auto& u : network_.uavs)
        bw[{u.x, u.y}] = u.bandwidthAt(t);

    // ✅ 只扣除其它流
    if (t >= 0 && t < (int)cube_.slices.size()) {
        const auto& slice = cube_.slices[t];
        for (const auto& L : slice.lignes) {
            if (L.flowId == fid) continue; // 不扣本流
            for (const auto& xy : L.pathXY) {
                auto& b = bw[xy];
                b = std::max(0.0, b - L.q);
            }
        }
    }
    return bw;
}


/* ------------------- 寻找最大 gap ------------------- */
std::tuple<int,int,double,bool> CubeOptimizer::findMaxEfficiencyGap() const {
    constexpr double EPS = CubeOptimizer::EPS;
    int bestF = -1, bestT = -1;
    double bestGap = 0.0;
    bool bestHasCapacity = false;

    for (const auto& flow : network_.flows) {
        int fid = flow.id;

        auto itCurrent = confirmedTable_.find(fid);
        if (itCurrent == confirmedTable_.end()) continue;
        auto itBase = baselineConfirmedTable_.find(fid);
        if (itBase == baselineConfirmedTable_.end()) continue;

        double effMinBase = 1e18;
        int tLow = -1;
        for (const auto& [t, baseCell] : itBase->second) {
            if (!baseCell.valid || baseCell.q <= EPS) continue;
            double qNow = 0.0;
            if (auto itCur = itCurrent->second.find(t);
                itCur != itCurrent->second.end() && itCur->second.valid)
                qNow = itCur->second.q;
            if (qNow <= EPS) continue; // 当前已经没有该时刻的流量
            if (baseCell.eff < effMinBase) {
                effMinBase = baseCell.eff;
                tLow = t;
            }
        }
        if (tLow == -1) continue;

        auto itP = potentialTable_.find(fid);
        if (itP == potentialTable_.end()) continue;

        for (const auto& [t, pcell] : itP->second) {
            if (!pcell.valid) continue;

            double qCurrent = 0.0;
            if (auto itCell = itCurrent->second.find(t);
                itCell != itCurrent->second.end() && itCell->second.valid)
                qCurrent = itCell->second.q;

            double capacityGain = pcell.q - qCurrent;
            double gap = pcell.eff - effMinBase;
            if (capacityGain > EPS) {
                if (gap > bestGap + 1e-9 ||
                    (gap > bestGap - 1e-9 && !bestHasCapacity)) {
                    bestGap = gap;
                    bestF = fid;
                    bestT = t;
                    bestHasCapacity = true;
                }
            } else if (t == tLow && qCurrent > EPS && gap > 1e-6 && !bestHasCapacity) {
                // 仅当当前没有“可扩容”候选时，才考虑同一时刻的纯替换
                if (gap > bestGap + 1e-9) {
                    bestGap = gap;
                    bestF = fid;
                    bestT = t;
                    bestHasCapacity = false;
                }
            }
        }
    }

    if (bestF == -1 || bestGap <= 1e-6)
        return {-1, -1, 0.0, false};

    return {bestF, bestT, bestGap, bestHasCapacity};
}

/* ------------------- 重分配（含兜底替换） ------------------- */
void CubeOptimizer::rebalanceFlow(int fid, int t_high, int t_low) {
    constexpr double EPS = CubeOptimizer::EPS;

    if (OPT_DEBUG)
        std::cout << "  ⚙️ Rebalancing Flow#" << fid
                  << " between t_low=" << t_low << " and t_high=" << t_high << "\n";

    // Step 1: 计算 Δq
    double q_low = 0.0;
    double q_high_current = 0.0;
    double eff_high_current = 0.0;
    double q_high_potential = 0.0;
    double eff_high_potential = 0.0;

    if (auto itC = confirmedTable_.find(fid); itC != confirmedTable_.end()) {
        if (auto it = itC->second.find(t_low); it != itC->second.end())
            q_low = it->second.q;
        if (auto it2= itC->second.find(t_high); it2!= itC->second.end()) {
            q_high_current = it2->second.q;
            eff_high_current = it2->second.eff;
        }
    }
    if (auto itP = potentialTable_.find(fid); itP != potentialTable_.end()) {
        if (auto it = itP->second.find(t_high); it != itP->second.end()) {
            q_high_potential = it->second.q;
            eff_high_potential = it->second.eff;
        }
    }

    double capacityGain = std::max(0.0, q_high_potential - q_high_current);
    double deltaQ = std::min(q_low, capacityGain);
    if (deltaQ <= EPS) {
        if (t_low == t_high && q_high_current > EPS &&
            eff_high_potential > eff_high_current + 1e-6) {
            if (OPT_DEBUG) {
                std::cout << "  🔁 Δq≈0，但尝试在 t=" << t_high
                          << " 内直接替换为潜力最优路径 (q=" << q_high_current << ")\n";
            }
            if (applyDirectReplacement(fid, t_high, q_high_current)) {
                buildConfirmedTable();
                if (OPT_DEBUG) {
                    std::cout << "  ✅ Flow#" << fid << " 在 t=" << t_high
                              << " 已替换为潜力路径（保持同等流量）。\n";
                }
            } else if (OPT_DEBUG) {
                std::cout << "  ⚠️ 直接替换失败：潜力路径不足以覆盖当前流量。\n";
            }
        } else if (OPT_DEBUG) {
            std::cout << "  ⚠️ Δq≈0，跳过本次重排。\n";
        }
        return;
    }

    double targetHighQ = q_high_current + deltaQ;

    if (OPT_DEBUG) {
        std::cout << "  → Δq=" << deltaQ
                  << " (t_low=" << t_low << " q_low=" << q_low
                  << " → t_high=" << t_high << " q_high_now=" << q_high_current
                  << " q_high_pot=" << q_high_potential
                  << " targetHighQ=" << targetHighQ << ")\n";
    }

    auto reduceFlowOnSlice = [&](int t, double amount) {
        if (t < 0 || t >= (int)cube_.slices.size()) return;
        auto& sl = cube_.slices[t];
        double need = amount;
        for (auto it = sl.lignes.begin(); it != sl.lignes.end() && need > EPS; ) {
            if (it->flowId != fid) { ++it; continue; }
            double consume = std::min(it->q, need);
            double remainQ = it->q - consume;
            if (it->q > EPS) {
                double scale = remainQ / it->q;
                it->score *= std::max(0.0, scale);
            }
            it->q = remainQ;
            need -= consume;
            if (it->q <= EPS) it = sl.lignes.erase(it);
            else ++it;
        }
        if (need > EPS && OPT_DEBUG) {
            std::cout << "  ⚠️ t=" << t << " 未能完全释放 Δq，剩余=" << need << "\n";
        }
    };

    // Step 3: 从 t_low 移除 Δq
    reduceFlowOnSlice(t_low, deltaQ);

    // Step 4: 在 t_high 插入/替换为潜力路径
    bool increased = false;
    auto bestLineOpt = computeBestPotentialLigne(fid, t_high);
    if (bestLineOpt) {
        Ligne newL = *bestLineOpt;
        if (newL.q > targetHighQ) {
            double s = targetHighQ / std::max(newL.q, EPS);
            newL.q *= s;
            newL.score *= s;
        } else if (newL.q < targetHighQ - EPS) {
            double s = targetHighQ / std::max(newL.q, EPS);
            newL.q *= s;
            newL.score *= s;
        }

        auto& slHigh = cube_.slices[t_high];
        slHigh.lignes.erase(std::remove_if(slHigh.lignes.begin(), slHigh.lignes.end(),
                                           [&](const Ligne& L){ return L.flowId == fid; }),
                            slHigh.lignes.end());
        newL.flowId = fid;
        slHigh.lignes.push_back(newL);

        double qChk = 0.0;
        for (const auto& L : slHigh.lignes)
            if (L.flowId == fid) qChk += L.q;
        increased = (qChk >= targetHighQ - 1e-6);

        if (OPT_DEBUG) {
            std::cout << "  ✅ t_high=" << t_high << " 用潜力路径重建 fid="
                      << fid << " q=" << qChk
                      << " (target=" << targetHighQ << ")\n";
        }
    } else if (OPT_DEBUG) {
        std::cout << "  ⚠️ 未找到 t_high=" << t_high << " 的潜力路径，无法扩充。\n";
    }

    // Step 5: 更新确定表（供下一轮判断）
    buildConfirmedTable();

    if (OPT_DEBUG)
        std::cout << "  ✅ Flow#" << fid << " 流量已从 t=" << t_low
                  << " 转移至 t=" << t_high << " (Δq=" << deltaQ << ")\n";
}

/* ------------------- 调试打印 ------------------- */
void CubeOptimizer::logTableSummary(const std::string& name, const Table& tbl) const {
    std::cout << "\n--- [" << name << "] ---\n";
    for (const auto& [fid, row] : tbl) {
        std::cout << "Flow#" << fid << ":\n";
        for (const auto& [t, cell] : row) {
            if (!cell.valid) continue;
            std::cout << "  t=" << std::setw(2) << t
                      << " q=" << std::setw(6) << std::fixed << std::setprecision(2) << cell.q
                      << " score=" << std::setw(8) << std::setprecision(3) << cell.score
                      << " eff=" << std::setw(7) << std::setprecision(3) << cell.eff
                      << " end=(" << cell.endXY.first << "," << cell.endXY.second << ")\n";
        }
    }
}

const Flow* CubeOptimizer::findFlow(int fid) const {
    for (const auto& f : network_.flows)
        if (f.id == fid) return &f;
    return nullptr;
}

std::optional<Ligne> CubeOptimizer::computeBestPotentialLigne(int fid, int t) const {
    constexpr double EPS = CubeOptimizer::EPS;
    const Flow* flowPtr = findFlow(fid);
    if (!flowPtr) return std::nullopt;

    auto bw = makeMaskedBwForPotential(fid, t);
    auto lastXY = getLastLanding(fid, t);
    auto nextXY = getNextLanding(fid, t);
    int  kCount = getLandingChangeCountGlobal(fid);
    int  nState = getNeighborState(fid, t);
    double rem  = getFlowTotalSize(network_, fid);

    LigneFinder finder(network_, *flowPtr, t, bw,
                       lastXY, nextXY, kCount, nState, rem);
    auto lignes = finder.runAStarOnce();
    const Ligne* bestLine = nullptr;
    double bestEff = -1e18;
    for (const auto& L : lignes) {
        if (L.q <= EPS) continue;
        double eff = L.score / L.q;
        if (eff > bestEff) {
            bestEff = eff;
            bestLine = &L;
        }
    }
    if (!bestLine) return std::nullopt;
    return *bestLine;
}

bool CubeOptimizer::applyDirectReplacement(int fid, int t, double desiredQ) {
    constexpr double EPS = CubeOptimizer::EPS;
    if (desiredQ <= EPS) return false;
    if (t < 0 || t >= (int)cube_.slices.size()) return false;

    auto bestLineOpt = computeBestPotentialLigne(fid, t);
    if (!bestLineOpt) return false;

    Ligne newL = *bestLineOpt;
    if (newL.q + EPS < desiredQ) {
        return false; // 潜力路径无法承载当前流量
    }

    if (newL.q > desiredQ + EPS) {
        double scale = desiredQ / std::max(newL.q, EPS);
        newL.q *= scale;
        newL.score *= scale;
    } else if (std::abs(newL.q - desiredQ) > EPS) {
        // 对齐目标流量
        double scale = desiredQ / std::max(newL.q, EPS);
        newL.q *= scale;
        newL.score *= scale;
    }

    newL.flowId = fid;
    auto& sl = cube_.slices[t];
    sl.lignes.erase(std::remove_if(sl.lignes.begin(), sl.lignes.end(),
                                   [&](const Ligne& L){ return L.flowId == fid; }),
                    sl.lignes.end());
    sl.lignes.push_back(newL);
    return true;
}
