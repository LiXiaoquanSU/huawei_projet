#include "SlicePlanner.h"
#include <iostream>
#include <algorithm>
#include <iomanip>

// ⚙️ 调试输出总开关
#define DEBUG_SLICEPLANNER 0

using XY = std::pair<int,int>;

SlicePlanner::SlicePlanner(const Network& net,
    const std::map<int, double>& remaining,
    const std::map<int, XY>& lastLanding,
    const std::map<int, XY>& nextLanding,
    const std::map<int, int>& changeCount,
    const std::map<int, int>& neighborState,
    int t,
    const std::map<XY, double>& bw)
: network_(net),
remaining_(remaining),
lastLanding_(lastLanding),
nextLanding_(nextLanding), 
changeCount_(changeCount),
neighborState_(neighborState), 
t_(t),
bw_(bw)
{}


/**
 * @brief 主函数：生成所有可能的 Slice 组合
 */
std::vector<Slice> SlicePlanner::planAllSlices() {
#if DEBUG_SLICEPLANNER
    std::cout << "\n========== [SlicePlanner::planAllSlices] START ==========\n";
    std::cout << "t=" << t_ << " | total flows=" << remaining_.size() << "\n";
    for (auto& [fid, r] : remaining_) {
        auto itL = lastLanding_.find(fid);
        auto itC = changeCount_.find(fid);
        auto last = (itL != lastLanding_.end()) ? itL->second : XY{-1,-1};
        int cc = (itC != changeCount_.end()) ? itC->second : 0;
        std::cout << "  Flow#" << fid
                  << "  remaining=" << r
                  << "  lastLanding=(" << last.first << "," << last.second << ")"
                  << "  changeCount=" << cc << "\n";
    }
#endif

    std::vector<Slice> allSlices;

    // 依据当前剩余/带宽评估每条流，获取一组候选顺序
    auto baseOrders = computeFlowOrder();
    if (baseOrders.empty()) return allSlices;

    const auto& baseFlowOrder = baseOrders.front();
    if (baseFlowOrder.empty()) return allSlices;

    std::vector<std::vector<int>> flowOrders;
    if (baseFlowOrder.size() > 4) {
        flowOrders = baseOrders;
    } else {
        // 保留的仅是有路径的流，因此全排列仅在这些 ID 上进行
        std::vector<int> flowIds = baseFlowOrder;
        std::sort(flowIds.begin(), flowIds.end());
        do {
            flowOrders.push_back(flowIds);
        } while (std::next_permutation(flowIds.begin(), flowIds.end()));
    }

    // ============ 2️⃣ 使用固定顺序们调用递归规划 ============
    auto initialBw = bw_;
    Slice initSlice(t_);
    if (flowOrders.empty()) return allSlices;
    for (const auto& flowOrder : flowOrders) {
        recursivePlan(0, flowOrder, initialBw, initSlice, allSlices);
    }

    // ============ 3️⃣ 去除空 Slice 与返回 ============
    // bool hasNonEmptySlice = std::any_of(allSlices.begin(), allSlices.end(),
    //                                     [](const Slice& s) { return !s.lignes.empty(); });
    // if (hasNonEmptySlice) {
    //    allSlices.erase(std::remove_if(allSlices.begin(), allSlices.end(),
    //                                     [](const Slice& s) { return s.lignes.empty(); }),
    //                     allSlices.end());
    // }

#if DEBUG_SLICEPLANNER
    std::cout << "\n========== [SlicePlanner::planAllSlices] END ==========\n";
    std::cout << "  Total candidate slices: " << allSlices.size() << "\n";
    for (size_t i = 0; i < allSlices.size(); ++i) {
        std::cout << "  Slice#" << i+1
                  << "  t=" << allSlices[i].t
                  << "  lignes=" << allSlices[i].lignes.size() << "\n";
        for (auto& L : allSlices[i].lignes) {
            std::cout << "    Flow#" << L.flowId
                      << "  q=" << std::fixed << std::setprecision(3) << L.q
                      << "  score=" << std::setw(7) << std::setprecision(3) << L.score
                      << "  end=(" << L.pathXY.back().first << "," << L.pathXY.back().second << ")\n";
        }
    }
#endif

    return allSlices;
}

std::vector<std::vector<int>> SlicePlanner::computeFlowOrder() const {
    // ============ 1️⃣ 计算每个流的平均分并排序 ============
    std::vector<std::pair<double, int>> flowScores; // (平均分, flowId)

#if DEBUG_SLICEPLANNER
    std::cout << "\n-- [Computing flow average scores] --\n";
#endif

    // 计算每个流的平均分
    for (const auto& [fid, _] : remaining_) {
        // 找到对应的 Flow 对象
        const Flow* flowPtr = nullptr;
        for (auto& f : network_.flows) {
            if (f.id == fid) {
                flowPtr = &f;
                break;
            }
        }

        if (!flowPtr) continue;

        // 获取流的上下文信息
        double remain   = remaining_.count(fid)     ? remaining_.at(fid)     : -1;
        XY prevLanding  = lastLanding_.count(fid)   ? lastLanding_.at(fid)   : XY{-1,-1};
        XY nextLanding  = nextLanding_.count(fid)   ? nextLanding_.at(fid)   : XY{-1,-1};
        int change      = changeCount_.count(fid)   ? changeCount_.at(fid)   : 0;
        int neighbor    = neighborState_.count(fid) ? neighborState_.at(fid) : 0;

        // 创建 LigneFinder 并获取候选路径
        LigneFinder finder(network_, *flowPtr, t_,
                           bw_,
                           prevLanding,
                           nextLanding,
                           change,
                           neighbor,
                           remain);

        auto lignes = finder.runAStarOnce();

        if (lignes.empty())
            continue;

        double totalScore = 0.0;
        for (const auto& ligne : lignes) {
            totalScore += ligne.score;
        }
        double avgScore = (lignes.empty() ? 0.0 : totalScore / lignes.size());

        if (avgScore <= 0.0)  // 过滤掉得分为0（无可落地价值）的流
            continue;

        flowScores.emplace_back(avgScore, fid);

#if DEBUG_SLICEPLANNER
        std::cout << "  Flow#" << fid
                  << "  candidates=" << lignes.size()
                  << "  avgScore=" << std::fixed << std::setprecision(3) << avgScore << "\n";
#endif
    }

    if (flowScores.empty()) {
        return {};
    }

    // 按平均分从高到低排序
    std::sort(flowScores.begin(), flowScores.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<std::vector<int>> flowOrders;
    flowOrders.reserve(1);

    // 顺序1：平均分由高到低
    std::vector<int> highToLow;
    highToLow.reserve(flowScores.size());
    for (const auto& [score, fid] : flowScores) {
        highToLow.push_back(fid);
    }
    flowOrders.push_back(std::move(highToLow));

#if DEBUG_SLICEPLANNER
    std::cout << "\n-- [Final flow order (high to low score)] --\n";
    const auto& flowOrderDbg = flowOrders.front();
    for (size_t i = 0; i < flowOrderDbg.size(); ++i) {
        std::cout << "  #" << (i+1) << " Flow#" << flowOrderDbg[i]
                  << " avgScore=" << std::fixed << std::setprecision(3) << flowScores[i].first << "\n";
    }
#endif

    return flowOrders;
}

/**
 * @brief 递归生成 Slice 组合
 */
void SlicePlanner::recursivePlan(int index,
                                 const std::vector<int>& flowOrder,
                                 std::map<XY, double> currentBw,
                                 Slice currentSlice,
                                 std::vector<Slice>& allSlices)
{
    if (index >= (int)flowOrder.size()) {
        // 所有 flow 都尝试完毕，保存一个 Slice
        bool duplicate = false;
        for (auto& s : allSlices) {
            if (s.isSameAs(currentSlice)) {
#if DEBUG_SLICEPLANNER
                std::cout << "    [-] Duplicate Slice skipped\n";
#endif
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
#if DEBUG_SLICEPLANNER
            std::cout << "    [+] New Slice accepted, total lignes=" << currentSlice.lignes.size() << "\n";
#endif
            allSlices.push_back(currentSlice);
        }
        return;
    }

    // ============ 1️⃣ 当前流 ============
    int fid = flowOrder[index];
    const Flow* flowPtr = nullptr;
    for (auto& f : network_.flows)
        if (f.id == fid) { flowPtr = &f; break; }

    if (!flowPtr) return; // 防御

    // ============ 2️⃣ 获取当前流的上下文信息 ============
    double remain   = remaining_.count(fid)     ? remaining_.at(fid)     : -1;
    XY prevLanding  = lastLanding_.count(fid)   ? lastLanding_.at(fid)   : XY{-1,-1};
    XY nextLanding  = nextLanding_.count(fid)   ? nextLanding_.at(fid)   : XY{-1,-1};
    int change      = changeCount_.count(fid)   ? changeCount_.at(fid)   : 0;
    int neighbor    = neighborState_.count(fid) ? neighborState_.at(fid) : 0;

#if DEBUG_SLICEPLANNER
    std::cout << "\n  [Flow#" << fid << "] remain=" << remain
              << "  prevLanding=(" << prevLanding.first << "," << prevLanding.second << ")"
              << "  nextLanding=(" << nextLanding.first << "," << nextLanding.second << ")"
              << "  changeCount=" << change
              << "  neighborState=" << neighbor << "\n";
#endif

    // ============ 3️⃣ 调用 LigneFinder ============
    LigneFinder finder(network_, *flowPtr, t_,
                       currentBw,
                       prevLanding,
                       nextLanding,
                       change,
                       neighbor,
                       remain);

    auto lignes = finder.runAStarOnce();

#if DEBUG_SLICEPLANNER
    std::cout << "    [LigneFinder] found " << lignes.size()
              << " lignes for flow#" << fid << "\n";
#endif

    if (lignes.empty()) {
#if DEBUG_SLICEPLANNER
        std::cout << "    [Skip] No available path for Flow#" << fid << "\n";
#endif
        // 没有路径，也要记录当前 slice（说明这个流无法传输）
        bool duplicate = false;
        for (auto& s : allSlices) {
            if (s.isSameAs(currentSlice)) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
            allSlices.push_back(currentSlice);
        // 继续尝试后续 flow，允许后续仍然调度
        recursivePlan(index+1, flowOrder, currentBw, currentSlice, allSlices);
        return;
    }

    // ============ 4️⃣ 遍历当前流所有可行路线 ============
    for (auto& L : lignes) {
#if DEBUG_SLICEPLANNER
        std::cout << "      # q=" << std::setw(6) << std::setprecision(3) << L.q
                  << " score=" << std::setw(7) << std::setprecision(3) << L.score
                  << " path=";
        for (auto& [x,y] : L.pathXY) std::cout << "("<<x<<","<<y<<")->";
        std::cout << " end=(" << L.pathXY.back().first << "," << L.pathXY.back().second << ")\n";
#endif

        // 复制带宽表并减去消耗
        auto bwCopy = currentBw;
        for (auto& [x,y] : L.pathXY) {
            double before = bwCopy[{x,y}];
            bwCopy[{x,y}] = std::max(0.0, before - L.q);
#if DEBUG_SLICEPLANNER
            std::cout << "      [bw-update] (" << x << "," << y << ")  "
                      << std::fixed << std::setprecision(3)
                      << before << "→" << bwCopy[{x,y}] << "\n";
#endif
        }

        // 构造新的 Slice
        Slice nextSlice = currentSlice;
        nextSlice.lignes.push_back(L);

#if DEBUG_SLICEPLANNER
        std::cout << "    [Recursive → next flow] index=" << index+1
                  << " | currentSlice lignes=" << nextSlice.lignes.size() << "\n";
#endif

        // 递归调用
        recursivePlan(index+1, flowOrder, bwCopy, nextSlice, allSlices);
    }
}