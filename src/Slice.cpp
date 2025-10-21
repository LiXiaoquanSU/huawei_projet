#include "Slice.h"
#include <iomanip>

Slice::Slice(int t_) 
    : t(t_), totalScore(0.0), totalQ(0.0), efficiency(0.0) {}

void Slice::calculate() {
    totalScore = 0.0;
    totalQ = 0.0;

    for (const auto& L : lignes) {
        totalScore += L.score;
        totalQ += L.q;
    }

    efficiency = (totalQ > 0.0) ? (totalScore / totalQ) : 0.0;
}

bool Slice::isDominatedBy(const Slice& other) const {
    return (other.totalScore >= totalScore &&
            other.efficiency > efficiency &&
            (other.totalScore > totalScore || other.efficiency > efficiency));
}

bool Slice::isSameAs(const Slice& other) const {
    if (t != other.t) return false;
    if (lignes.size() != other.lignes.size()) return false;

    // 按 flowId 排序，确保对齐
    auto A = lignes;
    auto B = other.lignes;
    std::sort(A.begin(), A.end(), [](const Ligne& a, const Ligne& b){ return a.flowId < b.flowId; });
    std::sort(B.begin(), B.end(), [](const Ligne& a, const Ligne& b){ return a.flowId < b.flowId; });

    for (size_t i = 0; i < A.size(); ++i) {
        if (A[i].flowId != B[i].flowId) return false;
        if (std::abs(A[i].score - B[i].score) > 1e-6) return false;
    }
    return true;
}

void Slice::print() const {
    std::cout << "=== Slice t=" << t << " ===\n";
    std::cout << "  Lignes: " << lignes.size() << "\n";
    std::cout << "  Total Score: " << std::fixed << std::setprecision(3) << totalScore << "\n";
    std::cout << "  Total Q: " << totalQ << "\n";
    std::cout << "  Efficiency: " << efficiency << "\n";
    for (size_t i = 0; i < lignes.size(); ++i) {
        std::cout << "    Flow#" << lignes[i].flowId << ": ";
        for (auto [x,y] : lignes[i].pathXY)
            std::cout << "(" << x << "," << y << ")->";
        std::cout << " score=" << lignes[i].score 
                  << " q=" << lignes[i].q << "\n";
    }
}
