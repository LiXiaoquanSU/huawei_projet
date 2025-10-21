#include "Slice.h"
#include <iomanip>

Slice::Slice(int t_) : t(t_) {}

bool Slice::isSameAs(const Slice& other) const {
    if (t != other.t) return false;
    if (lignes.size() != other.lignes.size()) return false;

    // 按 flowId 排序后逐一对比
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
    for (size_t i = 0; i < lignes.size(); ++i) {
        const auto& L = lignes[i];
        std::cout << "    Flow#" << L.flowId << ": ";
        for (auto [x, y] : L.pathXY)
            std::cout << "(" << x << "," << y << ")->";
        std::cout << " score=" << std::fixed << std::setprecision(4)
                  << L.score << " q=" << L.q << "\n";
    }
}
