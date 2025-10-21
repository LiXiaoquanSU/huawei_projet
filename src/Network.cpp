#include "Network.h"
#include <stdexcept>

Network::Network()
    : M(0), N(0), FN(0), T(0) {}

void Network::loadFromInput(std::istream& in) {
    if (!in) throw std::runtime_error("Invalid input stream");

    // 第一行：M, N, FN, T
    in >> M >> N >> FN >> T;

   // 读取 M*N 个 UAV，并自动编号
    for (int i = 0; i < M * N; ++i) {
        int x, y, phi;
        double B;
        in >> x >> y >> B >> phi;
        int id = i;  // 自动编号（从0开始）
        uavs.emplace_back(id, x, y, B, phi);
    }
    
    // 读取 FN 条 Flow
    for (int i = 0; i < FN; ++i) {
        int id, x, y, startTime, m1, n1, m2, n2;
        double size;
        in >> id >> x >> y >> startTime >> size >> m1 >> n1 >> m2 >> n2;
        flows.emplace_back(id, x, y, startTime, size, m1, n1, m2, n2);
    }
}

const UAV* Network::getUAV(int x, int y) const{
    for (auto& u : uavs) {
        if (u.x == x && u.y == y)
            return &u;
    }
    return nullptr; // 若未找到
}
