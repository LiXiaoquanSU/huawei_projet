#include "Scheduler.h"
#include <algorithm>
#include <iomanip>
#include <cmath>

Scheduler::Scheduler(Network& net) : network(net) {}

void Scheduler::run() {
    for (const auto& flow : network.flows) {
        double remaining = flow.size;
        std::vector<Record> recs;

        // 从 startTime 开始调度
        for (int t = flow.startTime; t < network.T && remaining > 0; ++t) {
            UAV* bestUav = nullptr;
            double bestBw = 0.0;

            // 在落地区域内寻找带宽最高的UAV
            for (auto& uav : network.uavs) {
                if (flow.inLandingRange(uav.x, uav.y)) {
                    double bw = uav.bandwidthAt(t);
                    if (bw > bestBw) {
                        bestBw = bw;
                        bestUav = &uav;
                    }
                }
            }

            if (bestUav && bestBw > 0) {
                double send = std::min(bestBw, remaining);
                remaining -= send;
                recs.push_back({t, bestUav->x, bestUav->y, send});
            }
        }

        scheduleRecords[flow.id] = recs;
    }
}

void Scheduler::outputResult(std::ostream& out) const {
    for (const auto& [flowId, recs] : scheduleRecords) {
        out << flowId << " " << recs.size() << "\n";
        for (const auto& r : recs) {
            // 判断是否为整数
            if (std::fabs(r.z - std::round(r.z)) < 1e-6)
                out << r.t << " " << r.x << " " << r.y << " " << (int)std::round(r.z) << "\n";
            else
                out << r.t << " " << r.x << " " << r.y << " " << std::fixed << std::setprecision(2) << r.z << "\n";
        }
    }
}
