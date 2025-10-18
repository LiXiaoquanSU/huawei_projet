#include "Scheduler.h"
#include <algorithm>
#include <iomanip>
#include <cmath>

Scheduler::Scheduler(Network& net) : network(net) {}

void Scheduler::run() {
    std::cout << "\n=== PathFinder A* 全时刻测试模式启动 ===\n";

    std::cout << "\n=== PathFinder 全时刻测试结束 ===\n";
}

void Scheduler::outputResult(std::ostream& out) const {
    
}
