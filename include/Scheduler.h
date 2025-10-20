#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <vector>
#include <map>
#include <tuple>
#include <iostream>
#include <optional>
#include "Network.h"
#include "Cube.h"  // 暂时直接操作 Cube，不依赖 DTCubeBuilder

class Scheduler {
private:
    Network& network;
    std::optional<Cube> resultCube;

public:
    explicit Scheduler(Network& net);

    // 执行调度算法
    void run();

    // 输出结果
    void outputResult(std::ostream& out) const;
};

#endif // SCHEDULER_H
