#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <vector>
#include <map>
#include <tuple>
#include <iostream>
#include <optional>
#include "Network.h"
#include "DTCube.h"

class Scheduler {
private:
    Network& network;
    DTCubeBuilder treeBuilder;
    std::optional<Cube> resultCube;

public:
    Scheduler(Network& net);

    // 执行调度算法
    void run();

    // 输出结果
    void outputResult(std::ostream& out) const;
};

#endif // SCHEDULER_H
