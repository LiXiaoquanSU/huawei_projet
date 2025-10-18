#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <vector>
#include <map>
#include <tuple>
#include <iostream>
#include "Network.h"

class Scheduler {
private:
    Network& network;

public:
    Scheduler(Network& net);

    // 执行调度算法
    void run();

    // 输出结果
    void outputResult(std::ostream& out) const;
};

#endif // SCHEDULER_H
