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

    // 每个flow对应的调度记录： flowId -> vector<Record>
    std::map<int, std::vector<Record>> scheduleRecords;

public:
    Scheduler(Network& net);

    // 执行调度算法
    void run();

    // 输出结果
    void outputResult(std::ostream& out) const;
};

#endif // SCHEDULER_H
