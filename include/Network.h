#ifndef NETWORK_H
#define NETWORK_H

#include <vector>
#include <iostream>
#include "UAV.h"
#include "Flow.h"

class Network {
public:
    int M; // 网格宽度
    int N; // 网格高度
    int FN; // 数据流数量
    int T; // 模拟时长（秒）

    std::vector<UAV> uavs;   // 所有UAV节点
    std::vector<Flow> flows;  // 所有数据流

    Network();

    // 从输入流加载网络拓扑与流信息
    void loadFromInput(std::istream& in);

    // 根据坐标查找UAV
    UAV* getUAV(int x, int y);
};

#endif // NETWORK_H
