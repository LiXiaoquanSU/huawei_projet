#ifndef UAV_H
#define UAV_H

#include <cmath>

class UAV {
public:
    int x;      // 坐标X
    int y;      // 坐标Y
    double B;   // 峰值带宽
    int phi;    // 相位 (0~9)

    UAV();
    UAV(int x, int y, double B, int phi);

    // 获取该UAV在t秒时刻的带宽（根据题目周期模型）
    double bandwidthAt(int t) const;
};

#endif // UAV_H
