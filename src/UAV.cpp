#include "UAV.h"

UAV::UAV() : x(0), y(0), B(0), phi(0) {}

UAV::UAV(int x, int y, double B, int phi)
    : x(x), y(y), B(B), phi(phi) {}

// 根据题目定义：周期10秒
// 0,1,8,9 -> 0 Mbps; 2,7 -> B/2; 3-6 -> B
double UAV::bandwidthAt(int t) const {
    int modTime = (phi + t) % 10;
    if (modTime == 0 || modTime == 1 || modTime == 8 || modTime == 9)
        return 0.0;
    else if (modTime == 2 || modTime == 7)
        return B / 2.0;
    else
        return B;
}
