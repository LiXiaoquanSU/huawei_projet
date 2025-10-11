#ifndef RECORD_H
#define RECORD_H
/**
 * UAV调度 * t: 时间
 * t: 时间
 * (x, y): UAV坐标
 * z: 传输速率
 */
struct Record {
    int t;      // 时间
    int x, y;   // UAV坐标
    double z;   // 传输速率
};
#endif // RECORD_H