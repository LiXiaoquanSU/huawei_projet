#ifndef FLOW_H
#define FLOW_H

#include <vector>
#include <string>

class Flow {
public:
    int id;          // 流编号 f
    int x, y;        // 接入UAV坐标
    int startTime;   // 开始时间 t_start
    double size;     // 总流量 s (Mbits)
    int m1, n1;      // 落地区域左上角
    int m2, n2;      // 落地区域右下角
    int rest = size; // 剩余待传流量
    // 构造函数
    Flow();
    Flow(int id, int x, int y, int startTime, double size,
         int m1, int n1, int m2, int n2);

    // 判断某个UAV是否在本流的落地范围内
    bool inLandingRange(int ux, int uy) const;
};

#endif // FLOW_H