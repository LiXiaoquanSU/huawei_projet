#include "Flow.h"

Flow::Flow()
    : id(0), x(0), y(0), startTime(0), size(0),
      m1(0), n1(0), m2(0), n2(0) {}

Flow::Flow(int id, int x, int y, int startTime, double size,
           int m1, int n1, int m2, int n2)
    : id(id), x(x), y(y), startTime(startTime), size(size),
      m1(m1), n1(n1), m2(m2), n2(n2) {}

bool Flow::inLandingRange(int ux, int uy) const {
    return (ux >= m1 && ux <= m2 && uy >= n1 && uy <= n2);
}
