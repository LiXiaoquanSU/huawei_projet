#pragma once
#include "Network.h"
#include "LigneFinder.h"
#include "Slice.h"
#include <map>
#include <vector>

class SlicePlanner {
public:
    using XY = std::pair<int,int>;

    SlicePlanner(const Network& net,
        const std::map<int, double>& remaining,
        const std::map<int, XY>& lastLanding,
        const std::map<int, XY>& nextLanding,
        const std::map<int, int>& changeCount,
        const std::map<int, int>& neighborState,
        int t,
        const std::map<XY, double>& bw);

    std::vector<Slice> planAllSlices();

private:
    const Network& network_;
    std::map<int, double> remaining_;
    std::map<int, XY> lastLanding_;
    std::map<int, XY> nextLanding_;
    std::map<int, int> changeCount_;
    std::map<int, int> neighborState_;  
    int t_;
    std::map<XY, double> bw_;

    void recursivePlan(int index,
                       std::vector<int>& flowOrder,
                       std::map<XY, double> currentBw,
                       Slice currentSlice,
                       std::vector<Slice>& allSlices);
};
