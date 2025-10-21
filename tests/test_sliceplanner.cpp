#include <gtest/gtest.h>

#include <utility>

#include "SlicePlanner.h"
#include "Network.h"

TEST(SlicePlannerTest, SingleFlowSingleSliceOutput) {
    Network network;
    network.M = 2;
    network.N = 2;
    network.FN = 1;
    network.T = 1;

    int id = 0;
    for (int y = 0; y < network.N; ++y) {
        for (int x = 0; x < network.M; ++x) {
            network.uavs.emplace_back(id++, x, y, 10.0, 3);
        }
    }

    network.flows.emplace_back(
        0,   // flow id
        0, 0,// access UAV
        0,   // start time
        10.0,// total traffic
        0, 0,// landing rectangle start (x1,y1)
        0, 1 // landing rectangle end (x2,y2)
    );

    SlicePlanner planner(network, /*currentT=*/0);
    const auto slices = planner.planSlices();

    ASSERT_EQ(slices.size(), 1u);
    const Slice& slice = slices.front();
    EXPECT_EQ(slice.t, 0);
    ASSERT_EQ(slice.lignes.size(), 1u);

    const Ligne& ligne = slice.lignes.front();
    EXPECT_EQ(ligne.flowId, 0);
    EXPECT_EQ(ligne.pathXY.size(), 2u);
    EXPECT_EQ(ligne.pathXY.back(), std::make_pair(0, 1));
    EXPECT_TRUE(ligne.landed);
    EXPECT_DOUBLE_EQ(ligne.q, 10.0);

    EXPECT_NEAR(slice.totalScore, 87.991, 1e-3);
}
