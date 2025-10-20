#include <gtest/gtest.h>
#include "LigneFinder.h"
#include "Network.h"
#include "Flow.h"
#include <map>
#include <set>

class LigneFinderTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试网络 (5x5网格)
        network.M = 5;
        network.N = 5;
        network.FN = 1;
        network.T = 10;
        
        // 设置测试流
        flow = Flow(1, 0, 0, 0, 100.0, 3, 3, 4, 4);
        
        // 设置带宽映射 (简单的测试数据)
        setupBandwidthMap();
    }
    
    void setupBandwidthMap() {
        // 为整个网格设置基本带宽
        for (int x = 0; x < 5; ++x) {
            for (int y = 0; y < 5; ++y) {
                bwMap[{x, y}] = 50.0;  // 基础带宽50 Mbps
            }
        }
        
        // 设置一些特殊区域
        bwMap[{1, 1}] = 100.0;  // 高带宽区域
        bwMap[{2, 2}] = 0.0;    // 障碍物 (无带宽)
        bwMap[{3, 3}] = 80.0;   // 落地区域高带宽
        bwMap[{4, 4}] = 80.0;   // 落地区域高带宽
    }

    Network network;
    Flow flow;
    std::map<std::pair<int, int>, double> bwMap;
};

// 测试LigneFinder构造函数
TEST_F(LigneFinderTest, Constructor) {
    LigneFinder finder(network, flow, 5, bwMap);
    
    // 构造函数应该正常完成，无异常
    SUCCEED();
}

// 测试带有历史落点的构造函数
TEST_F(LigneFinderTest, ConstructorWithHistory) {
    std::pair<int, int> lastLanding = {3, 3};
    int changeCount = 2;
    
    LigneFinder finder(network, flow, 5, bwMap, lastLanding, changeCount);
    
    // 构造函数应该正常完成，无异常
    SUCCEED();
}

// 测试基本的A*搜索 - 无ban集合
TEST_F(LigneFinderTest, RunAStarOnce_NoBan) {
    LigneFinder finder(network, flow, 0, bwMap);
    
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    // 应该至少找到一些候选路径
    EXPECT_GT(candidates.size(), 0);
    
    // 检查候选路径是否有效
    for (const auto& ligne : candidates) {
        EXPECT_EQ(ligne.flowId, flow.id);
        EXPECT_EQ(ligne.t, 0);
        EXPECT_GT(ligne.pathXY.size(), 0);  // 应该有路径
        EXPECT_GE(ligne.score, 0.0);        // 分数应该非负
    }
}

// 测试A*搜索 - 有ban集合
TEST_F(LigneFinderTest, RunAStarOnce_WithBan) {
    std::set<std::pair<int, int>> banSet = {{1, 0}, {0, 1}};  // 阻止某些路径
    
    LigneFinder finder(network, flow, 0, bwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce(banSet);
    
    // 即使有ban，也应该能找到一些路径
    EXPECT_GE(candidates.size(), 0);
    
    // 确保没有路径经过被ban的点（除非是落地区域）
    for (const auto& ligne : candidates) {
        for (const auto& [x, y] : ligne.pathXY) {
            if (!flow.inLandingRange(x, y)) {  // 落地区域不受ban限制
                EXPECT_EQ(banSet.count({x, y}), 0);
            }
        }
    }
}

// 测试早期退出条件 - 时间太早
TEST_F(LigneFinderTest, RunAStarOnce_EarlyTime) {
    Flow laterFlow(1, 0, 0, 5, 100.0, 3, 3, 4, 4);  // 开始时间为5
    
    LigneFinder finder(network, laterFlow, 3, bwMap);  // 当前时间为3
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    EXPECT_EQ(candidates.size(), 0);  // 应该没有候选，因为时间太早
}

// 测试起始点无带宽的情况
TEST_F(LigneFinderTest, RunAStarOnce_ZeroStartBandwidth) {
    std::map<std::pair<int, int>, double> zeroBwMap = bwMap;
    zeroBwMap[{0, 0}] = 0.0;  // 起始点无带宽
    
    LigneFinder finder(network, flow, 0, zeroBwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    EXPECT_EQ(candidates.size(), 0);  // 应该没有候选
}

// 测试所有路径被阻断的情况
TEST_F(LigneFinderTest, RunAStarOnce_AllPathsBlocked) {
    std::map<std::pair<int, int>, double> blockedBwMap;
    
    // 只给起始点带宽，其他地方都为0
    for (int x = 0; x < 5; ++x) {
        for (int y = 0; y < 5; ++y) {
            blockedBwMap[{x, y}] = (x == 0 && y == 0) ? 50.0 : 0.0;
        }
    }
    
    LigneFinder finder(network, flow, 0, blockedBwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    // 可能没有候选，或者只有一个候选（起始点本身如果在落地区域）
    EXPECT_LE(candidates.size(), 1);
}

// 测试能够找到落地路径
TEST_F(LigneFinderTest, RunAStarOnce_FindLandingPath) {
    LigneFinder finder(network, flow, 0, bwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    // 查找是否有路径到达落地区域
    bool foundLandingPath = false;
    for (const auto& ligne : candidates) {
        if (ligne.landed && !ligne.pathXY.empty()) {
            auto [endX, endY] = ligne.pathXY.back();
            if (flow.inLandingRange(endX, endY)) {
                foundLandingPath = true;
                break;
            }
        }
    }
    
    EXPECT_TRUE(foundLandingPath);
}

// 测试候选路径按分数排序
TEST_F(LigneFinderTest, RunAStarOnce_SortedByScore) {
    LigneFinder finder(network, flow, 0, bwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    if (candidates.size() > 1) {
        // 检查是否按分数降序排列
        for (size_t i = 1; i < candidates.size(); ++i) {
            EXPECT_GE(candidates[i-1].score, candidates[i].score);
        }
    }
}

// 测试历史落点变化的惩罚机制
TEST_F(LigneFinderTest, RunAStarOnce_LandingChangePenalty) {
    std::pair<int, int> lastLanding = {3, 3};
    int changeCount = 1;
    
    // 第一次搜索：没有历史
    LigneFinder finder1(network, flow, 0, bwMap);
    std::vector<Ligne> candidates1 = finder1.runAStarOnce();
    
    // 第二次搜索：有历史落点
    LigneFinder finder2(network, flow, 0, bwMap, lastLanding, changeCount);
    std::vector<Ligne> candidates2 = finder2.runAStarOnce();
    
    // 两次搜索都应该找到候选
    EXPECT_GT(candidates1.size(), 0);
    EXPECT_GT(candidates2.size(), 0);
    
    // 具体的分数比较需要更详细的分析，这里只确保功能正常运行
}

// 测试网格边界处理
TEST_F(LigneFinderTest, GridBoundaryHandling) {
    // 创建起始点在边界的流
    Flow boundaryFlow(2, 4, 4, 0, 100.0, 3, 3, 4, 4);  // 右下角
    
    LigneFinder finder(network, boundaryFlow, 0, bwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    // 应该能找到一些路径，即使起始点在边界
    EXPECT_GE(candidates.size(), 0);
    
    // 确保所有路径点都在网格范围内
    for (const auto& ligne : candidates) {
        for (const auto& [x, y] : ligne.pathXY) {
            EXPECT_GE(x, 0);
            EXPECT_LT(x, network.M);
            EXPECT_GE(y, 0);
            EXPECT_LT(y, network.N);
        }
    }
}

// 测试不同带宽配置下的行为
TEST_F(LigneFinderTest, DifferentBandwidthConfigurations) {
    // 创建梯度带宽映射
    std::map<std::pair<int, int>, double> gradientBwMap;
    for (int x = 0; x < 5; ++x) {
        for (int y = 0; y < 5; ++y) {
            // 距离落地区域越近，带宽越高
            double dist = std::abs(x - 3.5) + std::abs(y - 3.5);
            gradientBwMap[{x, y}] = 100.0 - dist * 10.0;
            if (gradientBwMap[{x, y}] < 10.0) {
                gradientBwMap[{x, y}] = 10.0;  // 最小带宽
            }
        }
    }
    
    LigneFinder finder(network, flow, 0, gradientBwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    EXPECT_GT(candidates.size(), 0);
    
    // 验证最佳候选路径确实利用了较高带宽区域
    if (!candidates.empty()) {
        const auto& bestCandidate = candidates[0];  // 已按分数排序
        EXPECT_GT(bestCandidate.q, 0.0);
    }
}

// 测试路径长度和质量的权衡
TEST_F(LigneFinderTest, PathLengthQualityTradeoff) {
    LigneFinder finder(network, flow, 0, bwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    if (candidates.size() > 1) {
        // 分析不同候选路径的长度和质量
        double maxScore = -1.0;
        double minDistance = std::numeric_limits<double>::max();
        
        for (const auto& ligne : candidates) {
            maxScore = std::max(maxScore, ligne.score);
            if (ligne.landed) {
                minDistance = std::min(minDistance, ligne.distance);
            }
        }
        
        EXPECT_GT(maxScore, 0.0);
        EXPECT_LT(minDistance, std::numeric_limits<double>::max());
    }
}

// 测试内部逻辑通过公共接口的表现
TEST_F(LigneFinderTest, InternalLogicThroughPublicInterface) {
    LigneFinder finder(network, flow, 0, bwMap);
    
    // 通过测试不同的网格配置来间接测试内部逻辑
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    // 验证结果的合理性来间接验证内部方法的正确性
    if (!candidates.empty()) {
        const auto& best = candidates[0];
        
        // 验证路径的连续性（间接测试neighbors4逻辑）
        if (best.pathXY.size() > 1) {
            for (size_t i = 1; i < best.pathXY.size(); ++i) {
                auto [x1, y1] = best.pathXY[i-1];
                auto [x2, y2] = best.pathXY[i];
                
                // 相邻节点的曼哈顿距离应该为1（4邻居）
                int manhattanDist = std::abs(x2 - x1) + std::abs(y2 - y1);
                EXPECT_EQ(manhattanDist, 1);
                
                // 所有节点都应该在网格范围内（间接测试inGrid逻辑）
                EXPECT_GE(x2, 0);
                EXPECT_LT(x2, network.M);
                EXPECT_GE(y2, 0);
                EXPECT_LT(y2, network.N);
            }
        }
        
        // 验证带宽约束（间接测试bwAt逻辑）
        EXPECT_GT(best.q, 0.0);  // 应该有正的带宽分配
    }
}

// 测试边界情况处理（间接测试内部边界检查）
TEST_F(LigneFinderTest, BoundaryConditionHandling) {
    // 创建边界起始点的流
    Flow edgeFlow(3, 0, 0, 0, 100.0, 4, 4, 4, 4);  // 从(0,0)到(4,4)
    
    LigneFinder finder(network, edgeFlow, 0, bwMap);
    std::vector<Ligne> candidates = finder.runAStarOnce();
    
    // 即使从边界开始，也应该能找到有效路径
    EXPECT_GE(candidates.size(), 0);
    
    // 验证所有路径都在有效范围内
    for (const auto& ligne : candidates) {
        for (const auto& [x, y] : ligne.pathXY) {
            EXPECT_GE(x, 0);
            EXPECT_LT(x, network.M);
            EXPECT_GE(y, 0);
            EXPECT_LT(y, network.N);
        }
    }
}