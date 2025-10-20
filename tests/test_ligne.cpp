#include <gtest/gtest.h>
#include "Ligne.h"
#include <cmath>

class LigneTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试用的Ligne对象
        ligne.flowId = 1;
        ligne.t = 5;
        ligne.t_start = 3;
        ligne.Q_total = 100.0;
        ligne.Tmax = 10.0;
    }

    Ligne ligne;
};

// 测试默认构造函数
TEST_F(LigneTest, DefaultConstructor) {
    Ligne defaultLigne;
    
    EXPECT_EQ(defaultLigne.flowId, 0);
    EXPECT_EQ(defaultLigne.t, 0);
    EXPECT_EQ(defaultLigne.t_start, 0);
    EXPECT_DOUBLE_EQ(defaultLigne.Q_total, 0.0);
    EXPECT_DOUBLE_EQ(defaultLigne.Tmax, 10.0);
    EXPECT_DOUBLE_EQ(defaultLigne.distance, 0.0);
    EXPECT_DOUBLE_EQ(defaultLigne.bandwidth, 0.0);
    EXPECT_DOUBLE_EQ(defaultLigne.q, 0.0);
    EXPECT_FALSE(defaultLigne.landed);
    EXPECT_DOUBLE_EQ(defaultLigne.score, 0.0);
    EXPECT_TRUE(defaultLigne.pathXY.empty());
}

// 测试单个节点添加（不立即评分版本）
TEST_F(LigneTest, AddPathUav_SingleNode) {
    // 测试正常添加
    double result = ligne.addPathUav(1, 1, 50.0);
    
    EXPECT_GE(result, 0.0);  // 应该成功返回非负值
    EXPECT_EQ(ligne.pathXY.size(), 1);
    EXPECT_EQ(ligne.pathXY[0].first, 1);
    EXPECT_EQ(ligne.pathXY[0].second, 1);
    EXPECT_DOUBLE_EQ(ligne.q, 50.0);
    EXPECT_DOUBLE_EQ(ligne.distance, 0.0);  // 单个节点距离为0
}

// 测试添加零带宽节点
TEST_F(LigneTest, AddPathUav_ZeroBandwidth) {
    double result = ligne.addPathUav(1, 1, 0.0);
    
    EXPECT_EQ(result, -1.0);  // 应该失败
    EXPECT_TRUE(ligne.pathXY.empty());
}

// 测试添加负带宽节点
TEST_F(LigneTest, AddPathUav_NegativeBandwidth) {
    double result = ligne.addPathUav(1, 1, -10.0);
    
    EXPECT_EQ(result, -1.0);  // 应该失败
    EXPECT_TRUE(ligne.pathXY.empty());
}

// 测试多个节点添加
TEST_F(LigneTest, AddPathUav_MultipleNodes) {
    // 添加第一个节点
    EXPECT_GE(ligne.addPathUav(1, 1, 50.0), 0.0);
    
    // 添加相邻节点
    EXPECT_GE(ligne.addPathUav(2, 1, 30.0), 0.0);  // 右邻居
    
    EXPECT_EQ(ligne.pathXY.size(), 2);
    EXPECT_DOUBLE_EQ(ligne.q, 30.0);  // 应该取最小值
    EXPECT_DOUBLE_EQ(ligne.distance, 1.0);  // 两个节点，距离为1
    
    // 添加另一个相邻节点
    EXPECT_GE(ligne.addPathUav(3, 1, 40.0), 0.0);  // 继续向右
    
    EXPECT_EQ(ligne.pathXY.size(), 3);
    EXPECT_DOUBLE_EQ(ligne.q, 30.0);  // 瓶颈仍然是30.0
    EXPECT_DOUBLE_EQ(ligne.distance, 2.0);
}

// 测试重复节点添加
TEST_F(LigneTest, AddPathUav_DuplicateNode) {
    ligne.addPathUav(1, 1, 50.0);
    
    // 尝试添加同一个节点
    double result = ligne.addPathUav(1, 1, 40.0);
    
    EXPECT_EQ(result, -1.0);  // 应该失败
    EXPECT_EQ(ligne.pathXY.size(), 1);  // 路径长度不变
}

// 测试非相邻节点添加
TEST_F(LigneTest, AddPathUav_NonAdjacentNode) {
    ligne.addPathUav(1, 1, 50.0);
    
    // 尝试添加非相邻节点
    double result = ligne.addPathUav(3, 3, 40.0);  // 距离为4，非相邻
    
    EXPECT_EQ(result, -1.0);  // 应该失败
    EXPECT_EQ(ligne.pathXY.size(), 1);
}

// 测试评分计算 - Q_total为0的情况
TEST_F(LigneTest, ComputeScore_ZeroQTotal) {
    ligne.Q_total = 0.0;
    ligne.addPathUav(1, 1, 50.0);
    
    double score = ligne.computeScore(1, 1, 5, 5, 7, 7, 0.1);
    
    EXPECT_DOUBLE_EQ(score, 0.0);
    EXPECT_DOUBLE_EQ(ligne.score, 0.0);
}

// 测试评分计算 - 基本情况
TEST_F(LigneTest, ComputeScore_BasicCase) {
    ligne.addPathUav(1, 1, 50.0);  // q = 50.0
    
    double score = ligne.computeScore(1, 1, 5, 5, 7, 7, 0.1);
    
    EXPECT_GT(score, 0.0);  // 分数应该大于0
    EXPECT_DOUBLE_EQ(ligne.score, score);
}

// 测试评分计算 - 已落地情况
TEST_F(LigneTest, ComputeScore_LandedCase) {
    ligne.addPathUav(5, 5, 50.0);  // 添加在落地区域内的点
    ligne.landed = true;
    
    double score = ligne.computeScore(1, 1, 5, 5, 7, 7, 0.1);
    
    EXPECT_GT(score, 0.0);
    EXPECT_DOUBLE_EQ(ligne.score, score);
}

// 测试添加节点并立即评分版本
TEST_F(LigneTest, AddPathUavWithImediateScoring) {
    double score = ligne.addPathUav(1, 1, 50.0, 1, 1, 5, 5, 7, 7, 0.1);
    
    EXPECT_GT(score, 0.0);  // 应该返回正分数
    EXPECT_EQ(ligne.pathXY.size(), 1);
    EXPECT_DOUBLE_EQ(ligne.q, 50.0);
    EXPECT_DOUBLE_EQ(ligne.score, score);
}

// 测试添加到落地区域的节点
TEST_F(LigneTest, AddPathUavWithScoring_LandingArea) {
    double score = ligne.addPathUav(5, 5, 50.0, 1, 1, 5, 5, 7, 7, 0.1);
    
    EXPECT_GT(score, 0.0);
    EXPECT_TRUE(ligne.landed);  // 应该标记为已落地
}

// 测试添加到非落地区域的节点
TEST_F(LigneTest, AddPathUavWithScoring_NonLandingArea) {
    double score = ligne.addPathUav(1, 1, 50.0, 1, 1, 5, 5, 7, 7, 0.1);
    
    EXPECT_GT(score, 0.0);
    EXPECT_FALSE(ligne.landed);  // 应该标记为未落地
}

// 测试导出输出功能 - 空路径
TEST_F(LigneTest, ExportOutput_EmptyPath) {
    auto [t, x, y, q] = ligne.exportOutput();
    
    EXPECT_EQ(t, ligne.t);
    EXPECT_EQ(x, -1);
    EXPECT_EQ(y, -1);
    EXPECT_DOUBLE_EQ(q, 0.0);
}

// 测试导出输出功能 - 有路径
TEST_F(LigneTest, ExportOutput_WithPath) {
    ligne.addPathUav(3, 4, 25.0);
    
    auto [t, x, y, q] = ligne.exportOutput();
    
    EXPECT_EQ(t, ligne.t);
    EXPECT_EQ(x, 3);
    EXPECT_EQ(y, 4);
    EXPECT_DOUBLE_EQ(q, 25.0);
}

// 测试优先级队列比较运算符
TEST_F(LigneTest, PriorityQueueComparison) {
    Ligne ligne1, ligne2;
    
    ligne1.score = 10.0;
    ligne2.score = 20.0;
    
    // score较低的应该被认为"小于"score较高的
    EXPECT_TRUE(ligne1 < ligne2);
    EXPECT_FALSE(ligne2 < ligne1);
    
    ligne1.score = ligne2.score;
    EXPECT_FALSE(ligne1 < ligne2);
    EXPECT_FALSE(ligne2 < ligne1);
}

// 测试路径有效性检查
TEST_F(LigneTest, PathValidation_AdjacentToOlderNodes) {
    // 创建一个L形路径
    ligne.addPathUav(1, 1, 50.0);  // 起始点
    ligne.addPathUav(2, 1, 50.0);  // 向右
    ligne.addPathUav(2, 2, 50.0);  // 向上
    
    // 尝试添加与第一个节点相邻但不是最后一个节点的点
    double result = ligne.addPathUav(1, 2, 50.0);  // 与(1,1)相邻
    
    EXPECT_EQ(result, -1.0);  // 应该失败，因为与除最后节点外的节点相邻
}

// 测试瓶颈带宽更新
TEST_F(LigneTest, BottleneckBandwidthUpdate) {
    ligne.addPathUav(1, 1, 100.0);  // q = 100.0
    ligne.addPathUav(2, 1, 50.0);   // q = min(100.0, 50.0) = 50.0
    ligne.addPathUav(3, 1, 75.0);   // q = min(50.0, 75.0) = 50.0
    ligne.addPathUav(4, 1, 30.0);   // q = min(50.0, 30.0) = 30.0
    
    EXPECT_DOUBLE_EQ(ligne.q, 30.0);
    EXPECT_EQ(ligne.pathXY.size(), 4);
    EXPECT_DOUBLE_EQ(ligne.distance, 3.0);
}

// 测试边界情况 - 大量节点
TEST_F(LigneTest, LargePathTest) {
    // 创建一条长路径（直线）
    for (int i = 0; i < 10; ++i) {
        double result = ligne.addPathUav(i, 0, 50.0 - i);  // 逐渐减少带宽
        EXPECT_GE(result, 0.0);
    }
    
    EXPECT_EQ(ligne.pathXY.size(), 10);
    EXPECT_DOUBLE_EQ(ligne.distance, 9.0);
    EXPECT_DOUBLE_EQ(ligne.q, 41.0);  // 最后一个节点的带宽
}