#ifndef SLICE_H
#define SLICE_H

#include "Ligne.h"
#include <vector>
#include <iostream>
#include <algorithm>

/**
 * @brief 表示某个时刻的一组路径集合（复合资源分配结果）
 * 
 * Slice 的作用：
 *  - 储存该时刻所有选中的路径 Ligne；
 *  - 计算总得分与效率；
 *  - 判断是否等价（同一批流，得分一致）。
 */
struct Slice {
    int t;                                        // 当前时刻
    std::vector<Ligne> lignes;                    // 该时刻包含的所有路径
    double totalScore;                            // 所有 Ligne 的总得分
    double totalQ;                                // 所有 Ligne 的总传输量
    double efficiency;                            // 得分效率 = totalScore / totalQ

    Slice(int t_ = 0);

    /// 计算并更新总分与效率
    void calculate();

    /// 输出调试信息
    void print() const;

    /// 判断是否等价（同一批流，得分完全一致）
    bool isSameAs(const Slice& other) const;
};

#endif // SLICE_H
