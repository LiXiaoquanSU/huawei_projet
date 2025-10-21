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
 *  - 判断是否等价（同一批流，得分完全一致）。
 */
struct Slice {
    int t;                                        // 当前时刻
    std::vector<Ligne> lignes;                    // 当前时刻包含的所有路径

    Slice(int t_ = 0);

    /// 判断是否等价（同一批流，得分完全一致）
    bool isSameAs(const Slice& other) const;

    /// 输出调试信息
    void print() const;
};

#endif // SLICE_H
