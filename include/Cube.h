#ifndef CUBE_H
#define CUBE_H

#include <vector>
#include <string>
#include <tuple>
#include "Slice.h"
#include "Network.h"

/**
 * @brief Cube 表示整个时间周期内的所有切片组合
 * 
 * 功能：
 *  - 存储全时刻的 Slice 列表
 *  - 计算整体总得分（所有 Slice 的得分总和）
 *  - 导出全时段的输出表
 *  - 打印调试摘要
 */
class Cube {
public:
    int T;  ///< 总时长（秒）
    std::vector<Slice> slices;  ///< 全时刻的切片
    double totalScore;          ///< Cube 的总得分

    // 构造函数
    Cube(int T);

    // 添加一个 Slice
    void addSlice(const Slice& slice);

    // 输出调试摘要
    std::string summary() const;
};

#endif // CUBE_H
