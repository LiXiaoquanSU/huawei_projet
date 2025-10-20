#ifndef SLICE_H
#define SLICE_H

#include <vector>
#include <string>
#include <tuple>
#include "Ligne.h"
//////加hash比较和包含比较
/**
 * @brief Slice 表示在时刻 t 的所有传输结果
 * 
 * 负责：
 *  - 存储当前时刻的所有 Ligne
 *  - 汇总分数
 *  - 提供导出标准输出格式的接口
 */
class Slice {
public:
    int t;  ///< 当前时刻
    std::vector<Ligne> lignes;  ///< 当前时刻所有路径
    double totalScore;          ///< 当前时刻得分总和

    // 构造函数
    Slice();
    Slice(int t, const std::vector<Ligne>& lignes);

    // 计算当前时刻的总得分
    double computeTotalScore();

    // 获取某个流的总传输量
    double getFlowTotalQ(int flowId) const;

    // 获取某个流的终点 (返回 (x, y))
    std::pair<int, int> getFlowEndPoint(int flowId, int M) const;

    // 导出标准输出表 (时刻, 终点x, 终点y, 数据量)
    std::vector<std::tuple<int, int, int, double>> exportOutputTable(int M) const;

    // 调试打印
    std::string summary() const;
};

#endif // SLICE_H
