#include "Slice.h"
#include <sstream>  // 用于字符串流操作
#include <iomanip>  // 用于格式化输出
#include <map>      // 用于映射容器（保留，可能的扩展需求）

/**
 * @brief 默认构造函数
 * 
 * 创建一个空的Slice对象，时刻设为0，总得分设为0
 */
Slice::Slice() : t(0), totalScore(0) {}

/**
 * @brief 参数化构造函数
 * 
 * @param t 时刻编号
 * @param lignes 该时刻的路径集合，包含所有流在当前时刻的传输路径
 * 
 * 创建指定时刻的Slice，包含一组Ligne路径。每个Ligne代表一个流的传输路径。
 */
Slice::Slice(int t, const std::vector<Ligne>& lignes)
    : t(t), lignes(lignes), totalScore(0) {}


double Slice::computeTotalScore() {
    totalScore = 0.0;
    
    // 遍历当前时刻所有流的传输路径
    for (auto& l : lignes) {
        totalScore += l.score;  // 累加每条路径的得分
    }
    
    return totalScore;
}


double Slice::getFlowTotalQ(int flowId) const {
    double sum = 0.0;
    
    // 遍历当前时刻的所有传输路径
    for (const auto& l : lignes) {
        // 找到属于指定流的路径，累加其传输速率
        if (l.flowId == flowId) {
            sum += l.q;  // q是该路径的瓶颈带宽（实际传输速率）
        }
    }
    
    return sum;
}


std::pair<int, int> Slice::getFlowEndPoint(int flowId, int M) const {
    // 遍历当前时刻的所有传输路径
    for (const auto& l : lignes) {
        // 检查是否为指定流且路径不为空
        if (l.flowId == flowId && !l.pathXY.empty()) {
            // 使用结构化绑定获取路径终点坐标
            auto [x, y] = l.pathXY.back();  // pathXY的最后一个元素即为终点
            return {x, y};
        }
    }
    
    // 未找到指定流或路径为空
    return {-1, -1};
}


std::vector<std::tuple<int, int, int, double>> Slice::exportOutputTable(int M) const {
    std::vector<std::tuple<int, int, int, double>> table;
    
    // 遍历当前时刻的所有传输路径
    for (const auto& l : lignes) {
        // 调用Ligne的exportOutput()方法获取标准格式输出
        // 返回格式：(时刻t, 终点x坐标, 终点y坐标, 传输速率q)
        table.push_back(l.exportOutput()); 
    }
    
    return table;
}


std::string Slice::summary() const {
    std::ostringstream oss;
    
    // 输出Slice标题，显示时刻编号
    oss << "--- Slice t=" << t << " ---\n";
    
    // 输出表格标题行，使用固定宽度对齐
    oss << std::setw(8) << "FlowID"      // 流ID列，宽度8
        << std::setw(12) << "Q (Mbps)"   // 传输速率列，宽度12
        << std::setw(12) << "Score"      // 得分列，宽度12
        << "\n";
    
    // 输出分隔线
    oss << "--------------------------------\n";
    
    // 遍历所有路径，输出每条路径的详细信息
    for (const auto& l : lignes) {
        oss << std::setw(8) << l.flowId                                    // 流ID
            << std::setw(12) << std::fixed << std::setprecision(2) << l.q  // 传输速率，保留2位小数
            << std::setw(12) << std::fixed << std::setprecision(2) << l.score  // 得分，保留2位小数
            << "\n";
    }
    
    // 输出底部分隔线和总得分
    oss << "--------------------------------\n";
    oss << "Total Score: " << std::fixed << std::setprecision(2) << totalScore << "\n";
    
    return oss.str();
}
