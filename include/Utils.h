#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include "Network.h"
#include "Scheduler.h"

namespace Utils {

    // 获取 data 文件夹中的所有输入文件路径
    std::vector<std::string> listInputFiles(const std::string& inputDir);

    // 读取单个输入文件为 Network 对象
    bool loadNetworkFromFile(const std::string& inputPath, Network& network);

    // 运行调度并输出结果到指定路径
    bool runSchedulerAndSave(const Network& network, const std::string& outputPath);

    // 构造输出文件名（将 data/xxx.txt → output/xxx_result.txt）
    std::string makeOutputPath(const std::string& inputPath,
                               const std::string& inputDir,
                               const std::string& outputDir);
}

#endif // UTILS_H
