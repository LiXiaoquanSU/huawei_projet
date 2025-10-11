#include "Utils.h"
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

namespace Utils {

std::vector<std::string> listInputFiles(const std::string& inputDir) {
    std::vector<std::string> files;
    if (!fs::exists(inputDir)) {
        std::cerr << "❌ Input directory not found: " << inputDir << std::endl;
        return files;
    }

    for (const auto& entry : fs::directory_iterator(inputDir)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path().string());
        }
    }

    return files;
}

bool loadNetworkFromFile(const std::string& inputPath, Network& network) {
    std::ifstream fin(inputPath);
    if (!fin.is_open()) {
        std::cerr << "❌ Cannot open input file: " << inputPath << std::endl;
        return false;
    }

    try {
        network.loadFromInput(fin);
    } catch (const std::exception& e) {
        std::cerr << "❌ Error loading file " << inputPath << ": " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool runSchedulerAndSave(const Network& network, const std::string& outputPath) {
    Scheduler scheduler(const_cast<Network&>(network));  // 调度需要非const引用
    scheduler.run();

    std::ofstream fout(outputPath);
    if (!fout.is_open()) {
        std::cerr << "❌ Cannot open output file: " << outputPath << std::endl;
        return false;
    }

    scheduler.outputResult(fout);
    std::cout << "✅ Result saved to: " << outputPath << std::endl;
    return true;
}

std::string makeOutputPath(const std::string& inputPath,
                           const std::string& inputDir,
                           const std::string& outputDir) {
    fs::path inPath(inputPath);
    std::string base = inPath.stem().string(); // test1
    return outputDir + "/" + base + "_result.txt";
}

} // namespace Utils
