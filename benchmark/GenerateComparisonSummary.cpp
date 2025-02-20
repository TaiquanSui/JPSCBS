#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <sstream>
#include <iomanip>
#include <filesystem>

struct ComparisonResult {
    std::string scen_name;
    size_t num_agents;
    bool cbs_success;
    double cbs_runtime;
    double cbs_total_cost;
    int cbs_nodes;
    bool jpscbs_success;
    double jpscbs_runtime;
    double jpscbs_total_cost;
    int jpscbs_nodes;
};

void process_csv_file(const std::string& input_path) {
    // 读取文件内容，但不包括 Overall Comparison 部分
    std::ifstream infile(input_path);
    std::vector<std::string> content;
    std::string line;
    
    std::getline(infile, line);
    content.push_back(line);
    
    // 读取数据行直到 Overall Comparison
    while (std::getline(infile, line)) {
        if (line.find("Overall Comparison") != std::string::npos) {
            break;
        }
        if (!line.empty()) {
            content.push_back(line);
        }
    }
    infile.close();
    
    // 处理数据并生成统计结果
    std::vector<ComparisonResult> results;
    for (size_t i = 1; i < content.size(); ++i) {  // 从第3行开始处理数据
        std::stringstream ss(content[i]);
        ComparisonResult result;
        std::string token;
        
        try {
            std::getline(ss, result.scen_name, ',');
            if (!std::getline(ss, token, ',') || token.empty()) continue;
            result.num_agents = std::stoi(token);
            
            std::getline(ss, token, ',');
            result.cbs_success = (token == "Yes");
            
            if (!std::getline(ss, token, ',') || token.empty()) continue;
            result.cbs_runtime = std::stod(token);
            
            if (!std::getline(ss, token, ',') || token.empty()) continue;
            result.cbs_total_cost = std::stod(token);
            
            if (!std::getline(ss, token, ',') || token.empty()) continue;
            result.cbs_nodes = std::stoi(token);
            
            std::getline(ss, token, ',');
            result.jpscbs_success = (token == "Yes");
            
            if (!std::getline(ss, token, ',') || token.empty()) continue;
            result.jpscbs_runtime = std::stod(token);
            
            if (!std::getline(ss, token, ',') || token.empty()) continue;
            result.jpscbs_total_cost = std::stod(token);
            
            if (!std::getline(ss, token, ',') || token.empty()) continue;
            result.jpscbs_nodes = std::stoi(token);
            
            results.push_back(result);
        } catch (const std::exception& e) {
            std::cerr << "error when processing line: " << content[i] << std::endl;
            std::cerr << "error info: " << e.what() << std::endl;
            continue;  // 跳过有问题的行
        }
    }
    
    // 计算统计数据
    std::map<size_t, int> cbs_success_count, jpscbs_success_count, both_success_count;
    std::map<size_t, double> cbs_runtime_sum, jpscbs_runtime_sum;
    std::map<size_t, double> cbs_nodes_sum, jpscbs_nodes_sum;
    
    // 使用set来记录所有唯一的场景名称
    std::set<std::string> unique_scenarios;
    
    for (const auto& result : results) {
        // 记录唯一场景
        unique_scenarios.insert(result.scen_name);
        
        if (result.cbs_success) {
            cbs_success_count[result.num_agents]++;
        }
        if (result.jpscbs_success) {
            jpscbs_success_count[result.num_agents]++;
        }
        
        // 只在两种算法都成功的情况下累加运行时间和节点数
        if (result.cbs_success && result.jpscbs_success) {
            both_success_count[result.num_agents]++;
            cbs_runtime_sum[result.num_agents] += result.cbs_runtime;
            cbs_nodes_sum[result.num_agents] += result.cbs_nodes;
            jpscbs_runtime_sum[result.num_agents] += result.jpscbs_runtime;
            jpscbs_nodes_sum[result.num_agents] += result.jpscbs_nodes;
        }
    }
    
    // 总场景数
    int total_scenarios = unique_scenarios.size();
    
    // 获取所有出现的智能体数量
    std::set<size_t> all_agent_counts;
    for (const auto& result : results) {
        all_agent_counts.insert(result.num_agents);
    }
    
    // 重写整个文件
    std::ofstream outfile(input_path);
    // 写入原有内容
    for (const auto& line : content) {
        outfile << line << "\n";
    }
    
    // 写入新的统计结果
    outfile << "\nOverall Comparison:\n";
    outfile << "Agents,CBS Success Rate,CBS Avg Runtime,CBS Avg Nodes,"
            << "JPSCBS Success Rate,JPSCBS Avg Runtime,JPSCBS Avg Nodes\n";
    
    // 使用所有智能体数量进行遍历
    for (const auto& agents : all_agent_counts) {
        double cbs_success_rate = cbs_success_count[agents] * 100.0 / total_scenarios;
        double jpscbs_success_rate = jpscbs_success_count[agents] * 100.0 / total_scenarios;
        
        // 计算平均运行时间和节点数（只考虑两种算法都成功的情况）
        double cbs_avg_runtime = 0.0;
        double jpscbs_avg_runtime = 0.0;
        double cbs_avg_nodes = 0.0;
        double jpscbs_avg_nodes = 0.0;

        if (both_success_count[agents] > 0) {
            cbs_avg_runtime = cbs_runtime_sum[agents] / both_success_count[agents];
            cbs_avg_nodes = cbs_nodes_sum[agents] / both_success_count[agents];
            jpscbs_avg_runtime = jpscbs_runtime_sum[agents] / both_success_count[agents];
            jpscbs_avg_nodes = jpscbs_nodes_sum[agents] / both_success_count[agents];
        }
        
        outfile << agents << ","
                << std::fixed << std::setprecision(2) << cbs_success_rate << "%,"
                << cbs_avg_runtime << ","
                << cbs_avg_nodes << ","
                << jpscbs_success_rate << "%,"
                << jpscbs_avg_runtime << ","
                << jpscbs_avg_nodes << "\n";
    }
    
    outfile.close();
    std::cout << "Has been processed: " << input_path << std::endl;
}

int main() {
    std::filesystem::path current_path = std::filesystem::current_path();
    while (!std::filesystem::exists(current_path / "data")) {
        if (current_path.parent_path() == current_path) {
            throw std::runtime_error("Project root directory not found");
        }
        current_path = current_path.parent_path();
    }

    std::filesystem::path results_dir = current_path / "benchmark_results";
    
    // 遍历目录中的所有文件
    for (const auto& entry : std::filesystem::directory_iterator(results_dir)) {
        if (entry.is_regular_file()) {  // 确保是普通文件
            std::string filename = entry.path().filename().string();
            
            // 检查文件是否以"benchmark_comparison_"开头且以".csv"结尾
            if (filename.starts_with("benchmark_comparison_") && 
                filename.ends_with(".csv")) {
                
                std::cout << "Processing file: " << filename << std::endl;
                process_csv_file(entry.path().string());
            }
        }
    }

    return 0;
} 