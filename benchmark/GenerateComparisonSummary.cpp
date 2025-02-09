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
    std::ifstream infile(input_path);
    if (!infile.is_open()) {
        std::cerr << "无法打开文件: " << input_path << std::endl;
        return;
    }
    
    std::vector<ComparisonResult> results;
    std::string line;
    
    // 跳过标题行
    std::getline(infile, line);
    
    // 读取所有结果
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        ComparisonResult result;
        std::string token;
        
        std::getline(ss, result.scen_name, ',');
        std::getline(ss, token, ','); result.num_agents = std::stoi(token);
        std::getline(ss, token, ','); result.cbs_success = (token == "Yes");
        std::getline(ss, token, ','); result.cbs_runtime = std::stod(token);
        std::getline(ss, token, ','); result.cbs_total_cost = std::stod(token);
        std::getline(ss, token, ','); result.cbs_nodes = std::stoi(token);
        std::getline(ss, token, ','); result.jpscbs_success = (token == "Yes");
        std::getline(ss, token, ','); result.jpscbs_runtime = std::stod(token);
        std::getline(ss, token, ','); result.jpscbs_total_cost = std::stod(token);
        std::getline(ss, token, ','); result.jpscbs_nodes = std::stoi(token);
        
        results.push_back(result);
    }
    
    // 计算统计数据
    std::map<size_t, int> cbs_success_count, jpscbs_success_count;
    std::map<size_t, int> total_count;
    std::map<size_t, double> cbs_runtime_sum, jpscbs_runtime_sum;
    std::map<size_t, double> cbs_nodes_sum, jpscbs_nodes_sum;
    
    for (const auto& result : results) {
        total_count[result.num_agents]++;
        if (result.cbs_success) {
            cbs_success_count[result.num_agents]++;
            cbs_runtime_sum[result.num_agents] += result.cbs_runtime;
            cbs_nodes_sum[result.num_agents] += result.cbs_nodes;
        }
        if (result.jpscbs_success) {
            jpscbs_success_count[result.num_agents]++;
            jpscbs_runtime_sum[result.num_agents] += result.jpscbs_runtime;
            jpscbs_nodes_sum[result.num_agents] += result.jpscbs_nodes;
        }
    }
    
    // 写入总结到原文件
    std::ofstream outfile(input_path, std::ios::app);
    outfile << "\nOverall Comparison:\n";
    outfile << "Agents,CBS Success Rate,CBS Avg Runtime,CBS Avg Nodes,"
            << "JPSCBS Success Rate,JPSCBS Avg Runtime,JPSCBS Avg Nodes\n";
    
    for (const auto& [agents, count] : total_count) {
        double cbs_success_rate = cbs_success_count[agents] * 100.0 / count;
        double jpscbs_success_rate = jpscbs_success_count[agents] * 100.0 / count;
        
        double cbs_avg_runtime = cbs_success_count[agents] > 0 ? 
            cbs_runtime_sum[agents] / cbs_success_count[agents] : 0;
        double jpscbs_avg_runtime = jpscbs_success_count[agents] > 0 ? 
            jpscbs_runtime_sum[agents] / jpscbs_success_count[agents] : 0;
            
        double cbs_avg_nodes = cbs_success_count[agents] > 0 ? 
            cbs_nodes_sum[agents] / cbs_success_count[agents] : 0;
        double jpscbs_avg_nodes = jpscbs_success_count[agents] > 0 ? 
            jpscbs_nodes_sum[agents] / jpscbs_success_count[agents] : 0;
        
        outfile << agents << ","
                << std::fixed << std::setprecision(2) << cbs_success_rate << "%,"
                << cbs_avg_runtime << ","
                << cbs_avg_nodes << ","
                << jpscbs_success_rate << "%,"
                << jpscbs_avg_runtime << ","
                << jpscbs_avg_nodes << "\n";
    }
    
    outfile.close();
    std::cout << "已将统计总结添加到文件: " << input_path << std::endl;
}

int main() {
    std::filesystem::path current_path = std::filesystem::current_path();
    while (!std::filesystem::exists(current_path / "data")) {
        if (current_path.parent_path() == current_path) {
            throw std::runtime_error("Project root directory not found");
        }
        current_path = current_path.parent_path();
    }
    const std::string filename = current_path.string() + "/benchmark_results/benchmark_comparison_Berlin_1_256.csv";
    process_csv_file(filename);
    return 0;


} 