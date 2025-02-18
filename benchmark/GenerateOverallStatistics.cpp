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

std::vector<ComparisonResult> read_comparison_file(const std::string& input_path) {
    std::ifstream infile(input_path);
    std::vector<ComparisonResult> results;
    std::string line;
    
    // 跳过前两行标题
    std::getline(infile, line);
    
    // 读取数据行直到 Overall Comparison
    while (std::getline(infile, line)) {
        if (line.find("Overall Comparison") != std::string::npos) {
            break;
        }
        if (line.empty()) continue;
        
        std::stringstream ss(line);
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
            std::cerr << "Error processing line: " << line << std::endl;
            std::cerr << "Error info: " << e.what() << std::endl;
            continue;
        }
    }
    
    return results;
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
    std::vector<ComparisonResult> all_results;
    
    // 遍历所有文件并收集结果
    for (const auto& entry : std::filesystem::directory_iterator(results_dir)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            
            if (filename.starts_with("benchmark_comparison_") && 
                filename.ends_with(".csv")) {
                
                std::cout << "Reading file: " << filename << std::endl;
                auto results = read_comparison_file(entry.path().string());
                all_results.insert(all_results.end(), results.begin(), results.end());
            }
        }
    }
    
    // 计算统计数据
    std::map<size_t, int> cbs_success_count, jpscbs_success_count;
    std::map<size_t, double> cbs_runtime_sum, jpscbs_runtime_sum;
    std::map<size_t, double> cbs_nodes_sum, jpscbs_nodes_sum;
    
    // 使用set来记录所有唯一的场景名称
    std::set<std::string> unique_scenarios;
    
    for (const auto& result : all_results) {
        unique_scenarios.insert(result.scen_name);
        
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
    
    // 总场景数
    int total_scenarios = unique_scenarios.size();
    
    // 写入总体统计结果
    std::ofstream outfile(results_dir / "overall_comparison_statistics.csv");
    outfile << "Overall Comparison Statistics Across All Maps:\n";
    outfile << "Agents,CBS Success Rate,CBS Avg Runtime,CBS Avg Nodes,"
            << "JPSCBS Success Rate,JPSCBS Avg Runtime,JPSCBS Avg Nodes\n";
    
    for (const auto& [agents, count] : cbs_success_count) {
        double cbs_success_rate = cbs_success_count[agents] * 100.0 / total_scenarios;
        double jpscbs_success_rate = jpscbs_success_count[agents] * 100.0 / total_scenarios;
        
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
    std::cout << "Overall statistics have been written to: overall_comparison_statistics.csv" << std::endl;

    return 0;
} 