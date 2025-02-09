#include "BenchmarkUtils.h"
#include "../src/cbs/CBS.h"
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    try {
        // 指定要测试的地图和场景
        std::string map_name = "Berlin_1_256";
        std::string scenario_type = "random";  // 或 "random"
        int scenario_number = 5;  // 场景编号 1-25
        
        // 初始化求解器
        auto cbs = std::make_unique<CBS>();
        cbs->set_time_limit(30.0);
        
        // 构建文件路径
        std::string root_dir = BenchmarkUtils::get_project_root();
        fs::path data_dir = fs::path(root_dir) / "data";
        fs::path map_path = data_dir / "mapf-map" / (map_name + ".map");
        fs::path scen_dir = data_dir / ("mapf-scen-" + scenario_type);
        
        std::string scen_file = BenchmarkUtils::make_scen_path(
            scen_dir.string(), 
            map_name, 
            scenario_type.c_str(), 
            scenario_number
        );
        
        // 检查文件是否存在
        if (!fs::exists(map_path)) {
            logger::log_error("地图文件不存在: " + map_path.string());
            return 1;
        }
        if (!fs::exists(scen_file)) {
            logger::log_error("场景文件不存在: " + scen_file);
            return 1;
        }
        
        logger::log_info("测试地图: " + map_name);
        logger::log_info("测试场景: " + scen_file);
        
        // 运行测试
        auto results = BenchmarkUtils::run_scen_file_impl(map_path.string(), scen_file, cbs.get());
        
        // 输出结果
        for (const auto& result : results) {
            logger::log_info("\n测试结果:");
            logger::log_info("智能体数量: " + std::to_string(result.num_agents));
            logger::log_info("是否成功: " + std::string(result.success ? "是" : "否"));
            logger::log_info("运行时间: " + std::to_string(result.runtime) + " 秒");
            logger::log_info("总代价: " + std::to_string(result.total_cost));
            logger::log_info("扩展节点数: " + std::to_string(result.nodes_expanded));
        }
        
    } catch (const std::exception& e) {
        logger::log_error("Exception in main: " + std::string(e.what()));
        return 1;
    }
    
    return 0;
} 