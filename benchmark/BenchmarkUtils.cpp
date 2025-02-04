#include "BenchmarkUtils.h"

void BenchmarkUtils::benchmark_all_scenarios(CBS* solver) {
    auto all_results = run_all_scenarios_impl(solver);
    // 计算并输出统计信息
    auto stats = calculate_stats(all_results);
    stats.print();

        
    // 输出到CSV文件
    write_results_to_csv("benchmark_results_single_solver.csv", all_results);
}

void BenchmarkUtils::benchmark_all_scenarios(JPSCBS* solver) {
    auto all_results = run_all_scenarios_impl(solver);
    // 计算并输出统计信息
    auto stats = calculate_stats(all_results);
    stats.print();

        
    // 输出到CSV文件
    write_results_to_csv("benchmark_results_single_solver.csv", all_results);
}

void BenchmarkUtils::benchmark_all_scenarios_comparison(
    CBS* solver1, 
    JPSCBS* solver2) {
    
    // 设置CPU亲和性为核心0
    set_thread_affinity(0);
    
    // 先运行 CBS 算法的所有场景
    logger::log_info("\nRunning all scenarios with CBS algorithm on CPU core 0...");
    std::vector<BenchmarkResult> cbs_results = run_all_scenarios_impl(solver1);
    
    // 冷却时间
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 再运行 JPSCBS 算法的所有场景，保持在同一个CPU核心上

    logger::log_info("\nRunning all scenarios with JPSCBS algorithm on CPU core 0...");
    std::vector<BenchmarkResult> jpscbs_results = run_all_scenarios_impl(solver2);
    
    // 输出到CSV文件
    write_comparison_results_to_csv("benchmark_comparison_results.csv", 
                                  cbs_results, 
                                  jpscbs_results);
    
    // 计算并输出两个算法的统计信息
    logger::log_info("\nCBS Algorithm Statistics:");
    auto stats1 = calculate_stats(cbs_results);
    stats1.print();
    
    logger::log_info("\nJPSCBS Algorithm Statistics:");
    auto stats2 = calculate_stats(jpscbs_results);
    stats2.print();
}

// 实现通用的场景文件运行逻辑
template<typename Solver>
std::vector<BenchmarkResult> BenchmarkUtils::run_scen_file_impl(
    const std::string& map_file,
    const std::string& scen_file,
    Solver* solver,
    double time_limit) {
    
    auto grid = load_map(map_file);
    auto all_agents = load_scen(scen_file, grid);
    std::vector<BenchmarkResult> results;  // 存储所有结果
    
    fs::path map_path(map_file);
    fs::path scen_path(scen_file);
    std::string map_name = map_path.filename().string();
    std::string scen_name = scen_path.filename().string();

    size_t num_agents = 1;

    while (num_agents <= all_agents.size()) {
        solver->reset_interrupt();

        std::vector<Agent> current_agents(all_agents.begin(), 
                                        all_agents.begin() + num_agents);
        
        logger::log_info("Testing " + map_name + " with " + 
                        std::to_string(num_agents) + " agents");
        
        try {
            auto start_time = std::chrono::steady_clock::now();
            
            std::future<std::vector<std::vector<Vertex>>> future = 
                std::async(std::launch::async, 
                         [solver, &current_agents, &grid]() {
                             return solver->solve(current_agents, grid);
                         });
            
            bool timeout = false;
            std::vector<std::vector<Vertex>> solution;
            int nodes_expanded = 0;

            if (future.wait_for(std::chrono::duration<double>(time_limit)) == 
                std::future_status::timeout) {
                timeout = true;
                logger::log_info("Solver timeout (" + std::to_string(time_limit) + " seconds)");
                solver->interrupt();
                future.wait();
                nodes_expanded = solver->get_expanded_nodes();
            } else {
                try {
                    solution = future.get();
                    nodes_expanded = solver->get_expanded_nodes();
                } catch (const std::exception& e) {
                    logger::log_error("Solver error: " + std::string(e.what()));
                    timeout = true;
                    solution.clear();
                    nodes_expanded = solver->get_expanded_nodes();
                }
            }

            auto end_time = std::chrono::steady_clock::now();
            double duration = std::chrono::duration<double>(end_time - start_time).count();

            bool success = !timeout && !solution.empty();
            double total_cost = 0.0;

            if (!solution.empty()) {
                total_cost = std::accumulate(solution.begin(), solution.end(), 0.0,
                    [](double sum, const auto& path) { 
                        return sum + utils::calculate_path_cost(path); 
                    });
            }

            BenchmarkResult result = {
                success,
                duration,
                total_cost,
                num_agents,
                nodes_expanded,
                map_name,
                scen_name
            };

            results.push_back(result);  // 保存每次测试的结果
            print_result(map_file, scen_file, result);

            if (!success || timeout) break;

            num_agents++;


        } catch (const std::exception& e) {
            logger::log_error("Error: " + std::string(e.what()));
            BenchmarkResult result = {
                false,
                0,
                0,
                num_agents,
                0,
                map_name,
                scen_name
            };
            results.push_back(result);  // 保存错误情况的结果
            break;

        }
    }

    return results;  // 返回所有结果
}

// 实现通用的所有场景运行逻辑
template<typename Solver>
std::vector<BenchmarkResult> BenchmarkUtils::run_all_scenarios_impl(Solver* solver) {
    std::vector<BenchmarkResult> all_results;
    
    try {
        std::string root_dir = get_project_root();
        fs::path data_dir = fs::path(root_dir) / "data";
        fs::path maps_dir = data_dir / "mapf-map";
        fs::path random_scen_dir = data_dir / "mapf-scen-random";
        fs::path even_scen_dir = data_dir / "mapf-scen-even";
        
        if (!fs::exists(maps_dir)) {
            throw std::runtime_error("Map directory not found: " + maps_dir.string());
        }
        
        for (const auto& map_entry : fs::directory_iterator(maps_dir)) {
            if (map_entry.path().extension() == ".map") {
                std::string map_path = map_entry.path().string();
                std::string map_name = get_map_name(map_path);
                logger::log_info("Testing map: " + map_name);
                
                struct ScenType {
                    const char* name;
                    fs::path dir;
                };

                for (const ScenType& scenario : {
                    ScenType{"even", even_scen_dir},
                    ScenType{"random", random_scen_dir}
                }) {
                    for (int i = 1; i <= 1; ++i) {
                        std::string scen_file = make_scen_path(scenario.dir.string(), 
                                                             map_name, 
                                                             scenario.name, i);
                        if (fs::exists(scen_file)) {
                            logger::log_info("Testing scenario: " + scen_file);
                            auto scenario_results = run_scen_file_impl(map_path, scen_file, solver);
                            all_results.insert(all_results.end(), 
                                            scenario_results.begin(), 
                                            scenario_results.end());
                        }
                    }
                }
            }
            break;
        }
        
    } catch (const std::exception& e) {
        logger::log_error(e.what());
        throw;
    }
    
    return all_results;
}

void BenchmarkUtils::set_thread_affinity(int cpu_id) {
#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
#elif _WIN32
    SetThreadAffinityMask(GetCurrentThread(), (1ULL << cpu_id));
#endif
}


std::string BenchmarkUtils::get_project_root() {
    fs::path current_path = fs::current_path();
    while (!fs::exists(current_path / "data")) {
        if (current_path.parent_path() == current_path) {
            throw std::runtime_error("Project root directory not found");
        }
        current_path = current_path.parent_path();
    }
    return current_path.string();
}

std::string BenchmarkUtils::get_map_name(const std::string& map_path) {
    fs::path path(map_path);
    return path.stem().string();
}

std::string BenchmarkUtils::make_scen_path(const std::string& scen_dir, 
                                         const std::string& map_name,
                                         const std::string& type,
                                         int index) {
    return (fs::path(scen_dir) / 
            (map_name + "-" + type + "-" + std::to_string(index) + ".scen")).string();
}

void BenchmarkStats::print() const {
    std::cout << std::fixed << std::setprecision(2)
              << "\nBenchmark Stats:\n"
              << "  Total Instances: " << total_instances << "\n"
              << "  Successful Instances: " << successful_instances << "\n"
              << "  Per-Agent Statistics:\n";
              
    for (const auto& [agents, rate] : success_rates) {
        std::cout << "    " << agents << " agents:\n"
                 << "      Success Rate: " << (rate * 100) << "%\n"
                 << "      Avg Runtime: " << avg_runtimes.at(agents) << "s\n"
                 << "      Avg Nodes: " << avg_nodes.at(agents) << "\n";
    }
    std::cout << "========================================" << std::endl;
}

void BenchmarkUtils::print_result(const std::string& map_file, 
                                const std::string& scen_file,
                                const BenchmarkResult& result) {
    std::cout << std::fixed << std::setprecision(2)
              << "Result:\n"
              << "  Map: " << map_file << "\n"
              << "  Scenario: " << scen_file << "\n"
              << "  Number of Agents: " << result.num_agents << "\n"
              << "  Success: " << (result.success ? "Yes" : "No") << "\n"
              << "  Runtime: " << result.runtime << " seconds\n"
              << "  Solution Cost: " << result.total_cost << "\n"
              << "  Nodes Expanded: " << result.nodes_expanded << "\n"
              << "----------------------------------------" << std::endl;
}

BenchmarkStats BenchmarkUtils::calculate_stats(const std::vector<BenchmarkResult>& results) {
    BenchmarkStats stats_obj;
    
    // 统计不同场景
    std::set<std::pair<std::string, std::string>> unique_scenarios;  // {map_name, scen_name}
    for (const auto& result : results) {
        unique_scenarios.insert({result.map_name, result.scen_name});
    }
    int total_scenarios = unique_scenarios.size();

    // 按照agent数量统计成功率
    std::map<size_t, int> success_counts;  // agent数量 -> 成功次数
    for (const auto& result : results) {
        if (result.success) {
            success_counts[result.num_agents]++;
        }
    }
    
    // 计算每个agent数量的成功率
    for (const auto& [agents, successes] : success_counts) {
        stats_obj.success_rates[agents] = static_cast<double>(successes) / total_scenarios;
    }
    
    // 统计不同场景和成功案例的运行时间、节点数
    std::map<size_t, std::pair<double, int>> runtime_stats;   // agent数量 -> {总时间, 成功次数}
    std::map<size_t, std::pair<double, int>> nodes_stats;     // agent数量 -> {总节点数, 成功次数}
    
    for (const auto& result : results) {
        if (result.success) {
            runtime_stats[result.num_agents].first += result.runtime;
            runtime_stats[result.num_agents].second++;
            nodes_stats[result.num_agents].first += result.nodes_expanded;
            nodes_stats[result.num_agents].second++;
        }
    }
    
    // 计算每个agent数量的平均值
    for (const auto& [agents, stats] : runtime_stats) {
        stats_obj.avg_runtimes[agents] = stats.second > 0 ? 
            stats.first / stats.second : 0.0;
    }
    
    for (const auto& [agents, stats] : nodes_stats) {
        stats_obj.avg_nodes[agents] = stats.second > 0 ? 
            stats.first / stats.second : 0.0;
    }
    
    // 计算其他统计信息
    stats_obj.total_instances = results.size();
    stats_obj.successful_instances = std::count_if(results.begin(), results.end(),
        [](const BenchmarkResult& r) { return r.success; });
    
    double total_runtime = 0.0;
    double total_nodes = 0.0;
    for (const auto& result : results) {
        if (result.success) {
            total_runtime += result.runtime;
            total_nodes += result.nodes_expanded;
        }
    }
    
    return stats_obj;
}

// 创建CSV文件并返回文件流
std::ofstream BenchmarkUtils::create_csv_file(const std::string& filename) {
    // 获取项目根目录
    std::string root_dir = get_project_root();
    fs::path results_dir = fs::path(root_dir) / "benchmark_results";
    
    // 确保目录存在
    if (!fs::exists(results_dir)) {
        logger::log_info("Creating directory: " + results_dir.string());
        if (!fs::create_directories(results_dir)) {
            throw std::runtime_error("Failed to create directory: " + results_dir.string());
        }
    }
    
    // 构建完整的文件路径
    fs::path file_path = results_dir / filename;
    logger::log_info("Writing results to: " + file_path.string());
    
    std::ofstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to create CSV file: " + file_path.string());
    }
    
    return file;
}

void BenchmarkUtils::write_results_to_csv(
    const std::string& filename,
    const std::vector<BenchmarkResult>& results) {
    
    try {
        auto file = create_csv_file(filename);
        
        // 写入详细结果
        file << "Detailed Results:\n";
        file << "Map,Scenario,Agents,Success,Runtime,Total Cost,Nodes Expanded\n";
        
        for (const auto& result : results) {
            file << result.map_name << ","
                 << result.scen_name << ","
                 << result.num_agents << ","
                 << (result.success ? "Yes" : "No") << ","
                 << std::fixed << std::setprecision(2) << result.runtime << ","
                 << result.total_cost << ","
                 << result.nodes_expanded << "\n";
        }
        
        // 写入统计信息
        auto stats = calculate_stats(results);
        file << "\nSummary Statistics:\n";
        file << "Agents,Success Rate,Average Runtime,Average Nodes\n";
        
        for (const auto& [agents, rate] : stats.success_rates) {
            file << agents << ","
                 << std::fixed << std::setprecision(2) << (rate * 100) << "%,"
                 << stats.avg_runtimes[agents] << ","
                 << stats.avg_nodes[agents] << "\n";
        }
        
        file.close();
        logger::log_info("Results and statistics written successfully");
        
    } catch (const std::exception& e) {
        logger::log_error("Error writing results to CSV: " + std::string(e.what()));
        throw;
    }
}

void BenchmarkUtils::write_comparison_results_to_csv(
    const std::string& filename,
    const std::vector<BenchmarkResult>& cbs_results,
    const std::vector<BenchmarkResult>& jpscbs_results) {
    
    try {
        auto file = create_csv_file(filename);
        
        // 写入详细比较结果
        file << "Detailed Comparison Results:\n";
        file << "Map,Scenario,Agents,CBS Success,CBS Runtime,CBS Total Cost,CBS Nodes,"
             << "JPSCBS Success,JPSCBS Runtime,JPSCBS Total Cost,JPSCBS Nodes\n";

        size_t max_size = std::max(cbs_results.size(), jpscbs_results.size());
        for (size_t i = 0; i < max_size; ++i) {
            if (i < cbs_results.size()) {
                const auto& cbs = cbs_results[i];
                file << cbs.map_name << ","
                     << cbs.scen_name << ","
                     << cbs.num_agents << ","
                     << (cbs.success ? "Yes" : "No") << ","
                     << std::fixed << std::setprecision(2) << cbs.runtime << ","
                     << cbs.total_cost << ","
                     << cbs.nodes_expanded << ",";
            } else {
                file << ",,,,,,,,";
            }

            if (i < jpscbs_results.size()) {
                const auto& jpscbs = jpscbs_results[i];
                file << (jpscbs.success ? "Yes" : "No") << ","
                     << std::fixed << std::setprecision(2) << jpscbs.runtime << ","
                     << jpscbs.total_cost << ","
                     << jpscbs.nodes_expanded << "\n";
            } else {
                file << ",,,,\n";
            }
        }
        
        // 写入统计信息比较
        auto cbs_stats = calculate_stats(cbs_results);
        auto jpscbs_stats = calculate_stats(jpscbs_results);
        
        file << "\nSummary Statistics Comparison:\n";
        file << "Agents,CBS Success Rate,CBS Avg Runtime,CBS Avg Nodes,"
             << "JPSCBS Success Rate,JPSCBS Avg Runtime,JPSCBS Avg Nodes\n";
        
        // 合并所有agent数量
        std::set<size_t> all_agents;
        for (const auto& [agents, _] : cbs_stats.success_rates) all_agents.insert(agents);
        for (const auto& [agents, _] : jpscbs_stats.success_rates) all_agents.insert(agents);
        
        for (size_t agents : all_agents) {
            file << agents << ","
                 << std::fixed << std::setprecision(2)
                 << (cbs_stats.success_rates[agents] * 100) << "%,"
                 << cbs_stats.avg_runtimes[agents] << ","
                 << cbs_stats.avg_nodes[agents] << ","
                 << (jpscbs_stats.success_rates[agents] * 100) << "%,"
                 << jpscbs_stats.avg_runtimes[agents] << ","
                 << jpscbs_stats.avg_nodes[agents] << "\n";
        }
        
        file.close();
        logger::log_info("Comparison results written successfully");
        
    } catch (const std::exception& e) {
        logger::log_error("Error writing comparison results to CSV: " + std::string(e.what()));
        throw;
    }
}