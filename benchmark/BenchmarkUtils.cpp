#include "BenchmarkUtils.h"

void BenchmarkUtils::benchmark_all_scenarios(SolverFunction solver, InterruptFunction interrupter) {
    auto all_results = run_all_scenarios(solver, interrupter);
    // 计算并输出统计信息
    auto stats = calculate_stats(all_results);
    stats.print();
        
    // 输出到CSV文件
    write_results_to_csv("benchmark_results_single_solver.csv", all_results);
}

void BenchmarkUtils::benchmark_all_scenarios_comparison(
    SolverFunction solver1, 
    InterruptFunction interrupter1, 
    SolverFunction solver2, 
    InterruptFunction interrupter2) {
    
    // 设置CPU亲和性为核心0
    set_thread_affinity(0);
    
    // 先运行 CBS 算法的所有场景
    logger::log_info("\nRunning all scenarios with CBS algorithm on CPU core 0...");
    std::vector<BenchmarkResult> cbs_results = run_all_scenarios(
        solver1, 
        interrupter1
    );
    
    // 冷却时间
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 再运行 JPSCBS 算法的所有场景，保持在同一个CPU核心上
    logger::log_info("\nRunning all scenarios with JPSCBS algorithm on CPU core 0...");
    std::vector<BenchmarkResult> jpscbs_results = run_all_scenarios(
        solver2, 
        interrupter2
    );
    
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


BenchmarkResult BenchmarkUtils::run_scen_file(
    const std::string& map_file,
    const std::string& scen_file,
    SolverFunction solver,
    InterruptFunction interrupter,
    double time_limit) {
    
    auto grid = load_map(map_file);
    auto all_agents = load_scen(scen_file);

    BenchmarkResult result;
    
    fs::path map_path(map_file);
    fs::path scen_path(scen_file);
    std::string map_name = map_path.filename().string();
    std::string scen_name = scen_path.filename().string();

    // 创建路径记录文件
    // std::string solver_name = "UNKNOWN";
    // if (auto* cbs = dynamic_cast<CBS*>(solver.target<CBS>())) {
    //     solver_name = "CBS";
    // } else if (auto* jpscbs = dynamic_cast<JPSCBS*>(solver.target<JPSCBS>())) {
    //     solver_name = "JPSCBS";
    // }
    
    // std::string path_log_filename = scen_name + "_" + solver_name + "_paths.txt";
    // std::ofstream path_log(path_log_filename);
    // if (!path_log.is_open()) {
    //     throw std::runtime_error("Unable to create path log file: " + path_log_filename);
    // }

    size_t num_agents = 1;
    bool last_success = true;

    while (num_agents <= all_agents.size() && last_success) {
        std::vector<Agent> current_agents(all_agents.begin(), 
                                        all_agents.begin() + num_agents);
        
        logger::log_info("Testing " + map_name + " with " + 
                        std::to_string(num_agents) + " agents");
        for(auto& agent : current_agents) {
            logger::log_info("Agent " + std::to_string(agent.id) + " start: (" + 
                            std::to_string(agent.start.x) + ", " + std::to_string(agent.start.y) + 
                            ") goal: (" + std::to_string(agent.goal.x) + ", " + std::to_string(agent.goal.y) + ")");
        }

        try {
            auto start_time = std::chrono::steady_clock::now();
            
            std::future<std::vector<std::vector<Vertex>>> future = 
                std::async(std::launch::async, solver, current_agents, grid);
            
            bool timeout = false;
            std::vector<std::vector<Vertex>> solution;

            if (future.wait_for(std::chrono::duration<double>(time_limit)) == std::future_status::timeout) {
                timeout = true;
                logger::log_info("Solver timeout (" + std::to_string(time_limit) + " seconds)");
                
                interrupter();  // 调用中断函数
                future.wait();  // 等待线程结束
            } else {
                try {
                    solution = future.get();
                } catch (const std::exception& e) {
                    logger::log_error("Solver error: " + std::string(e.what()));
                    timeout = true;
                    solution.clear();
                }
            }

            auto end_time = std::chrono::steady_clock::now();
            double duration = std::chrono::duration<double>(end_time - start_time).count();

            // 记录结果
            // path_log << "\nTest with " << num_agents << " agents:\n";
            // if (!timeout && !solution.empty()) {
            //     for (size_t i = 0; i < solution.size(); ++i) {
            //         path_log << "Agent " << i << " path: ";
            //         for (const auto& vertex : solution[i]) {
            //             path_log << "(" << vertex.x << "," << vertex.y << ") ";
            //         }
            //         path_log << "\nPath length: " << solution[i].size() << "\n";
            //     }
            // } else {
            //     path_log << (timeout ? "TIMEOUT" : "No solution found") << "\n";
            // }
            // path_log << "----------------------------------------\n";

            bool success = !timeout && !solution.empty();
            int total_cost = 0;
            int nodes_expanded = 0;

            if (success) {
                total_cost = std::accumulate(solution.begin(), solution.end(), 0,
                    [](int sum, const auto& path) { 
                        return sum + utils::calculate_path_cost(path); 
                    });
                if (auto* cbs = dynamic_cast<CBS*>(solver.target<CBS>())) {
                    nodes_expanded = cbs->get_expanded_nodes();
                } else if (auto* jpscbs = dynamic_cast<JPSCBS*>(solver.target<JPSCBS>())) {
                    nodes_expanded = jpscbs->get_expanded_nodes();
                }
            }

            result = {
                success,
                duration,
                total_cost,
                num_agents,
                nodes_expanded,
                map_name,
                scen_name
            };

            print_result(map_file, scen_file, result);

            last_success = success;
            if (!success) break;

            num_agents++;

        } catch (const std::exception& e) {
            logger::log_error("Error: " + std::string(e.what()));
            result = {
                false,
                0,
                0,
                num_agents,
                0,
                map_name,
                scen_name
            };
            // path_log << "Error: " << e.what() << "\n";
            last_success = false;
            break;
        }
    }

    // path_log.close();
    return result;
}

std::vector<BenchmarkResult> BenchmarkUtils::run_all_scenarios(
    SolverFunction solver,
    InterruptFunction interrupter) {
    
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
                            auto results = run_scen_file(map_path, scen_file, solver, interrupter);
                            all_results.push_back(results);
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
              << "  Success Rate: " << (success_rate * 100) << "%\n"
              << "  Average Runtime: " << avg_runtime << " seconds\n"
              << "  Average Nodes Expanded: " << avg_nodes_expanded << "\n"
              << "========================================" << std::endl;
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
    BenchmarkStats stats;
    stats.total_instances = results.size();
    stats.successful_instances = std::count_if(results.begin(), results.end(),
        [](const BenchmarkResult& r) { return r.success; });
    
    double total_runtime = 0.0;
    double total_nodes = 0.0;
    for (const auto& result : results) {
        if (result.success) {
            total_runtime += result.runtime;
            total_nodes += result.nodes_expanded;
        }
    }
    
    stats.success_rate = stats.successful_instances / static_cast<double>(stats.total_instances);
    stats.avg_runtime = stats.successful_instances > 0 ? 
        total_runtime / stats.successful_instances : 0.0;
    stats.avg_nodes_expanded = stats.successful_instances > 0 ? 
        total_nodes / stats.successful_instances : 0.0;
    
    return stats;
}

void BenchmarkUtils::write_results_to_csv(
    const std::string& filename,
    const std::vector<BenchmarkResult>& results) {
    
    // 获取项目根目录
    std::string root_dir = get_project_root();
    fs::path data_dir = fs::path(root_dir) / "data" / "benchmark_results";
    
    // 确保目录存在
    if (!fs::exists(data_dir)) {
        fs::create_directories(data_dir);
    }
    
    // 构建完整的文件路径
    fs::path file_path = data_dir / filename;
    std::ofstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to create CSV file: " + file_path.string());
    }
    
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
}

void BenchmarkUtils::write_comparison_results_to_csv(
    const std::string& filename,
    const std::vector<BenchmarkResult>& cbs_results,
    const std::vector<BenchmarkResult>& jpscbs_results) {
    
    // 获取项目根目录
    std::string root_dir = get_project_root();
    fs::path data_dir = fs::path(root_dir) / "data" / "benchmark_results";
    
    // 确保目录存在
    if (!fs::exists(data_dir)) {
        fs::create_directories(data_dir);
    }
    
    // 构建完整的文件路径
    fs::path file_path = data_dir / filename;
    std::ofstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to create CSV file: " + file_path.string());
    }

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
            file << ",,,\n";
        }
    }
}