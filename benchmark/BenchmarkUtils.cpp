#include "BenchmarkUtils.h"

#ifdef _WIN32
    DWORD_PTR BenchmarkUtils::original_affinity = 0;
    int BenchmarkUtils::original_priority = 0;
#elif __linux__
    cpu_set_t BenchmarkUtils::original_affinity;
    int BenchmarkUtils::original_priority = 0;
#endif



void BenchmarkUtils::benchmark_all_scenarios(CBS* solver) {
    std::vector<BenchmarkResult> all_results;
    auto map_paths = get_all_map_paths();
    std::string root_dir = get_project_root();
    
    for (const auto& map_path : map_paths) {
        std::string csv_filename = (fs::path(root_dir) / ("benchmark_cbs_" + get_map_name(map_path) + ".csv")).string();
        if (fs::exists(csv_filename)) {
            logger::log_info("跳过已完成的地图: " + get_map_name(map_path));
            continue;

        }
        auto results = run_all_scenarios_impl(map_path, solver);
        all_results.insert(all_results.end(), results.begin(), results.end());
        write_results_to_csv("benchmark_cbs_"+get_map_name(map_path)+".csv", results);
    }
    
    // 计算并输出统计信息
    auto stats = calculate_stats(all_results);
    stats.print();
    write_summary_to_csv("benchmark_cbs_summary.csv", all_results);
}

void BenchmarkUtils::benchmark_all_scenarios(JPSCBS* solver) {
    std::vector<BenchmarkResult> all_results;
    auto map_paths = get_all_map_paths();
    std::string root_dir = get_project_root();
    
    for (const auto& map_path : map_paths) {
        std::string csv_filename = (fs::path(root_dir) / "benchmark_results" / ("benchmark_jpscbs_" + get_map_name(map_path) + ".csv")).string();
        if (fs::exists(csv_filename)) {
            logger::log_info("跳过已完成的地图: " + get_map_name(map_path));
            continue;
        }
        
        auto results = run_all_scenarios_impl(map_path, solver);
        all_results.insert(all_results.end(), results.begin(), results.end());
        write_results_to_csv("benchmark_jpscbs_"+get_map_name(map_path)+".csv", results);
    }

    // 计算并输出统计信息
    auto stats = calculate_stats(all_results);
    stats.print();
    write_summary_to_csv("benchmark_jpscbs_summary.csv", all_results);
}

void BenchmarkUtils::benchmark_all_scenarios_comparison(
    CBS* solver1, 
    JPSCBS* solver2,
    int cpu_core) {
    
    try {
        int selected_core = cpu_core;
        if (selected_core < 0) {
            selected_core = find_least_busy_cpu();
            logger::log_info("Automatically selected CPU core: " + std::to_string(selected_core));
        }
        
        set_thread_affinity(selected_core);
        optimize_thread_priority();
        
        logger::log_info("\nUsing CPU core " + std::to_string(selected_core) + " for benchmarking...");
        
        std::vector<BenchmarkResult> cbs_results, jpscbs_results;
        auto map_paths = get_all_map_paths();
        std::string root_dir = get_project_root();
        
        for (const auto& map_path : map_paths) {
            std::string csv_filename = (fs::path(root_dir) / "benchmark_results" / ("benchmark_comparison_" + get_map_name(map_path) + ".csv")).string();
            logger::log_info("csv_filename: " + csv_filename);
            if (fs::exists(csv_filename)) {
                logger::log_info("跳过已完成的地图: " + get_map_name(map_path));
                continue;
            }

            // 运行CBS
            logger::log_info("\nRunning CBS algorithm on map: " + get_map_name(map_path));
            auto map_cbs_results = run_all_scenarios_impl(map_path, solver1);
            cbs_results.insert(cbs_results.end(), map_cbs_results.begin(), map_cbs_results.end());
            
            // 冷却时间
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // 运行JPSCBS
            logger::log_info("\nRunning JPSCBS algorithm on map: " + get_map_name(map_path));
            auto map_jpscbs_results = run_all_scenarios_impl(map_path, solver2);
            jpscbs_results.insert(jpscbs_results.end(), map_jpscbs_results.begin(), map_jpscbs_results.end());
            
            // 即时写入当前地图的结果
            write_comparison_results_to_csv("benchmark_comparison_"+get_map_name(map_path)+".csv", 
                                          map_cbs_results, 
                                          map_jpscbs_results);
        }

        write_comparison_summary_to_csv("benchmark_comparison_summary.csv", 
                                        cbs_results, 
                                        jpscbs_results);
        
        restore_thread_priority();
        
    } catch (...) {
        restore_thread_priority();
        throw;
    }
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
std::vector<BenchmarkResult> BenchmarkUtils::run_all_scenarios_impl(
    const std::string& map_path,
    Solver* solver) {
    
    std::vector<BenchmarkResult> all_results;
    
    try {
        std::string root_dir = get_project_root();
        fs::path data_dir = fs::path(root_dir) / "data";
        fs::path random_scen_dir = data_dir / "mapf-scen-random";
        fs::path even_scen_dir = data_dir / "mapf-scen-even";
        
        std::string map_name = get_map_name(map_path);
        logger::log_info("Testing map: " + map_name);
        
        struct ScenType {
            const char* name;
            fs::path dir;
        };
        
        std::vector<ScenType> scenario_types = {
            {"even", even_scen_dir},
            {"random", random_scen_dir}
        };

        for (const auto& scenario : scenario_types) {
            logger::log_info("Testing " + std::string(scenario.name) + " scenarios");
            for (int i = 1; i <= 5; ++i) {
                std::string scen_file = make_scen_path(scenario.dir.string(), 
                                                     map_name, 
                                                     scenario.name, i);
                if (fs::exists(scen_file)) {
                    logger::log_info("Testing scenario: " + scen_file);
                    auto scenario_results = run_scen_file_impl(map_path, scen_file, solver);
                    all_results.insert(all_results.end(), 
                                    scenario_results.begin(), 
                                    scenario_results.end());
                } else {
                    logger::log_warning("Scenario file not found: " + scen_file);
                }
            }
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

std::vector<std::string> BenchmarkUtils::get_all_map_paths() {
    std::vector<std::string> map_paths;
    std::string root_dir = get_project_root();
    fs::path data_dir = fs::path(root_dir) / "data";
    fs::path maps_dir = data_dir / "mapf-map";
    
    if (!fs::exists(maps_dir)) {
        throw std::runtime_error("Map directory not found: " + maps_dir.string());
    }
    
    for (const auto& map_entry : fs::directory_iterator(maps_dir)) {
        if (map_entry.path().extension() == ".map") {
            map_paths.push_back(map_entry.path().string());
        }
    }
    
    return map_paths;
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
        file << "Scenario,Agents,Success,Runtime,Total Cost,Nodes Expanded\n";
        
        for (const auto& result : results) {
            file << result.scen_name << ","
                 << result.num_agents << ","
                 << (result.success ? "Yes" : "No") << ","
                 << std::fixed << std::setprecision(2) << result.runtime << ","
                 << result.total_cost << ","
                 << result.nodes_expanded << "\n";
        }
        
        file.close();
        logger::log_info("Results written to: " + filename);
        
    } catch (const std::exception& e) {
        logger::log_error("Error writing results to CSV: " + std::string(e.what()));
        throw;
    }
}

void BenchmarkUtils::write_summary_to_csv(
    const std::string& filename,
    const std::vector<BenchmarkResult>& all_results) {
    
    try {
        auto file = create_csv_file(filename);
        auto stats = calculate_stats(all_results);
        
        file << "Overall Statistics:\n";
        file << "Agents,Success Rate,Average Runtime,Average Nodes\n";
        
        for (const auto& [agents, rate] : stats.success_rates) {
            file << agents << ","
                 << std::fixed << std::setprecision(2) << (rate * 100) << "%,"
                 << stats.avg_runtimes[agents] << ","
                 << stats.avg_nodes[agents] << "\n";
        }
        
        file.close();
        logger::log_info("Summary statistics written to: " + filename);
        
    } catch (const std::exception& e) {
        logger::log_error("Error writing summary to CSV: " + std::string(e.what()));
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
        file << "Scenario,Agents,CBS Success,CBS Runtime,CBS Total Cost,CBS Nodes,"
             << "JPSCBS Success,JPSCBS Runtime,JPSCBS Total Cost,JPSCBS Nodes\n";

        // 按场景名称组织结果
        std::map<std::string, std::pair<
            std::unordered_map<size_t, BenchmarkResult>,
            std::unordered_map<size_t, BenchmarkResult>>> results_by_scen;
            
        // 组织CBS结果
        for (const auto& result : cbs_results) {
            results_by_scen[result.scen_name].first[result.num_agents] = result;
        }
        
        // 组织JPSCBS结果
        for (const auto& result : jpscbs_results) {
            results_by_scen[result.scen_name].second[result.num_agents] = result;
        }
        
        // 写入每个场景的比较结果
        for (const auto& [scen_name, scen_results] : results_by_scen) {
            const auto& cbs_map = scen_results.first;
            const auto& jpscbs_map = scen_results.second;
            
            std::set<size_t> all_agents;
            for (const auto& [agents, _] : cbs_map) all_agents.insert(agents);
            for (const auto& [agents, _] : jpscbs_map) all_agents.insert(agents);
            
            for (size_t agents : all_agents) {
                file << scen_name << "," << agents << ",";
                
                // CBS结果
                if (auto it = cbs_map.find(agents); it != cbs_map.end()) {
                    const auto& cbs = it->second;
                    file << (cbs.success ? "Yes" : "No") << ","
                         << std::fixed << std::setprecision(2) << cbs.runtime << ","
                         << cbs.total_cost << ","
                         << cbs.nodes_expanded << ",";
                } else {
                    file << "No,0,0,0,";
                }

                // JPSCBS结果
                if (auto it = jpscbs_map.find(agents); it != jpscbs_map.end()) {
                    const auto& jpscbs = it->second;
                    file << (jpscbs.success ? "Yes" : "No") << ","
                         << std::fixed << std::setprecision(2) << jpscbs.runtime << ","
                         << jpscbs.total_cost << ","
                         << jpscbs.nodes_expanded << "\n";
                } else {
                    file << "No,0,0,0\n";
                }
            }
        }
        
        file.close();
        logger::log_info("Comparison results written to: " + filename);
        
    } catch (const std::exception& e) {
        logger::log_error("Error writing comparison results to CSV: " + std::string(e.what()));
        throw;
    }
}

void BenchmarkUtils::write_comparison_summary_to_csv(
    const std::string& filename,
    const std::vector<BenchmarkResult>& cbs_results,
    const std::vector<BenchmarkResult>& jpscbs_results) {
    
    try {
        auto file = create_csv_file(filename);
        auto cbs_stats = calculate_stats(cbs_results);
        auto jpscbs_stats = calculate_stats(jpscbs_results);
        
        file << "Overall Comparison Statistics:\n";
        file << "Agents,CBS Success Rate,CBS Avg Runtime,CBS Avg Nodes,"
             << "JPSCBS Success Rate,JPSCBS Avg Runtime,JPSCBS Avg Nodes\n";
        
        std::set<size_t> all_agents;
        for (const auto& [agents, _] : cbs_stats.success_rates) all_agents.insert(agents);
        for (const auto& [agents, _] : jpscbs_stats.success_rates) all_agents.insert(agents);
        
        for (size_t agents : all_agents) {
            file << agents << ","
                 << std::fixed << std::setprecision(2)
                 << (cbs_stats.success_rates.count(agents) ? cbs_stats.success_rates[agents] * 100 : 0.0) << "%,"
                 << (cbs_stats.avg_runtimes.count(agents) ? cbs_stats.avg_runtimes[agents] : 0.0) << ","
                 << (cbs_stats.avg_nodes.count(agents) ? cbs_stats.avg_nodes[agents] : 0.0) << ","
                 << (jpscbs_stats.success_rates.count(agents) ? jpscbs_stats.success_rates[agents] * 100 : 0.0) << "%,"
                 << (jpscbs_stats.avg_runtimes.count(agents) ? jpscbs_stats.avg_runtimes[agents] : 0.0) << ","
                 << (jpscbs_stats.avg_nodes.count(agents) ? jpscbs_stats.avg_nodes[agents] : 0.0) << "\n";
        }
        
        file.close();
        logger::log_info("Comparison summary written to: " + filename);
        
    } catch (const std::exception& e) {
        logger::log_error("Error writing comparison summary to CSV: " + std::string(e.what()));
        throw;
    }
}

// 添加查找最空闲CPU核心的函数
int BenchmarkUtils::find_least_busy_cpu() {
    SYSTEM_INFO sysinfo;
    GetSystemInfo(&sysinfo);
    int num_cpus = sysinfo.dwNumberOfProcessors;
    
    double min_load = std::numeric_limits<double>::max();
    int selected_cpu = 1;  // 默认从CPU 1开始，避免使用CPU 0
    
    // 收集每个CPU的负载
    for (int i = 1; i < num_cpus; ++i) {
        double load = get_cpu_load(i);
        if (load < min_load) {
            min_load = load;
            selected_cpu = i;
        }
    }
    
    logger::log_info("Selected CPU " + std::to_string(selected_cpu) + 
                    " with load " + std::to_string(min_load * 100) + "%");
    return selected_cpu;
}

double BenchmarkUtils::get_cpu_load(int cpu_id) {
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    
    // 检查CPU ID是否有效
    if (cpu_id < 0 || cpu_id >= static_cast<int>(sysInfo.dwNumberOfProcessors)) {
        logger::log_error("Invalid CPU ID: " + std::to_string(cpu_id));
        return 0.0;
    }

    DWORD_PTR oldMask = SetThreadAffinityMask(GetCurrentThread(), (static_cast<DWORD_PTR>(1) << cpu_id));
    if (oldMask == 0) {
        logger::log_error("Failed to set thread affinity for CPU " + std::to_string(cpu_id));
        return 0.0;
    }

    FILETIME idleTime1, kernelTime1, userTime1;
    FILETIME idleTime2, kernelTime2, userTime2;

    // 获取第1次CPU时间
    if (!GetSystemTimes(&idleTime1, &kernelTime1, &userTime1)) {
        SetThreadAffinityMask(GetCurrentThread(), oldMask);
        logger::log_error("Failed to get first CPU times for CPU " + std::to_string(cpu_id));
        return 0.0;
    }

    // 等待100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 获取第2次CPU时间
    if (!GetSystemTimes(&idleTime2, &kernelTime2, &userTime2)) {
        SetThreadAffinityMask(GetCurrentThread(), oldMask);
        logger::log_error("Failed to get second CPU times for CPU " + std::to_string(cpu_id));
        return 0.0;
    }

    // 恢复原来的线程亲和性
    SetThreadAffinityMask(GetCurrentThread(), oldMask);

    // 计算时间差
    ULONGLONG idle1 = (static_cast<ULONGLONG>(idleTime1.dwHighDateTime) << 32) | idleTime1.dwLowDateTime;
    ULONGLONG kernel1 = (static_cast<ULONGLONG>(kernelTime1.dwHighDateTime) << 32) | kernelTime1.dwLowDateTime;
    ULONGLONG user1 = (static_cast<ULONGLONG>(userTime1.dwHighDateTime) << 32) | userTime1.dwLowDateTime;

    ULONGLONG idle2 = (static_cast<ULONGLONG>(idleTime2.dwHighDateTime) << 32) | idleTime2.dwLowDateTime;
    ULONGLONG kernel2 = (static_cast<ULONGLONG>(kernelTime2.dwHighDateTime) << 32) | kernelTime2.dwLowDateTime;
    ULONGLONG user2 = (static_cast<ULONGLONG>(userTime2.dwHighDateTime) << 32) | userTime2.dwLowDateTime;

    ULONGLONG idleDiff = idle2 - idle1;
    ULONGLONG kernelDiff = kernel2 - kernel1;
    ULONGLONG userDiff = user2 - user1;

    ULONGLONG totalDiff = kernelDiff + userDiff;
    if (totalDiff == 0) {
        logger::log_warning("Zero total time difference for CPU " + std::to_string(cpu_id));
        return 0.0;
    }

    // 计算CPU使用率（返回0-1之间的值）
    double cpuUsage = 1.0 - (static_cast<double>(idleDiff) / totalDiff);
    
    logger::log_info("CPU " + std::to_string(cpu_id) + " load: " + 
                    std::to_string(cpuUsage * 100) + "%");
    
    return cpuUsage;
}

void BenchmarkUtils::optimize_thread_priority() {
    #ifdef _WIN32
        // 保存原始设置
        HANDLE thread = GetCurrentThread();
        original_affinity = SetThreadAffinityMask(thread, 0);
        original_priority = GetThreadPriority(thread);
        
        // 设置为高优先级
        SetThreadPriority(thread, THREAD_PRIORITY_HIGHEST);
        
        // 设置进程优先级
        HANDLE process = GetCurrentProcess();
        SetPriorityClass(process, HIGH_PRIORITY_CLASS);
        
    #elif __linux__
        // 保存原始设置
        pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &original_affinity);
        original_priority = getpriority(PRIO_PROCESS, 0);
        
        // 设置为高优先级
        setpriority(PRIO_PROCESS, 0, -20);  // Linux优先级范围：-20(最高)到19(最低)
        
        // 设置实时调度策略
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    #endif
}

void BenchmarkUtils::restore_thread_priority() {
    #ifdef _WIN32
        HANDLE thread = GetCurrentThread();
        SetThreadAffinityMask(thread, original_affinity);
        SetThreadPriority(thread, original_priority);
        
        HANDLE process = GetCurrentProcess();
        SetPriorityClass(process, NORMAL_PRIORITY_CLASS);
        
    #elif __linux__
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &original_affinity);
        setpriority(PRIO_PROCESS, 0, original_priority);
        
        struct sched_param param;
        param.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    #endif
}

