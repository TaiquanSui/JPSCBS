#ifndef BENCHMARK_UTILS_H
#define BENCHMARK_UTILS_H

#include "../src/Agent.h"
#include "../data_loader/MapLoader.h"
#include "../src/utilities/Utility.h"
#include "../src/utilities/Log.h"
#include "../src/cbs/CBS.h"
#include "../src/jpscbs/JPSCBS.h"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <filesystem>
#include <string>
#include <functional>
#include <fstream>
#include <future>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "ThreadSafeInterruptor.h"

#ifdef __linux__
    #include <pthread.h>
#elif _WIN32
    #define NOMINMAX
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#endif

namespace fs = std::filesystem;

using SolverFunction = std::function<std::vector<std::vector<Vertex>>(
    const std::vector<Agent>&, 
    const std::vector<std::vector<int>>&
)>;

using InterruptFunction = std::function<void()>;

struct BenchmarkResult {
    bool success;
    double runtime;
    double total_cost;
    size_t num_agents;
    int nodes_expanded;
    std::string map_name;
    std::string scen_name;
};

struct BenchmarkStats {
    int total_instances;
    int successful_instances;
    double avg_runtime;
    double avg_nodes_expanded;
    double success_rate;
    
    void print() const;
};

class BenchmarkUtils {
public:
    static void set_thread_affinity(int cpu_id);

    static void benchmark_all_scenarios(SolverFunction solver, InterruptFunction interrupter);
    static void benchmark_all_scenarios_comparison(SolverFunction solver1, InterruptFunction interrupter1, SolverFunction solver2, InterruptFunction interrupter2); 
    
    static BenchmarkResult run_scen_file(
        const std::string& map_file,
        const std::string& scen_file,
        SolverFunction solver,
        InterruptFunction interrupter,
        double time_limit = 30.0);                                              
                           
    static std::vector<BenchmarkResult> run_all_scenarios(
        SolverFunction solver,
        InterruptFunction interrupter);


    //辅助函数
    static std::string get_project_root();
    static std::string get_map_name(const std::string& map_path);
    static std::string make_scen_path(const std::string& scen_dir, 
                                    const std::string& map_name,
                                    const std::string& type,
                                    int index);
    
    static void print_result(const std::string& map_file, 
                           const std::string& scen_file,
                           const BenchmarkResult& result);
                           
    static BenchmarkStats calculate_stats(const std::vector<BenchmarkResult>& results);
    
    static void write_results_to_csv(const std::string& filename, 
                                   const std::vector<BenchmarkResult>& results);
                                   
    static void write_comparison_results_to_csv(const std::string& filename,
                                              const std::vector<BenchmarkResult>& cbs_results,
                                              const std::vector<BenchmarkResult>& jpscbs_results);
                                              
};

#endif // BENCHMARK_UTILS_H