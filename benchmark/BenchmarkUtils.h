#ifndef BENCHMARK_UTILS_H
#define BENCHMARK_UTILS_H

#include "../src/basic/Agent.h"
#include "../data_loader/MapLoader.h"
#include "../src/utilities/GridUtility.h"
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
#include <set>
#include <map>

#ifdef __linux__
    #include <pthread.h>
#elif _WIN32
    #define NOMINMAX
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#endif

namespace fs = std::filesystem;

using SolverFunction = std::function<std::vector<std::vector<Vertex>>(
    CBS* solver,
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
    std::map<size_t, double> success_rates;    // agent数量 -> 成功率
    std::map<size_t, double> avg_runtimes;     // agent数量 -> 平均运行时间
    std::map<size_t, double> avg_nodes;        // agent数量 -> 平均展开节点数
    
    void print() const;
};

class BenchmarkUtils {
public:
    static void benchmark_all_scenarios(CBS* solver);
    static void benchmark_all_scenarios(JPSCBS* solver);
    
    static void benchmark_all_scenarios_comparison(
        CBS* solver1,
        JPSCBS* solver2,
        int cpu_core = -1);

    static std::string get_project_root();
    static std::string make_scen_path(const std::string& scen_dir, 
                                    const std::string& map_name,
                                    const std::string& type,
                                    int index);

    template<typename Solver>
    static std::vector<BenchmarkResult> run_scen_file_impl(
        const std::string& map_file,
        const std::string& scen_file,
        Solver* solver,
        double time_limit = 30.0);

    template<typename Solver>
    static std::vector<BenchmarkResult> run_all_scenarios_impl(const std::string& map_path, Solver* solver);

private:

    // 辅助函数
    static std::vector<std::string> get_all_map_paths();
    static std::string get_map_name(const std::string& map_path);
         
    static void print_result(const std::string& map_file, 
                           const std::string& scen_file,
                           const BenchmarkResult& result);
                           
    static BenchmarkStats calculate_stats(const std::vector<BenchmarkResult>& results);
    
    static void write_results_to_csv(const std::string& filename, 
                                   const std::vector<BenchmarkResult>& results);
    static void write_summary_to_csv(const std::string& filename,
                                   const std::vector<BenchmarkResult>& all_results);

    static void write_comparison_results_to_csv(const std::string& filename,
                                              const std::vector<BenchmarkResult>& cbs_results,
                                              const std::vector<BenchmarkResult>& jpscbs_results);
    static void write_comparison_summary_to_csv(const std::string& filename,
                                              const std::vector<BenchmarkResult>& cbs_results,
                                              const std::vector<BenchmarkResult>& jpscbs_results);

    // 创建CSV文件并返回文件流
    static std::ofstream create_csv_file(const std::string& filename);

    static void set_thread_affinity(int cpu_id);

    static int find_least_busy_cpu();
    static double get_cpu_load(int cpu_id);

    // 新增函数
    static void optimize_thread_priority();
    static void restore_thread_priority();
    
    #ifdef _WIN32
        static DWORD_PTR original_affinity;
        static int original_priority;
    #elif __linux__
        static cpu_set_t original_affinity;
        static int original_priority;
    #endif
};


#endif // BENCHMARK_UTILS_H