#ifndef BENCHMARK_UTILS_H
#define BENCHMARK_UTILS_H

#include "../algorithm/Agent.h"
#include "../data_loader/MapLoader.h"
#include "../algorithm/utilities/Utility.h"
#include "../algorithm/utilities/Log.h"
#include "../algorithm/cbs/CBS.h"
#include "../algorithm/jpscbs/JPSCBS.h"
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

#ifdef __linux__
    #include <pthread.h>
#elif _WIN32
    #define NOMINMAX
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
#endif

namespace fs = std::filesystem;

using SolverFunction = std::function<std::vector<std::vector<Vertex>>(
    const std::vector<Agent>&, const std::vector<std::vector<int>>&)>;

struct BenchmarkResult {
    bool success;
    double runtime;
    int total_cost;
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

class ThreadSafeInterruptor {
private:
    std::atomic<bool> interrupted{false};
    std::mutex mtx;
    std::condition_variable cv;

public:
    void interrupt();
    bool is_interrupted() const;
    void reset();
    
    template<typename Duration>
    bool wait_for(Duration duration) {
        std::unique_lock<std::mutex> lock(mtx);
        return !cv.wait_for(lock, duration, [interrupted = &this->interrupted] { 
            return interrupted->load(); 
        });
    }
};

class BenchmarkUtils {
public:
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
                                              
    static void set_thread_affinity(int cpu_id);
    
    static std::vector<BenchmarkResult> run_scen_file(const std::string& map_file,
                                                     const std::string& scen_file,
                                                     SolverFunction solver,
                                                     double time_limit = 30.0);
                                                     
    static std::pair<std::vector<BenchmarkResult>, std::vector<BenchmarkResult>> 
    run_algorithm_comparison(const std::string& map_file,
                           const std::string& scen_file,
                           SolverFunction cbs_solver,
                           SolverFunction jpscbs_solver,
                           double time_limit = 30.0,
                           int cpu_core = 0);
                           
    static void run_all_scenarios(SolverFunction solver);
    static void run_all_scenarios_comparison(SolverFunction solver1, SolverFunction solver2);
};

#endif // BENCHMARK_UTILS_H