#ifndef BENCHMARK_UTILS_H
#define BENCHMARK_UTILS_H

#include "../algorithm/Agent.h"
#include "../data_loader/MapLoader.h"
#include "../algorithm/utilities/Utility.h"
#include "../algorithm/utilities/Log.h"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <filesystem>
#include <string>
#include <functional>

namespace benchmark {
    namespace fs = std::filesystem;

    // Get project root directory
    inline std::string get_project_root() {
        fs::path current_path = fs::current_path();
        while (!fs::exists(current_path / "data")) {
            if (current_path.parent_path() == current_path) {
                throw std::runtime_error("Cannot find project root directory");
            }
            current_path = current_path.parent_path();
        }
        return current_path.string();
    }

    // Extract map name from map file path
    inline std::string get_map_name(const std::string& map_path) {
        fs::path path(map_path);
        return path.stem().string();  // Use stem() to get filename without extension
    }

    // Build scenario file path
    inline std::string make_scen_path(const std::string& scen_dir, 
                                    const std::string& map_name,
                                    const std::string& type,
                                    int index) {
        return (fs::path(scen_dir) / 
                (map_name + "-" + type + "-" + std::to_string(index) + ".scen")).string();
    }

    using SolverFunction = std::function<std::vector<std::vector<Vertex>>(
        const std::vector<Agent>&, const std::vector<std::vector<int>>&)>;

    struct BenchmarkResult {
        bool success;
        double runtime;
        int total_cost;
        size_t num_agents;
    };

    // Print result
    inline void print_result(const std::string& map_file, 
                           const std::string& scen_file,
                           const BenchmarkResult& result) {
        std::cout << std::fixed << std::setprecision(2)
                  << "Results:\n"
                  << "  Map: " << map_file << "\n"
                  << "  Scenario: " << scen_file << "\n"
                  << "  Total Agents: " << result.num_agents << "\n"
                  << "  Success: " << (result.success ? "Yes" : "No") << "\n"
                  << "  Runtime: " << result.runtime << " seconds\n"
                  << "  Solution Cost: " << result.total_cost << "\n"
                  << "----------------------------------------" << std::endl;
    }

    // Run benchmark for a single scenario
    BenchmarkResult run_scenario(const std::string& map_file, 
                               const std::string& scen_file,
                               SolverFunction solver) {
        try {
            auto grid = load_map(map_file);
            auto scenarios = load_scen(scen_file);
            
            auto start_time = std::chrono::high_resolution_clock::now();
            auto solution = solver(scenarios, grid);
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start_time).count();
                
            int total_cost = 0;
            if (!solution.empty()) {
                total_cost = std::accumulate(solution.begin(), solution.end(), 0,
                    [](int sum, const auto& path) { 
                        return sum + utils::calculate_path_cost(path); 
                    });
            }
            
            BenchmarkResult result{
                !solution.empty(),
                duration / 1000.0,
                total_cost,
                scenarios.size()
            };

            print_result(map_file, scen_file, result);
            return result;
                      
        } catch (const std::exception& e) {
            logger::log_error("Error testing " + map_file + " with " + scen_file + ": " + e.what());
            throw;
        }
    }

    // Run benchmark for all scenarios
    std::vector<BenchmarkResult> run_all_scenarios(SolverFunction solver) {
        std::vector<BenchmarkResult> results;
        
        try {
            std::string root_dir = get_project_root();
            fs::path data_dir = fs::path(root_dir) / "data";
            fs::path maps_dir = data_dir / "mapf-map";
            fs::path random_scen_dir = data_dir / "mapf-scen-random";
            fs::path even_scen_dir = data_dir / "mapf-scen-even";
            
            if (!fs::exists(maps_dir)) {
                throw std::runtime_error("Maps directory not found: " + maps_dir.string());
            }
            
            for (const auto& map_entry : fs::directory_iterator(maps_dir)) {
                if (map_entry.path().extension() == ".map") {
                    std::string map_path = map_entry.path().string();
                    std::string map_name = get_map_name(map_path);
                    logger::log_info("Testing map: " + map_name);
                    
                    // Test random and even scenarios
                    struct ScenType {
                        const char* name;
                        fs::path dir;
                    };

                    for (const ScenType& scenario : {
                        ScenType{"even", even_scen_dir},
                        ScenType{"random", random_scen_dir}
                    }) {
                        for (int i = 1; i <= 25; ++i) {
                            std::string scen_file = make_scen_path(scenario.dir.string(), 
                                                                 map_name, scenario.name, i);
                            if (fs::exists(scen_file)) {
                                logger::log_info("Testing " + std::string(scenario.name) + " scenario " + 
                                             std::to_string(i));
                                results.push_back(run_scenario(map_path, scen_file, solver));
                            }
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            logger::log_error(e.what());
            throw;
        }
        
        return results;
    }
}

#endif // BENCHMARK_UTILS_H 