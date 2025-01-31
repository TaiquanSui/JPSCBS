#include "BenchmarkUtils.h"
#include "../src/jpscbs/JPSCBS.h"

int main() {
    try {
        JPSCBS jpscbs;
        ThreadSafeInterruptor interruptor;
        
        jpscbs.set_interruptor(&interruptor);
        jpscbs.set_time_limit(30.0);  // Set 30 seconds timeout
        
        logger::log_info("Starting benchmark for all scenarios");
        BenchmarkUtils::run_all_scenarios(
            [&jpscbs](const std::vector<Agent>& agents, const std::vector<std::vector<int>>& grid) {
                return jpscbs.solve(agents, grid);
            },
            [&interruptor]() { interruptor.interrupt(); }
        );
        logger::log_info("Finished all scenarios");
    } catch (const std::exception& e) {
        logger::log_error("Exception in main: " + std::string(e.what()));
        return 1;
    }
    return 0;
} 