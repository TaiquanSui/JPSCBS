#include "BenchmarkUtils.h"
#include "../src/cbs/CBS.h"

int main() {
    try {
        CBS cbs(true);  // Enable optimization
        ThreadSafeInterruptor interruptor;
        
        cbs.set_interruptor(&interruptor);
        cbs.set_time_limit(30.0);  // Set 30 seconds timeout
        
        logger::log_info("Starting benchmark for all scenarios");
        BenchmarkUtils::run_all_scenarios(
            [&cbs](const std::vector<Agent>& agents, const std::vector<std::vector<int>>& grid) {
                return cbs.solve(agents, grid);
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