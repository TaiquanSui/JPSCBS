#include "BenchmarkUtils.h"
#include "../src/cbs/CBS.h"

int main() {
    try {
        auto cbs = std::make_unique<CBS>(true);  // Enable optimization
        
        logger::log_info("Starting benchmark for all scenarios");
        BenchmarkUtils::benchmark_all_scenarios(cbs.get());
        logger::log_info("Finished all scenarios");
    } catch (const std::exception& e) {
        logger::log_error("Exception in main: " + std::string(e.what()));
        return 1;
    }
    return 0;
} 