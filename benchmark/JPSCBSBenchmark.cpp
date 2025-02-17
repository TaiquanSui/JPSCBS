#include "BenchmarkUtils.h"
#include "../src/jpscbs/JPSCBS.h"

int main() {
    try {
        auto jpscbs = std::make_unique<JPSCBS>();
        
        logger::log_info("Starting benchmark for all scenarios");
        BenchmarkUtils::benchmark_all_scenarios(jpscbs.get());
        logger::log_info("Finished all scenarios");
    } catch (const std::exception& e) {
        logger::log_error("Exception in main: " + std::string(e.what()));
        return 1;
    }
    return 0;
} 