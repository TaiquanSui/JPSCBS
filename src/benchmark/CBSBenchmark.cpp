#include "BenchmarkUtils.h"
#include "../algorithm/cbs/CBS.h"

int main() {
    try {
        CBS cbs(true);  // Enable optimization
        benchmark::run_all_scenarios(
            [&cbs](const std::vector<Agent>& agents, const std::vector<std::vector<int>>& grid) {
                return cbs.solve(agents, grid);
            }
        );
    } catch (const std::exception& e) {
        logger::log_error(e.what());
        return 1;
    }
    return 0;
} 