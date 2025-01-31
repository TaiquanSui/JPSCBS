#include "BenchmarkUtils.h"
#include "../algorithm/jpscbs/JPSCBS.h"

int main() {
    try {
        JPSCBS jpscbs;
        jpscbs.set_time_limit(30.0);  // Set 30 seconds timeout
        BenchmarkUtils::run_all_scenarios(
            [&jpscbs](const std::vector<Agent>& agents, const std::vector<std::vector<int>>& grid) {
                return jpscbs.solve(agents, grid);
            }
        );
    } catch (const std::exception& e) {
        logger::log_error(e.what());
        return 1;
    }
    return 0;
} 