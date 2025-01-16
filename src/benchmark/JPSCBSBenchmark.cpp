#include "BenchmarkUtils.h"
#include "../algorithm/jpscbs/JPSCBS.h"

int main() {
    try {
        JPSCBS jpscbs;
        jpscbs.set_time_limit(300.0);  // 设置5分钟超时
        benchmark::run_all_scenarios(
            [&jpscbs](const std::vector<Agent>& agents, const std::vector<std::vector<int>>& grid) {
                return jpscbs.solve(agents, grid);
            }
        );
    } catch (const std::exception& e) {
        utils::log_error(e.what());
        return 1;
    }
    return 0;
} 