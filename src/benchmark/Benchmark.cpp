#include "BenchmarkUtils.h"
#include "../algorithm/cbs/CBS.h"
#include "../algorithm/jpscbs/JPSCBS.h"

int main() {
    try {
        CBS cbs(true);  // Enable optimization
        JPSCBS jpscbs;
        
        // 运行所有场景的比较测试
        BenchmarkUtils::run_all_scenarios_comparison(
            [&cbs](const std::vector<Agent>& agents, const std::vector<std::vector<int>>& grid) {
                return cbs.solve(agents, grid);
            },
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