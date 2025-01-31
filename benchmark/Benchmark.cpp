#include "BenchmarkUtils.h"
#include "../src/cbs/CBS.h"
#include "../src/jpscbs/JPSCBS.h"

int main() {
    try {
        // 创建算法实例
        CBS cbs(true);  // Enable optimization
        JPSCBS jpscbs;
        
        // 创建中断器
        ThreadSafeInterruptor interruptor1, interruptor2;
        
        // 设置算法的中断器
        cbs.set_interruptor(&interruptor1);
        jpscbs.set_interruptor(&interruptor2);
        
        // 运行所有场景的比较测试
        BenchmarkUtils::benchmark_all_scenarios_comparison(
            [&cbs](const std::vector<Agent>& agents, const std::vector<std::vector<int>>& grid) {
                return cbs.solve(agents, grid);
            },
            [&interruptor1]() { interruptor1.interrupt(); },
            [&jpscbs](const std::vector<Agent>& agents, const std::vector<std::vector<int>>& grid) {
                return jpscbs.solve(agents, grid);
            },
            [&interruptor2]() { interruptor2.interrupt(); }
        );
        
    } catch (const std::exception& e) {
        logger::log_error(e.what());
        return 1;
    }
    return 0;
} 