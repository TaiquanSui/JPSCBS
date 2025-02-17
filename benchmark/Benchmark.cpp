#include "BenchmarkUtils.h"
#include "../src/cbs/CBS.h"
#include "../src/jpscbs/JPSCBS.h"

int main() {
    try {
        // 创建算法实例
        auto cbs = std::make_unique<CBS>(true);  // Enable optimization
        auto jpscbs = std::make_unique<JPSCBS>();
        
        // 运行所有场景的比较测试
        BenchmarkUtils::benchmark_all_scenarios_comparison(
            cbs.get(),
            jpscbs.get()
        );
        
    } catch (const std::exception& e) {
        logger::log_error(e.what());
        return 1;
    }
    return 0;
} 