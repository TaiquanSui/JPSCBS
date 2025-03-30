# Multi-Agent Path Finding with CBS and JPSCBS

This project implements and compares two Multi-Agent Path Finding (MAPF) algorithms: the classic Conflict-Based Search (CBS) with bypass improvement and a new algorithm JPSCBS improved by Muti-Solution Jump point Search based on CBS. For the theoretical part, see Master Thesis.pdf. A shorter theoretical explaination can be found in Localized_conflict_resolution.pdf

## Overview

The project provides:
- Standard implementation of CBS algorithm
- Enhanced JPSCBS algorithm with Jump Point Search optimization
- Comprehensive benchmarking framework
- Map visualization tools
- Detailed statistical analysis

## Prerequisites

- C++ 20
- CMake 3.14+
- Google Test (auto-downloaded)
- Python 3.x (for data analysis and visualization)
  - numpy
  - pandas
  - matplotlib
  - seaborn

## Project Structure 
```
├── src/ # Core algorithm implementations
├── data_loader/        # Map and scenario loaders
├── benchmark/          # Benchmarking code
├── test/               # Unit tests
├── data/               # Test data
│ ├── mapf-map/         # Map files
│ └── mapf-scen-/       # Test scenarios
└── benchmark_results/  # Test results output
└── plots/              # Generated comparison plots
```


## Building the Project

1. Clone the repository
```bash
git clone [repository-url]
cd [repository-name]
```

2. Create build directory
```bash
mkdir build && cd build
```

3. Build the project
```bash
cmake ..
make
```

## Running Tests

### Single Instance Tests
```bash
# Run CBS single test
./bin/cbs_single_test

# Run JPSCBS single test
./bin/jpscbs_single_test
```

### Benchmark Tests
```bash
# Run CBS benchmarks
./bin/cbs_benchmark

# Run JPSCBS benchmarks
./bin/jpscbs_benchmark

# Run comparison benchmarks
./bin/comparison_benchmark
```

### Analysis Tools
```bash
# Generate comparison summary
./bin/generate_comparison_summary

# Generate overall statistics
./bin/generate_overall_statistics
```

## Benchmark Results

The benchmark results are stored in the `benchmark_results` directory and include:
- Success rate comparison
- Runtime analysis
- Total path cost
- Number of expanded nodes
- Comparative plots and visualizations

Results are automatically generated in both CSV format and visual plots for easy analysis.

## Map Visualization

The project includes a Python-based visualization tool that can:
- Display map layouts
- Visualize agent paths
- Generate comparison charts
- Create statistical plots

## Configuration

Test scenarios can be configured in the benchmark files:
- Map selection (e.g., "Berlin_1_256", "random-64-64-10")
- Scenario types ("even" or "random")
- Number of agents
- Time limits
- Test parameters
