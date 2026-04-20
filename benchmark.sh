#!/bin/bash
# Benchmark script for Original vs Kalibr optimization strategies

# Get absolute project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

echo "==================================================================="
echo "BENCHMARK: Original vs Kalibr Optimization Strategy"
echo "==================================================================="
echo ""

declare -a scenarios=(
    "Scenario1"
    "Scenario2"
    "Scenario3"
    "Scenario4"
    "Scenario5"
    "diff_size"
    "non_square"
)

results_file="/tmp/benchmark_results.txt"
echo "Scenario,Original_Error,Kalibr_Error,Improvement" > "$results_file"

for scenario in "${scenarios[@]}"; do
    echo "-------------------------------------------------------------------"
    echo "Running: $scenario"
    echo "-------------------------------------------------------------------"
    
    # Run Original
    echo "  [1/2] Original strategy..."
    orig_log="/tmp/${scenario}_original.log"
    ./build/apps/calibrate/calibrate "tests/configs_for_end2end_tests/calib_param_synth_${scenario}_DS.yml" > "$orig_log" 2>&1
    orig_error=$(grep "mean reprojection error" "$orig_log" | tail -1 | awk '{print $NF}')
    
    # Run Kalibr
    echo "  [2/2] Kalibr strategy..."
    kalib_log="/tmp/${scenario}_kalibr.log"
    ./build/apps/calibrate/calibrate "tests/configs_for_end2end_tests/calib_param_synth_${scenario}_DS_Kalibr.yml" > "$kalib_log" 2>&1
    kalib_error=$(grep "mean reprojection error" "$kalib_log" | tail -1 | awk '{print $NF}')
    
    # Calculate improvement
    if [ -n "$orig_error" ] && [ -n "$kalib_error" ]; then
        improvement=$(echo "scale=2; (($orig_error - $kalib_error) / $orig_error) * 100" | bc)
        echo "$scenario,$orig_error,$kalib_error,$improvement%" >> "$results_file"
        echo "  Original: $orig_error px"
        echo "  Kalibr:   $kalib_error px"
        echo "  Improvement: ${improvement}%"
    else
        echo "$scenario,FAILED,FAILED,N/A" >> "$results_file"
        echo "  FAILED"
    fi
    echo ""
done

echo "==================================================================="
echo "BENCHMARK SUMMARY"
echo "==================================================================="
column -t -s',' "$results_file"
echo ""
echo "Detailed logs saved to /tmp/*_{original,kalibr}.log"
