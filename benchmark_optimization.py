#!/usr/bin/env python3
"""
Benchmark Original vs Kalibr Optimization Strategy
Runs all scenarios with both strategies and compares results.
"""
import subprocess
import os
import re
import yaml
from pathlib import Path

# Define scenarios
scenarios = [
    "calib_param_synth_Scenario1_DS",
    "calib_param_synth_Scenario2_DS",
    "calib_param_synth_Scenario3_DS",
    "calib_param_synth_Scenario4_DS",
    "calib_param_synth_Scenario5_DS",
    "calib_param_synth_diff_size_DS",
    "calib_param_synth_non_square_DS"
]

def extract_reprojection_error(log_text):
    """Extract mean reprojection error from calibration log."""
    # Look for pattern like "mean reprojection error :: X.XX"
    pattern = r"mean reprojection error\s*::\s*([\d.]+)"
    matches = re.findall(pattern, log_text)
    if matches:
        return float(matches[-1])  # Return last occurrence
    return None

def run_calibration(config_file, output_log):
    """Run calibration and return reprojection error."""
    cmd = f"./build/apps/calibrate/calibrate tests/configs_for_end2end_tests/{config_file}.yml"
    print(f"Running: {config_file}")
    
    try:
        result = subprocess.run(
            cmd, 
            shell=True, 
            cwd="/home/haruka_nanase/Desktop/Adv3D/My_Portfolio/MC-Calib",
            capture_output=True,
            text=True,
            timeout=300  # 5 minute timeout
        )
        
        # Save log
        with open(output_log, 'w') as f:
            f.write(result.stdout)
            f.write(result.stderr)
        
        # Extract error
        full_log = result.stdout + result.stderr
        error = extract_reprojection_error(full_log)
        return error, result.returncode
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT!")
        return None, -1
    except Exception as e:
        print(f"  ERROR: {e}")
        return None, -1

def main():
    results = []
    
    for scenario in scenarios:
        print(f"\n{'='*60}")
        print(f"Scenario: {scenario}")
        print(f"{'='*60}")
        
        # Run original
        original_log = f"/tmp/{scenario}_original.log"
        original_error, original_rc = run_calibration(scenario, original_log)
        
        # Run kalibr
        kalibr_config = f"{scenario}_Kalibr"
        kalibr_log = f"/tmp/{kalibr_config}.log"
        kalibr_error, kalibr_rc = run_calibration(kalibr_config, kalibr_log)
        
        # Store results
        results.append({
            'scenario': scenario.replace('calib_param_synth_', '').replace('_DS', ''),
            'original_error': original_error,
            'original_rc': original_rc,
            'kalibr_error': kalibr_error,
            'kalibr_rc': kalibr_rc
        })
        
        # Print comparison
        print(f"\nResults:")
        if original_error is not None:
            print(f"  Original: {original_error:.4f} px")
        else:
            print(f"  Original: FAILED")
        
        if kalibr_error is not None:
            print(f"  Kalibr:   {kalibr_error:.4f} px")
        else:
            print(f"  Kalibr:   FAILED")
        
        if original_error and kalibr_error:
            improvement = ((original_error - kalibr_error) / original_error) * 100
            print(f"  Improvement: {improvement:+.2f}%")
    
    # Print summary table
    print(f"\n\n{'='*80}")
    print("BENCHMARK SUMMARY")
    print(f"{'='*80}")
    print(f"{'Scenario':<20} {'Original (px)':<15} {'Kalibr (px)':<15} {'Improvement':<15}")
    print(f"{'-'*80}")
    
    for r in results:
        orig = f"{r['original_error']:.4f}" if r['original_error'] else "FAILED"
        kalib = f"{r['kalibr_error']:.4f}" if r['kalibr_error'] else "FAILED"
        
        if r['original_error'] and r['kalibr_error']:
            improvement = ((r['original_error'] - r['kalibr_error']) / r['original_error']) * 100
            imp_str = f"{improvement:+.2f}%"
        else:
            imp_str = "N/A"
        
        print(f"{r['scenario']:<20} {orig:<15} {kalib:<15} {imp_str:<15}")
    
    print(f"{'='*80}\n")

if __name__ == "__main__":
    main()
