#!/usr/bin/env python3
"""
Analyze FIFO RealSense Line Detection Test Results
Parses CSV files and generates summary comparison report
"""

import csv
import glob
import sys
from pathlib import Path

def parse_csv_summary(csv_file):
    """Extract summary statistics from CSV file"""
    stats = {}
    with open(csv_file, 'r') as f:
        for line in f:
            if line.startswith('# '):
                parts = line[2:].strip().split(',')
                if len(parts) == 2:
                    key = parts[0].strip()
                    try:
                        value = float(parts[1].strip())
                        stats[key] = value
                    except ValueError:
                        stats[key] = parts[1].strip()
    return stats

def analyze_results():
    """Analyze all FIFO test CSV files"""
    csv_files = sorted(glob.glob("fifo_realsense_houghlines_*.csv"))
    
    if not csv_files:
        print("ERROR: No CSV files found matching 'fifo_realsense_houghlines_*.csv'")
        print("Run the FIFO test first: sudo ./fifo_realsense_linedetect")
        return 1
    
    print("\n" + "="*70)
    print("FIFO REALSENSE LINE DETECTION TEST - ANALYSIS REPORT")
    print("="*70)
    
    results = []
    for csv_file in csv_files:
        resolution = Path(csv_file).stem.split('_')[-1]  # Extract resolution
        stats = parse_csv_summary(csv_file)
        
        if not stats:
            print(f"\nWARNING: Could not parse {csv_file}")
            continue
        
        results.append({
            'resolution': resolution,
            'file': csv_file,
            'stats': stats
        })
    
    # Print comparison table
    print("\n{:<12} {:>12} {:>12} {:>12} {:>12}".format(
        "Resolution", "Mean (ms)", "StdDev (ms)", "Max (ms)", "Miss %"
    ))
    print("-" * 70)
    
    for r in results:
        stats = r['stats']
        mean_ms = stats.get('Average exec time us', 0) / 1000.0
        std_ms = stats.get('Exec std dev us', 0) / 1000.0
        max_ms = stats.get('Max exec time us', 0) / 1000.0
        miss_pct = stats.get('Miss percentage', 0)
        
        print("{:<12} {:>12.2f} {:>12.2f} {:>12.2f} {:>11.2f}%".format(
            r['resolution'], mean_ms, std_ms, max_ms, miss_pct
        ))
    
    # Recommendations
    print("\n" + "="*70)
    print("RECOMMENDATIONS FOR LINE FOLLOWING")
    print("="*70)
    
    DEADLINE_MS = 50.0  # 50ms for 20 Hz control loop
    ACCEPTABLE_MISS_PCT = 5.0
    
    print(f"\nTarget: Mean exec time <{DEADLINE_MS}ms, Miss % <{ACCEPTABLE_MISS_PCT}%\n")
    
    suitable = []
    for r in results:
        stats = r['stats']
        mean_ms = stats.get('Average exec time us', 0) / 1000.0
        miss_pct = stats.get('Miss percentage', 0)
        
        meets_deadline = mean_ms < DEADLINE_MS
        meets_miss = miss_pct < ACCEPTABLE_MISS_PCT
        
        status = "✓ EXCELLENT" if (meets_deadline and meets_miss) else \
                 "✓ GOOD" if meets_deadline else \
                 "⚠ MARGINAL" if mean_ms < DEADLINE_MS * 1.2 else \
                 "✗ POOR"
        
        print(f"{r['resolution']:12} {status:12}  (Mean: {mean_ms:5.1f}ms, Miss: {miss_pct:4.1f}%)")
        
        if meets_deadline and meets_miss:
            suitable.append(r['resolution'])
    
    print("\n" + "="*70)
    print("RESOLUTION SELECTION")
    print("="*70)
    
    if suitable:
        print(f"\nSuitable resolutions for real-time line following:")
        for res in suitable:
            print(f"  ✓ {res}")
        
        print(f"\nRECOMMENDATION: Use {suitable[-1]} for best quality among suitable options")
        print(f"                 (Higher resolution = better line detection accuracy)")
    else:
        print("\n⚠ WARNING: No resolution meets both deadline and miss % requirements")
        print("  Consider:")
        print("    - Using lower resolution than tested (e.g., 160x120)")
        print("    - Optimizing algorithm (simpler edge detection)")
        print("    - Accepting higher deadline (e.g., 70ms for 15 Hz control)")
    
    # Additional insights
    print("\n" + "="*70)
    print("NEXT STEPS")
    print("="*70)
    print("\n1. Update ros_stream camera publisher to use recommended resolution")
    print("2. Consider Hough vs current Canny approach:")
    
    if results:
        lowest_res_stats = results[0]['stats']
        mean_ms = lowest_res_stats.get('Average exec time us', 0) / 1000.0
        print(f"   - Hough lines at {results[0]['resolution']}: ~{mean_ms:.1f}ms")
        print(f"   - Current Canny approach: estimated ~{mean_ms * 0.4:.1f}ms (40% of Hough)")
        print(f"   - Trade-off: Hough more accurate but {mean_ms / (mean_ms * 0.4):.1f}x slower")
    
    print("\n3. Run Phase 1-4: ROS monitoring to measure end-to-end latency")
    print("4. Validate <100ms total latency (camera → motor actuation)")
    
    print("\n" + "="*70)
    
    return 0

if __name__ == "__main__":
    sys.exit(analyze_results())
