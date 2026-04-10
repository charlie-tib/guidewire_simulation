"""
Plot navigation test results: compare 1-Magnet vs 3-Magnets performance.
Reads summary CSV files and generates comparison bar charts.
"""
import numpy as np
import csv
import matplotlib
matplotlib.use('Agg') # Non-interactive backend
import matplotlib.pyplot as plt
import os

MAX_DEVIATION_MM = 15.0  # Matches current run_navigation_test.py

def load_summary(filename):
    """Load nav_summary CSV into a dict of lists."""
    if not os.path.exists(filename):
        print(f"Warning: {filename} not found")
        return None
    data = {'Trial': [], 'Success': [], 'Steps': [], 'WP_Reached': [], 
            'WP_Total': [], 'PeakForce_mN': [], 'CumForceImpulse_mNs': [], 'RMSE_mm': []}
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['Trial'].append(int(row['Trial']))
            data['Success'].append(int(row['Success']))
            data['Steps'].append(int(row['Steps']))
            data['WP_Reached'].append(int(row['WP_Reached']))
            data['WP_Total'].append(int(row['WP_Total']))
            data['PeakForce_mN'].append(float(row['PeakForce_mN']))
            data['CumForceImpulse_mNs'].append(float(row['CumForceImpulse_mNs']))
            data['RMSE_mm'].append(float(row['RMSE_mm']))
    return data

def load_timeseries(filename):
    """Load per-step timeseries CSV."""
    if not os.path.exists(filename):
        return None
    data = {'Step': [], 'Deviation_mm': [], 'ContactForce_mN': [], 'Insertion_mm': []}
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['Step'].append(int(row['Step']))
            data['Deviation_mm'].append(float(row['Deviation_mm']))
            data['ContactForce_mN'].append(float(row['ContactForce_mN']))
            data['Insertion_mm'].append(float(row['Insertion_mm']))
    return data

# Try to load data for both vessel types
vessels = ['synthetic', 'real']
modes = {'1mag': '1-Magnet', '3mag': '3-Magnets (+1,-1,+1)'}
colors = {'1mag': '#E74C3C', '3mag': '#3498DB'}

for vessel in vessels:
    summaries = {}
    has_data = False
    
    for mode_key, mode_label in modes.items():
        fname = f"nav_summary_{mode_key}_{vessel}.csv"
        data = load_summary(fname)
        if data and len(data['Trial']) > 0:
            summaries[mode_key] = data
            has_data = True
    
    if not has_data:
        print(f"No data found for vessel={vessel}, skipping.")
        continue
    
    print(f"\n{'='*60}")
    print(f"  Results for {vessel} vessel")
    print(f"{'='*60}")
    
    # === Figure: 4-panel comparison ===
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Navigation Performance: {vessel.capitalize()} Vessel', fontsize=16, fontweight='bold')
    
    bar_width = 0.35
    x_positions = np.arange(len(summaries))
    labels = []
    
    # Panel 1: Success Rate
    ax1 = axes[0, 0]
    for idx, (mode_key, data) in enumerate(summaries.items()):
        sr = np.mean(data['Success']) * 100
        ax1.bar(idx, sr, bar_width * 2, color=colors.get(mode_key, 'gray'), 
               label=modes[mode_key], alpha=0.8)
        ax1.text(idx, sr + 2, f'{sr:.0f}%', ha='center', fontweight='bold', fontsize=12)
        labels.append(modes[mode_key])
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title('① Navigability (Success Rate)')
    ax1.set_xticks(range(len(labels)))
    ax1.set_xticklabels(labels, fontsize=9)
    ax1.set_ylim(0, 120)
    ax1.axhline(y=100, color='green', linestyle='--', alpha=0.3)
    
    # Panel 2: Peak Contact Force
    ax2 = axes[0, 1]
    for idx, (mode_key, data) in enumerate(summaries.items()):
        mean_f = np.mean(data['PeakForce_mN'])
        std_f = np.std(data['PeakForce_mN']) if len(data['PeakForce_mN']) > 1 else 0
        ax2.bar(idx, mean_f, bar_width * 2, yerr=std_f, color=colors.get(mode_key, 'gray'),
               alpha=0.8, capsize=5)
        ax2.text(idx, mean_f + std_f + 0.5, f'{mean_f:.1f}', ha='center', fontsize=10)
    ax2.set_ylabel('Peak Contact Force (mN)')
    ax2.set_title('② Safety (Peak Contact Force)')
    ax2.set_xticks(range(len(labels)))
    ax2.set_xticklabels(labels, fontsize=9)
    
    # Panel 3: Completion Steps
    ax3 = axes[1, 0]
    for idx, (mode_key, data) in enumerate(summaries.items()):
        mean_s = np.mean(data['Steps'])
        std_s = np.std(data['Steps']) if len(data['Steps']) > 1 else 0
        ax3.bar(idx, mean_s, bar_width * 2, yerr=std_s, color=colors.get(mode_key, 'gray'),
               alpha=0.8, capsize=5)
        ax3.text(idx, mean_s + std_s + 100, f'{mean_s:.0f}', ha='center', fontsize=10)
    ax3.set_ylabel('Steps to Completion')
    ax3.set_title('③ Efficiency (Completion Steps)')
    ax3.set_xticks(range(len(labels)))
    ax3.set_xticklabels(labels, fontsize=9)
    
    # Panel 4: RMSE
    ax4 = axes[1, 1]
    for idx, (mode_key, data) in enumerate(summaries.items()):
        mean_r = np.mean(data['RMSE_mm'])
        std_r = np.std(data['RMSE_mm']) if len(data['RMSE_mm']) > 1 else 0
        ax4.bar(idx, mean_r, bar_width * 2, yerr=std_r, color=colors.get(mode_key, 'gray'),
               alpha=0.8, capsize=5)
        ax4.text(idx, mean_r + std_r + 0.2, f'{mean_r:.2f}', ha='center', fontsize=10)
    ax4.set_ylabel('Path Deviation RMSE (mm)')
    ax4.set_title('④ Accuracy (Path Tracking Error)')
    ax4.set_xticks(range(len(labels)))
    ax4.set_xticklabels(labels, fontsize=9)
    
    plt.tight_layout()
    plt.savefig(f'nav_comparison_{vessel}.png', dpi=150, bbox_inches='tight')
    print(f"  Saved: nav_comparison_{vessel}.png")
    # plt.show() # Commented out for automated run

    # --- Print comparison table ---
    for mode_key, data in summaries.items():
        n = len(data['Trial'])
        print(f"\n  [{modes[mode_key]}] ({n} trials)")
        print(f"    Success Rate: {np.mean(data['Success'])*100:.0f}%")
        print(f"    Mean Steps: {np.mean(data['Steps']):.0f} ± {np.std(data['Steps']):.0f}")
        print(f"    Peak Force: {np.mean(data['PeakForce_mN']):.2f} ± {np.std(data['PeakForce_mN']):.2f} mN")
        print(f"    RMSE: {np.mean(data['RMSE_mm']):.2f} ± {np.std(data['RMSE_mm']):.2f} mm")

# Also plot timeseries for the first trial if available
for vessel in vessels:
    for mode_key in modes:
        ts_file = f"nav_results_{mode_key}_{vessel}_trial1.csv"
        ts = load_timeseries(ts_file)
        if ts is None:
            continue
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        fig.suptitle(f'{modes[mode_key]} - {vessel.capitalize()} Vessel (Trial 1)', fontsize=14)
        
        ax1.plot(ts['Step'], ts['ContactForce_mN'], color=colors[mode_key], alpha=0.7, linewidth=0.8)
        ax1.set_ylabel('Contact Force (mN)')
        ax1.set_title('Contact Force over Time')
        ax1.grid(True, alpha=0.3)
        
        ax2.plot(ts['Step'], ts['Deviation_mm'], color=colors[mode_key], alpha=0.7, linewidth=0.8)
        ax2.axhline(y=MAX_DEVIATION_MM, color='red', linestyle='--', alpha=0.5, label=f'Fail threshold ({MAX_DEVIATION_MM}mm)')
        ax2.set_ylabel('Path Deviation (mm)')
        ax2.set_xlabel('Simulation Step')
        ax2.set_title('Path Deviation over Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'nav_timeseries_{mode_key}_{vessel}.png', dpi=150, bbox_inches='tight')
        # plt.show() # Commented out for automated run

