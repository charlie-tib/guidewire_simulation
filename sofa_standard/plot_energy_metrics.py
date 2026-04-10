"""
Plotting Script for Metric 1: Energy and Curvature Analysis
Visualizes:
1. Curvature vs. Arc Length (Active deformation points)
2. Elastic Energy vs. Field Angle
3. Total Potential Energy Map
"""
import csv
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_energy_comparison():
    files = {
        '1-Magnet': 'energy_results_1mag.csv',
        '3-Magnets': 'energy_results_3mag.csv'
    }
    
    # 1. Curvature vs Arc Length at a specific angle (e.g. 90 deg)
    plt.figure(figsize=(10, 6))
    for label, filename in files.items():
        if os.path.exists(filename):
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                s_mm, curvature = [], []
                for row in reader:
                    if int(float(row['Angle'])) == 90:
                        s_mm.append(float(row['S_mm']))
                        curvature.append(float(row['Curvature']))
                plt.plot(s_mm, curvature, label=label, linewidth=2)
            
    plt.title(f'Curvature Profile along Arc Length (Field Angle = 90°)', fontsize=14)
    plt.xlabel('Arc Length (mm)', fontsize=12)
    plt.ylabel('Curvature (1/m)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.savefig('metric1_curvature_profile.png', dpi=300)
    print("Saved metric1_curvature_profile.png")

    # 2. Total Elastic Energy vs Angle
    plt.figure(figsize=(10, 6))
    for label, filename in files.items():
        if os.path.exists(filename):
            energy_map = {} # Angle -> Sum(ElasticEnergy)
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    ang = int(float(row['Angle']))
                    energy_map[ang] = energy_map.get(ang, 0) + float(row['ElasticEnergy'])
            angles = sorted(energy_map.keys())
            energies = [energy_map[a] * 1e6 for a in angles]
            plt.plot(angles, energies, label=label, marker='o', markersize=4)
            
    plt.title('Total Elastic Strain Energy vs Field Angle', fontsize=14)
    plt.xlabel('Magnetic Field Angle (degrees)', fontsize=12)
    plt.ylabel('Elastic Energy (uJ)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.savefig('metric1_energy_vs_angle.png', dpi=300)
    print("Saved metric1_energy_vs_angle.png")

    # 3. Total Potential Energy Map (E_el + E_mag)
    plt.figure(figsize=(10, 6))
    for label, filename in files.items():
        if os.path.exists(filename):
            total_e_map = {}
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    ang = int(float(row['Angle']))
                    total_e = float(row['ElasticEnergy']) + float(row['MagEnergy'])
                    total_e_map[ang] = total_e_map.get(ang, 0) + total_e
            angles = sorted(total_e_map.keys())
            energies = [total_e_map[a] * 1e6 for a in angles]
            # Normalize
            min_e = min(energies)
            energies = [e - min_e for e in energies]
            plt.plot(angles, energies, label=label)
            
    plt.title('Total Potential Energy Landscape (Normalized)', fontsize=14)
    plt.xlabel('Magnetic Field Angle (degrees)', fontsize=12)
    plt.ylabel('Relative Potential Energy (uJ)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.savefig('metric1_potential_landscape.png', dpi=300)
    print("Saved metric1_potential_landscape.png")

if __name__ == "__main__":
    plot_energy_comparison()

if __name__ == "__main__":
    plot_energy_comparison()
