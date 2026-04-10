import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver
import time
import os

def run_energy_sweep(mode=3):
    # 结构定义
    if mode == 3:
        segments = [
            {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0},
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': -1.0},
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
        ]
    else:
        segments = [
            {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
            {'length': 0.080, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
        ]

    solver = DVSMagneticRodSolver(segments)
    
    angles = np.linspace(0, 360, 72) # 5 deg steps
    results = {
        'angle': [],
        'e_total': [],
        'e_elastic': [],
        'e_magnetic': [],
        'tip_deflection': []
    }
    
    # EPM 放在侧向 150mm
    epm_pos = np.array([0.08, 0.0, 0.055])
    
    print(f"Sweeping Energy Landscape for Mode {mode}...")
    for ang_deg in angles:
        ang_rad = np.radians(ang_deg)
        epm_m = np.array([np.sin(ang_rad), 0, np.cos(ang_rad)]) * 5.0 
        
        sol, e_total = solver.solve(epm_pos=epm_pos, epm_m=epm_m)
        points = solver.get_geometry(sol)
        
        # 重新计算能量分量 (用于绘图)
        # 弹性能
        e_el = solver.total_energy(sol, epm_pos=epm_pos, epm_m=np.zeros(3)) 
        e_mag = e_total - e_el
        
        # Tip deflection (X-coord)
        tip_x = points[-1, 0]
        
        results['angle'].append(ang_deg)
        results['e_total'].append(e_total)
        results['e_elastic'].append(e_el)
        results['e_magnetic'].append(e_mag)
        results['tip_deflection'].append(tip_x)
        
    return results

if __name__ == "__main__":
    res3 = run_energy_sweep(mode=3)
    res1 = run_energy_sweep(mode=1)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # 1. Energy Landscape
    ax1.plot(res3['angle'], res3['e_total'], 'b-', linewidth=2, label='3nd-Mag Total Energy')
    # ax1.plot(res3['angle'], res3['e_elastic'], 'b--', alpha=0.5, label='3nd-Mag Elastic')
    # ax1.plot(res3['angle'], res3['e_magnetic'], 'b:', alpha=0.5, label='3nd-Mag Magnetic')
    
    ax1.plot(res1['angle'], res1['e_total'], 'r-', linewidth=2, label='1st-Mag Total Energy')
    
    ax1.set_title("Potential Energy Landscape vs. Magnetic Field Angle", fontsize=14)
    ax1.set_xlabel("EPM Angle (deg)")
    ax1.set_ylabel("Total Potential Energy (J)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. Tip Response (Kinematic curve)
    ax2.plot(res3['angle'], np.array(res3['tip_deflection'])*1000, 'b-', linewidth=2, label='3nd-Mag Tip X')
    ax2.plot(res1['angle'], np.array(res1['tip_deflection'])*1000, 'r--', linewidth=2, label='1st-Mag Tip X')
    
    ax2.set_title("Tip Deflection (X) vs. Magnetic Field Angle", fontsize=14)
    ax2.set_xlabel("EPM Angle (deg)")
    ax2.set_ylabel("Tip X Deflection (mm)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig("metric1_energy_vs_angle.png")
    print("分析完成，结果已保存至 metric1_energy_vs_angle.png")
