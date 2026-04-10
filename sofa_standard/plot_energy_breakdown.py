import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver
import os

def run_energy_components(mode=3):
    segments = []
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
    epm_pos = np.array([0.08, 0.0, 0.055])
    angles = np.linspace(0, 360, 72)
    
    el_ens = []
    mag_ens = []
    tot_ens = []
    
    for ang in angles:
        ang_rad = np.radians(ang)
        epm_m = np.array([np.sin(ang_rad), 0, np.cos(ang_rad)]) * 5.0
        
        sol, tot = solver.solve(epm_pos=epm_pos, epm_m=epm_m)
        
        # Calculate individual components
        # Elastic Energy: sum(0.5 * K * theta^2)
        e_el = solver.total_energy(sol, epm_pos=epm_pos, epm_m=np.zeros(3))
        e_mag = tot - e_el
        
        el_ens.append(e_el)
        mag_ens.append(e_mag)
        tot_ens.append(tot)
        
    return angles, el_ens, mag_ens, tot_ens

if __name__ == "__main__":
    a3, el3, mg3, tot3 = run_energy_components(mode=3)
    a1, el1, mg1, tot1 = run_energy_components(mode=1)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # 3-Magnets (+-+) Components
    ax1.plot(a3, el3, 'g--', label='Elastic (Stiffness Penalty)')
    ax1.plot(a3, mg3, 'r:', label='Magnetic (Interaction Benefit)')
    ax1.plot(a3, tot3, 'b-', linewidth=2, label='Total Potential Energy')
    ax1.set_title("3-Magnets (+-+): Energy Component Breakdown", fontsize=14)
    ax1.set_ylabel("Energy (J)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 1-Magnet Components
    ax2.plot(a1, el1, 'g--', label='Elastic (Stiffness Penalty)')
    ax2.plot(a1, mg1, 'r:', label='Magnetic (Interaction Benefit)')
    ax2.plot(a1, tot1, 'b-', linewidth=2, label='Total Potential Energy')
    ax2.set_title("1-Magnet (Control): Energy Component Breakdown", fontsize=14)
    ax2.set_xlabel("EPM Angle (deg)")
    ax2.set_ylabel("Energy (J)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig("metric1_energy_breakdown.png")
    print("分析完成，结果已保存至 metric1_energy_breakdown.png")
