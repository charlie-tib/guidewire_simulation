import numpy as np
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver
import os

# 使用非交互式后端
import matplotlib
matplotlib.use('Agg')

def test_sensitivity():
    # 几何参数
    len_shaft = 0.020 
    len_soft = 0.035
    len_mag = 0.010
    
    moduli = [10e6, 1e6] # 对比 10MPa 和 1MPa
    colors = ['green', 'blue']
    labels = ['10 MPa', '1 MPa']
    
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111)
    
    for E_soft, color, label in zip(moduli, colors, labels):
        segments = [
            {'length': len_shaft, 'E': 200e6, 'is_magnet': False}, 
            {'length': len_soft, 'E': E_soft, 'is_magnet': False},
            {'length': len_mag, 'E': 200e6, 'is_magnet': True, 'polarity': 1.0},
            {'length': len_soft, 'E': E_soft, 'is_magnet': False},
            {'length': len_mag, 'E': 200e6, 'is_magnet': True, 'polarity': -1.0},
            {'length': len_soft, 'E': E_soft, 'is_magnet': False},
            {'length': len_mag, 'E': 200e6, 'is_magnet': True, 'polarity': 1.0}
        ]
        
        solver = DVSMagneticRodSolver(segments)
        solver.base_theta = np.pi / 2
        solver.g_vec = np.array([0, 0, 9.81])
        
        n_seg = solver.N - 1
        x0 = np.zeros(n_seg * 2)
        x0[:n_seg] = np.linspace(np.pi/2, np.pi * 0.8, n_seg)
        
        sol, _ = solver.solve(epm_pos=np.zeros(3), epm_m=np.zeros(3), maxiter=500, ftol=1e-5, x0=x0)
        pts = solver.get_geometry(sol)
        
        ax.plot(pts[:, 0], pts[:, 2], color=color, linewidth=3, label=label)
        print(f"E={E_soft/1e6:.1f}MPa, Tip X={pts[-1,0]:.4f}, Tip Z={pts[-1,2]:.4f}")

    # 固定坐标轴范围，防止 Auto-scaling 误导
    ax.set_xlim(-0.01, 0.16)
    ax.set_ylim(-0.16, 0.05)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_title("Modulus Sensitivity Test (Fixed Axes)")
    
    save_path = "/home/wen-zheng/guidewire_simulation/modulus_comparison.png"
    plt.savefig(save_path)
    print(f"Comparison plot saved to {save_path}")

if __name__ == "__main__":
    test_sensitivity()
