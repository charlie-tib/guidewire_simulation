import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from mcr_python_solver import DVSMagneticRodSolver
import os

class Fig2bReplicator(DVSMagneticRodSolver):
    def solve_constrained(self, target_theta_deg, epm_pos, epm_m):
        target_theta_rad = np.radians(target_theta_deg)
        n_vars = (self.N - 1)
        x0 = np.ones(n_vars) * (target_theta_rad / n_vars)
        cons = ({'type': 'eq', 'fun': lambda x: np.sum(x) - target_theta_rad})
        def objective(x):
            state = np.zeros(n_vars * 2)
            state[:n_vars] = x
            return self.total_energy(state, epm_pos, epm_m)
        res = minimize(objective, x0, constraints=cons, method='SLSQP', tol=1e-6)
        if res.success:
            state_final = np.zeros(n_vars * 2)
            state_final[:n_vars] = res.x
            e_el = self.total_energy(state_final, epm_pos, np.zeros(3))
            e_mag = res.fun - e_el
            return res.fun, e_el, e_mag
        return None, None, None

def run_compare_sweep(phi_deg):
    # 3-Mag
    segments3 = [
        {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
        {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
        {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0},
        {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
        {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': -1.0},
        {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
        {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
    ]
    # 1-Mag
    segments1 = [
        {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
        {'length': 0.080, 'E': 10e6, 'is_magnet': False}, 
        {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
    ]

    epm_pos = np.array([0.08, 0.0, 0.055])
    phi_rad = np.radians(phi_deg)
    epm_m = np.array([np.sin(phi_rad), 0, np.cos(phi_rad)]) * 5.0

    thetas = np.linspace(-180, 180, 91)
    
    rep3 = Fig2bReplicator(segments3)
    rep1 = Fig2bReplicator(segments1)
    
    res3 = {'theta': [], 'total': []}
    res1 = {'theta': [], 'total': []}
    
    print(f"Sweeping for phi = {phi_deg} deg...")
    for th in thetas:
        tot3, _, _ = rep3.solve_constrained(th, epm_pos, epm_m)
        tot1, _, _ = rep1.solve_constrained(th, epm_pos, epm_m)
        if tot3: res3['theta'].append(th); res3['total'].append(tot3)
        if tot1: res1['theta'].append(th); res1['total'].append(tot1)
            
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    ax1.plot(res3['theta'], res3['total'], 'b-', linewidth=2, label='3-Magnets (+-+)')
    # Find minima for 3-Mag
    t3 = np.array(res3['total'])
    for i in range(1, len(t3)-1):
        if t3[i] < t3[i-1] and t3[i] < t3[i+1]:
            ax1.scatter(res3['theta'][i], t3[i], color='blue', s=80)
            ax1.annotate(f"{res3['theta'][i]:.0f}°", (res3['theta'][i], t3[i]), textcoords="offset points", xytext=(0,10), ha='center')
    ax1.set_title(f"3-Magnets Energy Landscape (φ = {phi_deg}°)", fontsize=14)
    ax1.set_ylabel("Total Potential Energy (J)")
    ax1.legend(); ax1.grid(True, alpha=0.3)
    
    ax2.plot(res1['theta'], res1['total'], 'r-', linewidth=2, label='1-Magnet (Control)')
    # Find minima for 1-Mag
    t1 = np.array(res1['total'])
    for i in range(1, len(t1)-1):
        if t1[i] < t1[i-1] and t1[i] < t1[i+1]:
            ax2.scatter(res1['theta'][i], t1[i], color='red', s=80)
            ax2.annotate(f"{res1['theta'][i]:.0f}°", (res1['theta'][i], t1[i]), textcoords="offset points", xytext=(0,10), ha='center')
    ax2.set_title(f"1-Magnet Energy Landscape (φ = {phi_deg}°)", fontsize=14)
    ax2.set_xlabel("Bending Angle θ (deg)"); ax2.set_ylabel("Total Potential Energy (J)")
    ax2.legend(); ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig("metric1_fig2b_compare_random.png")
    print(f"分析完成，随机角度 φ = {phi_deg}°")

if __name__ == "__main__":
    # 随机选择一个角度，比如 135 度 (钝角)
    run_compare_sweep(135)
