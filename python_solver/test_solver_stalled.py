import numpy as np
from mcr_python_solver import DVSMagneticRodSolver

modulus_soft = 10e6
modulus_hard = 200e6

segments = [
    {'length': 0.080, 'E': 50e9, 'is_magnet': False}, 
    {'length': 0.035, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.010, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0},
    {'length': 0.035, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.010, 'E': modulus_hard, 'is_magnet': True, 'polarity': -1.0},
    {'length': 0.035, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.010, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0}
]

print("Initializing Solver (215mm/5mm)...")
solver = DVSMagneticRodSolver(segments)
print(f"Nodes: {solver.N}, DL: {solver.dl}")

solver.base_theta = np.pi/2
solver.g_vec = np.array([0, 0, 9.81])

# Initial Guess
n_seg = solver.N - 1
x0 = np.zeros(n_seg * 2)
x0[:n_seg] = np.linspace(np.pi/2, np.pi, n_seg)

print("Starting Solver (Verbose)...")
sol, _ = solver.solve(maxiter=500, ftol=1e-5, x0=x0, disp=True)
print("Solver Finished Successfully.")

sim_pts = solver.get_geometry(sol)

# RMSE Calculation
import pandas as pd
import os
csv_path = 'centerline_real.csv'
if os.path.exists(csv_path):
    df = pd.read_csv(csv_path)
    real_x = df['x_mm'].values / 1000.0
    real_z = df['y_mm'].values / -1000.0 
    
    # 使用弧长插值对齐
    dx = np.diff(real_x); dz = np.diff(real_z)
    real_s = np.concatenate(([0], np.cumsum(np.sqrt(dx**2 + dz**2))))
    
    sim_dx = np.diff(sim_pts[:,0]); sim_dz = np.diff(sim_pts[:,2])
    sim_s = np.concatenate(([0], np.cumsum(np.sqrt(sim_dx**2 + sim_dz**2))))
    
    # 仅在重叠区间对比
    max_s = min(real_s[-1], sim_s[-1])
    eval_s = np.linspace(0, max_s, 20)
    
    from scipy.interpolate import interp1d
    fx_real = interp1d(real_s, real_x); fz_real = interp1d(real_s, real_z)
    fx_sim = interp1d(sim_s, sim_pts[:,0]); fz_sim = interp1d(sim_s, sim_pts[:,2])
    
    rmse = np.sqrt(np.mean((fx_real(eval_s)-fx_sim(eval_s))**2 + (fz_real(eval_s)-fz_sim(eval_s))**2))
    print(f"\n>>> [RMSE CHECK] Current RMSE: {rmse*1000:.2f} mm (based on {max_s*1000:.1f}mm overlap)")
else:
    print("CSV not found.")
