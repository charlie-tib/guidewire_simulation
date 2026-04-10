import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
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

solver = DVSMagneticRodSolver(segments)
solver.base_theta = np.pi/2
solver.g_vec = np.array([0, 0, 9.81])

n_seg = solver.N - 1
x0 = np.zeros(n_seg * 2)
x0[:n_seg] = np.linspace(np.pi/2, np.pi, n_seg)

sol, _ = solver.solve(maxiter=500, ftol=1e-5, x0=x0)
sim_pts = solver.get_geometry(sol)

# Plotting
csv_path = 'centerline_real.csv'
df = pd.read_csv(csv_path)
real_x = df['x_mm'].values / 1000.0
real_z = df['y_mm'].values / -1000.0 

plt.figure(figsize=(10, 6))
plt.plot(sim_pts[:,0], sim_pts[:,2], 'g-', label='Simulation (215mm)')
plt.plot(real_x, real_z, 'r--', label='Experiment (Centerline)')
plt.legend()
plt.title("Gravity Droop Comparison (Metric 1 Alignment)")
plt.xlabel("X (m)")
plt.ylabel("Z (m)")
plt.grid(True)
plt.axis('equal')

save_path = '/home/wen-zheng/.gemini/antigravity/brain/d311d1c0-5266-44e5-993e-2bb8df2fdf0c/gravity_full_comparison.png'
plt.savefig(save_path)
print(f"Plot saved to {save_path}")
