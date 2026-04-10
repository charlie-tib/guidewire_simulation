import numpy as np
from mcr_python_solver import DVSMagneticRodSolver
import pandas as pd

print("Starting verification (One angle test)...")

real_data = pd.read_csv('/home/wen-zheng/guidewire_simulation/centerline_real.csv')
exp_x = real_data['x_mm'].values / 1000.0
exp_y = real_data['y_mm'].values / 1000.0

with open('/home/wen-zheng/guidewire_simulation/epm_coords_val.txt', 'r') as f:
    lines = f.readlines()
    epm_x = float(lines[0].split('=')[1]) / 1000.0
    epm_y = float(lines[1].split('=')[1]) / 1000.0
epm_pos = np.array([epm_x, epm_y, 0.0])

scale_factor = exp_x[-1] / 0.110
modulus_soft = 2e5
modulus_hard = 2e9

segs_3mag = [
    {'length': 0.080 * scale_factor, 'E': modulus_hard, 'is_magnet': False},
    {'length': 0.0067 * scale_factor, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.0033 * scale_factor, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0},
    {'length': 0.0067 * scale_factor, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.0033 * scale_factor, 'E': modulus_hard, 'is_magnet': True, 'polarity': -1.0},
    {'length': 0.0067 * scale_factor, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.0033 * scale_factor, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0}
]

solver = DVSMagneticRodSolver(segs_3mag)
solver.base_theta = np.pi/2
solver.g = 0.0

print("Solver initialized. Starting minimization...")
epm_m = np.array([20.0, 0.0, 0.0])
sol, _ = solver.solve(epm_pos=epm_pos, epm_m=epm_m, maxiter=50)
print("Minimization done.")
pts = solver.get_geometry(sol)
print(f"Tip position: {pts[-1]*1000} mm")
