import numpy as np
import time
from mcr_python_solver import DVSMagneticRodSolver

segments = [{'length': 0.110, 'E': 100e6, 'is_magnet': True, 'polarity': 1.0}]
solver = DVSMagneticRodSolver(segments)
epm_pos = np.array([0, 0, 0.15])
epm_m = np.array([0, 10, 0]) * 0.1

print("Starting solve...")
start = time.time()
sol, energy = solver.solve(epm_pos=epm_pos, epm_m=epm_m)
print(f"Solve completed in {time.time() - start:.4f}s")
print(f"Final Energy: {energy}")
