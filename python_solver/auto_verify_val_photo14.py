import numpy as np
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver
import os
import pandas as pd
from scipy.interpolate import interp1d

# 1. 加载实验数据
real_data = pd.read_csv('/home/wen-zheng/guidewire_simulation/centerline_real.csv')
exp_x = real_data['x_mm'].values / 1000.0
exp_y = real_data['y_mm'].values / 1000.0

# 加载 EPM 位置
with open('/home/wen-zheng/guidewire_simulation/epm_coords_val.txt', 'r') as f:
    lines = f.readlines()
    epm_x = float(lines[0].split('=')[1]) / 1000.0
    epm_y = float(lines[1].split('=')[1]) / 1000.0
epm_pos = np.array([epm_x, epm_y, 0.0])

print(f"DEBUG: exp_x range [{exp_x[0]:.3f}, {exp_x[-1]:.3f}]")

# 2. 配置仿真器
scale_factor = exp_x[-1] / 0.110
modulus_soft = 2e5
modulus_hard = 2e9

# 修改分段比例以更好适应 168mm 的实际表现
# 如果 110mm 是 80(轴) + 30(软/磁)，168mm 按比例分配
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

# 3. 搜索最佳角度 (增量打印)
best_rmse = float('inf')
best_angle = 0
best_pts = None

print("\n--- 正在搜索最佳磁铁角度 ---")
for ang_deg in range(0, 360, 20):
    ang_rad = np.deg2rad(ang_deg)
    M_strength = 20.0
    epm_m = np.array([M_strength * np.cos(ang_rad), M_strength * np.sin(ang_rad), 0.0])
    
    sol, _ = solver.solve(epm_pos=epm_pos, epm_m=epm_m, maxiter=30)
    sim_pts = solver.get_geometry(sol)
    
    interp_func = interp1d(exp_x, exp_y, bounds_error=False, fill_value="extrapolate")
    sim_x = sim_pts[:, 0]
    target_y = interp_func(sim_x)
    
    rmse = np.sqrt(np.mean((sim_pts[:, 1] - target_y)**2))
    print(f"Angle: {ang_deg:>3} deg | RMSE: {rmse*1000:7.2f} mm")
    
    if rmse < best_rmse:
        best_rmse = rmse
        best_angle = ang_deg
        best_pts = sim_pts

# 4. 可视化
plt.figure(figsize=(10, 6))
plt.plot(exp_x*1000, exp_y*1000, 'ko', markersize=3, label='Real (Photo 14)')
plt.plot(best_pts[:, 0]*1000, best_pts[:, 1]*1000, 'r-', linewidth=2, label=f'Best Sim @ {best_angle}deg')
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.savefig('validation_result_photo14.png')
print(f"\n最佳拟合 RMSE: {best_rmse*1000:.2f} mm")
