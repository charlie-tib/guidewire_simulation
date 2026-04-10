"""
反向拟合杨氏模量 E_soft
通过扫描 E_soft 候选值，找到使仿真下垂曲线与真实数据 RMSE 最小的值
"""
import numpy as np
import matplotlib.pyplot as plt
import os
from mcr_python_solver import DVSMagneticRodSolver

# ============================================================
# 加载真实数据并缩放到正确的 45mm 物理长度
# ============================================================
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
real_data = np.loadtxt(os.path.join(base_dir, 'logs_and_data/calibration/centerline_real.csv'), delimiter=',', skiprows=1)
# 计算提取曲线的当前弧长
dx = np.diff(real_data[:, 0])
dy = np.diff(real_data[:, 1])
s_calc = np.sum(np.sqrt(dx**2 + dy**2))
print(f"提取曲线的当前弧长: {s_calc:.2f} mm")

# 缩放因子，使弧长正好等于真实的 45mm
L_real_mm = 45.0
scale_factor = L_real_mm / s_calc
print(f"应用缩放因子: {scale_factor:.4f} (45mm / {s_calc:.2f}mm)")

real_x = real_data[:, 0] * scale_factor / 1000.0  # mm -> m
real_y = real_data[:, 1] * scale_factor / 1000.0  # mm -> m

# ============================================================
# 定义导丝结构：35mm 软段 + 10mm 磁铁 (3对组合)
# ============================================================
modulus_hard = 2e9  # 磁铁/NiTi段

def build_solver(E_soft):
    # 3对组合，总长 45mm
    len_soft = 0.035 / 3.0
    len_mag = 0.010 / 3.0
    segments = []
    for _ in range(3):
        segments.append({'length': len_soft, 'E': E_soft, 'is_magnet': False})
        segments.append({'length': len_mag, 'E': modulus_hard, 'is_magnet': True})
    
    solver = DVSMagneticRodSolver(segments)
    solver.base_theta = np.pi / 2 # 水平放置
    solver.base_phi = 0.0
    solver.g = 9.81
    return solver

# ============================================================
# 扫描 E_soft 范围
# ============================================================
E_candidates = np.linspace(3e6, 8e6, 30) # 3.0 MPa ~ 8.0 MPa
rmse_list = []
tip_droop_list = []

print("\n正在进行精细拟合扫描...")
print(f"{'E_soft (MPa)':>15s} {'Tip Droop (mm)':>15s} {'RMSE (mm)':>12s}")
print("-" * 45)

for E_soft in E_candidates:
    solver = build_solver(E_soft)
    
    # 更强的向下初始猜测 (0.1 rad ≈ 5.7度)
    n_dof = (solver.N - 1) * 2
    x0 = np.zeros(n_dof)
    x0[:solver.N-1] = 0.1 
    
    sol, _ = solver.solve(epm_pos=None, epm_m=None, maxiter=500, ftol=1e-10, x0=x0)
    pts = solver.get_geometry(sol)
    
    sim_x = pts[:, 0]
    sim_y = pts[:, 1]
    
    tip_droop = abs(sim_y[-1]) * 1000
    tip_droop_list.append(tip_droop)
    
    # 插值对比
    from scipy.interpolate import interp1d
    f_sim = interp1d(sim_x, sim_y, kind='linear', fill_value='extrapolate')
    sim_y_at_real = f_sim(real_x)
    rmse = np.sqrt(np.mean((sim_y_at_real - real_y)**2)) * 1000
    rmse_list.append(rmse)
    
    # 打印进度（每5个打印一次）
    if len(rmse_list) % 5 == 0:
        print(f"{E_soft/1e6:>15.2f} {tip_droop:>15.2f} {rmse:>12.3f}")

rmse_list = np.array(rmse_list)
best_idx = np.argmin(rmse_list)
best_E = E_candidates[best_idx]
best_rmse = rmse_list[best_idx]

print(f"\n拟合完成！")
print(f"最佳杨氏模量 E_soft: {best_E/1e6:.2f} MPa")
print(f"最小平均误差 RMSE: {best_rmse:.3f} mm")

# 可视化
solver_best = build_solver(best_E)
x0_f = np.zeros((solver_best.N-1)*2); x0_f[:solver_best.N-1]=0.05
sol_best, _ = solver_best.solve(maxiter=1000, ftol=1e-12, x0=x0_f)
pts_best = solver_best.get_geometry(sol_best)

fig, axes = plt.subplots(1, 2, figsize=(15, 6))
axes[0].semilogx(E_candidates/1e6, rmse_list, 'b.-')
axes[0].axvline(best_E/1e6, color='r', linestyle='--')
axes[0].set_title('Error Surface')
axes[0].set_xlabel('E_soft (MPa)')
axes[0].set_ylabel('RMSE (mm)')

axes[1].plot(real_x*1000, real_y*1000, 'r-', label='Real (Rescaled 45mm)')
axes[1].plot(pts_best[:,0]*1000, pts_best[:,1]*1000, 'b--', label=f'Sim (E={best_E/1e6:.2f}MPa)')
axes[1].set_title(f'Shape Comparison (RMSE={best_rmse:.2f} mm)')
axes[1].set_xlabel('X (mm)')
axes[1].set_ylabel('Y (mm)')
axes[1].grid(True)
axes[1].legend()
axes[1].invert_yaxis()
axes[1].set_aspect('equal')

plt.tight_layout()
save_path = '/home/wen-zheng/guidewire_simulation/calibration_final_result.png'
plt.savefig(save_path, dpi=150)
print(f"\nSaved: {save_path}")
