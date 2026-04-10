import numpy as np
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver
import os

# 定义导丝的分段结构
# 1-Magnet: 20mm 软铰链 + 10mm 磁铁 (末端)
# 3-Magnets: (20mm 软 + 3.3mm 磁) * 3 , 中间磁铁极性反转
modulus_soft = 2e5
modulus_hard = 2e9

segs_1mag = [
    {'length': 0.080, 'E': modulus_hard, 'is_magnet': False}, # 硬轴 80mm
    {'length': 0.020, 'E': modulus_soft, 'is_magnet': False}, # 软铰链 20mm
    {'length': 0.010, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0}   # 磁铁 10mm
]

segs_3mag = [
    {'length': 0.080, 'E': modulus_hard, 'is_magnet': False},
    {'length': 0.0067, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.0033, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0},
    {'length': 0.0067, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.0033, 'E': modulus_hard, 'is_magnet': True, 'polarity': -1.0},
    {'length': 0.0067, 'E': modulus_soft, 'is_magnet': False},
    {'length': 0.0033, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0}
]

# 初始化求解器
solver_1m = DVSMagneticRodSolver(segs_1mag)
solver_3m = DVSMagneticRodSolver(segs_3mag)

# 为了让导丝在 XY 平面上弯曲（方便 2D 俯视观察），我们将基座设为沿 X 轴正方向
solver_1m.base_theta = np.pi / 2
solver_1m.base_phi = 0.0
# 消除重力影响，专注看磁力弯曲
solver_1m.g = 0.0 

solver_3m.base_theta = np.pi / 2
solver_3m.base_phi = 0.0
solver_3m.g = 0.0

# --- EPM 位置设定 ---
# 导丝没弯曲前，尖端坐标位于 X = 110mm (0.11 m), Y = 0
tip_x = 0.110
tip_y = 0.0
distance = 0.020 # 20mm
angle = np.pi / 4 # 逆时针 45 度

epm_x = tip_x + distance * np.cos(angle)
epm_y = tip_y + distance * np.sin(angle)
epm_z = 0.0
epm_pos = np.array([epm_x, epm_y, epm_z])

# EPM的磁化方向也顺着这个45度，吸力最强 (也可以设为指向上方，取决于您的桌面实验把磁铁怎么拿着)
# 假设是圆柱状大磁铁，磁矩方向向外
M_strength = 20.0 # 等效 N52 磁矩
epm_m = np.array([M_strength * np.cos(angle), M_strength * np.sin(angle), 0.0])

print(f"EPM 位置: ({epm_pos[0]*1000:.1f} mm, {epm_pos[1]*1000:.1f} mm, {epm_pos[2]*1000:.1f} mm)")

print("\n--- 正在求解 1-Magnet ---")
sol_1m, energy_1m = solver_1m.solve(epm_pos=epm_pos, epm_m=epm_m)
pts_1m = solver_1m.get_geometry(sol_1m)

print("--- 正在求解 3-Magnets ---")
# 为了帮助优化器找到不一样的局部极小值（S型），可以给个微小初值倾向
def energy_3m(x):
    return solver_3m.total_energy(x, epm_pos, epm_m)
from scipy.optimize import minimize
x0 = np.ones((solver_3m.N - 1) * 2) * 1e-3
x0[solver_3m.N - 1:] = 0.0 # phi 设为 0，强制在 XY 平面内寻找
res_3m = minimize(energy_3m, x0, method='L-BFGS-B', tol=1e-6)
pts_3m = solver_3m.get_geometry(res_3m.x)


# --- 可视化 2D 形态 ---
plt.figure(figsize=(10, 8))
# 绘制原始参考轴线
plt.plot([-0.01, 0.13], [0, 0], 'k--', alpha=0.3, label='Centerline')

# 绘制 1-Magnet
plt.plot(pts_1m[:, 0], pts_1m[:, 1], 'r-', linewidth=3, label='1-Magnet Structure')
plt.scatter(pts_1m[-10:, 0], pts_1m[-10:, 1], c='red', s=20, label='Tip Magnet (1-Mag)')

# 绘制 3-Magnets
plt.plot(pts_3m[:, 0], pts_3m[:, 1], 'b-', linewidth=3, label='3-Magnets (+1, -1, +1)')
# 标出 3-Mag 的三个磁铁位置
for m_node in solver_3m.magnet_nodes:
    m_range = slice(m_node - int(0.0033/0.001), m_node+1)
    plt.scatter(pts_3m[m_range, 0], pts_3m[m_range, 1], c='blue', s=20)

# 绘制大磁铁 EPM
plt.scatter(epm_x, epm_y, c='orange', s=300, marker='o', edgecolors='black', label='EPM (Distance=20mm, 45°)')
# 画一个箭头指示 EPM 的磁场方向
plt.arrow(epm_x, epm_y, 0.005 * np.cos(angle), 0.005 * np.sin(angle), head_width=0.002, head_length=0.002, fc='orange', ec='black')

plt.xlim(0.0, 0.15)
plt.ylim(-0.02, 0.08)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('2D Simulation Validation: EPM placed 20mm from Tip @ 45 Degrees')
plt.grid(True)
plt.legend()
plt.axis('equal') # 保证 X 和 Y 的比例尺相同，看到的形状不变形

save_path = '/home/wen-zheng/.gemini/antigravity/brain/d311d1c0-5266-44e5-993e-2bb8df2fdf0c/validation_20mm_45deg.png'
plt.savefig(save_path, dpi=300, bbox_inches='tight')
print(f"\n✅ 仿真完成，可视化图像已保存至: {save_path}")
