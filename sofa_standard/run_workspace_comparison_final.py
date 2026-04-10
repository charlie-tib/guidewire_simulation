import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver
from scipy.spatial import ConvexHull
import os

# --- 1. 物理参数配置 ---
E_SOFT = 12e6
E_HARD = 200e6
DL = 0.005 # 计算步长, 0.005m = 5mm (为提高工作空间采样速度)

# 磁铁属性 (N52 D50xH30)
M_MAG_VAL = (1.45 * (np.pi * 0.025**2 * 0.030)) / (4e-7 * np.pi)

# --- 2. 结构定义 ---
def get_segments(mode):
    if mode == "my_design":
        # Distributed 3-Mag +++ (Total 155mm)
        return [
            {'length': 0.020, 'E': E_HARD, 'is_magnet': False}, 
            {'length': 0.035, 'E': E_SOFT, 'is_magnet': False},
            {'length': 0.010, 'E': E_HARD, 'is_magnet': True, 'polarity': 1.0},
            {'length': 0.035, 'E': E_SOFT, 'is_magnet': False},
            {'length': 0.010, 'E': E_HARD, 'is_magnet': True, 'polarity': 1.0},
            {'length': 0.035, 'E': E_SOFT, 'is_magnet': False},
            {'length': 0.010, 'E': E_HARD, 'is_magnet': True, 'polarity': 1.0}
        ]
    elif mode == "standard":
        # Traditional Stiff Shaft 1-Mag (Total 155mm)
        return [
            {'length': 0.020, 'E': E_HARD, 'is_magnet': False},
            {'length': 0.090, 'E': E_HARD, 'is_magnet': False}, 
            {'length': 0.035, 'E': E_SOFT, 'is_magnet': False},
            {'length': 0.010, 'E': E_HARD, 'is_magnet': True, 'polarity': 1.0}
        ]
    elif mode == "floppy":
        # All Soft 1-Mag (Total 155mm)
        return [
            {'length': 0.020, 'E': E_HARD, 'is_magnet': False},
            {'length': 0.125, 'E': E_SOFT, 'is_magnet': False},
            {'length': 0.010, 'E': E_HARD, 'is_magnet': True, 'polarity': 1.0}
        ]

# --- 3. 采样引擎 ---
def run_monte_carlo(mode, num_samples=1000):
    # 【保证绝对可重复】每次调用函数前不重置 seed，但在主循环前重置
    segments = get_segments(mode)
    solver = DVSMagneticRodSolver(segments, dl=DL)
    solver.base_theta = np.pi/2
    solver.g_vec = np.array([0, 0, -9.81])
    
    tip_positions = []
    tip_orientations = []
    
    print(f"\n>>> Running Monte Carlo for [{mode}]: {num_samples} samples...")
    
    for i in range(num_samples):
        # 3.1 随机生成磁铁位置 (3D 容积采样: R=120mm 到 250mm)
        r = np.random.uniform(0.120, 0.250)
        phi = np.random.uniform(0, 2*np.pi)
        costheta = np.random.uniform(-1, 1) # 保证球面采样均匀
        theta = np.arccos(costheta)
        
        epm_pos = np.array([
            r * np.sin(theta) * np.cos(phi),
            r * np.sin(theta) * np.sin(phi), # 开启 Y 轴自由度
            r * np.cos(theta)
        ])
        
        # 3.2 随机生成磁铁朝向 (单位球)
        u = np.random.normal(0, 1, 3)
        epm_m = (u / np.linalg.norm(u)) * M_MAG_VAL
        
        # 3.3 求解
        sol, _ = solver.solve(epm_pos=epm_pos, epm_m=epm_m, maxiter=200)
        pts = solver.get_geometry(sol)
        
        # 记录尖端坐标 (mm)
        tip_pos = pts[-1] * 1000.0
        tip_positions.append(tip_pos)
        
        # 记录尖端指向向量
        v_tip = (pts[-1] - pts[-2]) / np.linalg.norm(pts[-1] - pts[-2])
        tip_orientations.append(v_tip)
        
        if (i+1) % 50 == 0:
            print(f"    - Sample {i+1}/{num_samples} done.")
            
    return np.array(tip_positions), np.array(tip_orientations)

# --- 4. 执行对比 ---
SAMPLES = 1000 # 提升至 1000 次采样以致密化分布
results = {}

# 固定全局随机种子，确保小数点后六位的绝对可重复性
np.random.seed(42)

for mode in ["my_design", "standard"]:
    pos, ori = run_monte_carlo(mode, SAMPLES)
    results[mode] = {'pos': pos, 'ori': ori}

# --- 5. 计算指标与绘图 ---
# --- 5. 计算指标与绘图 (精简双屏专业版) ---
fig = plt.figure(figsize=(16, 7))
gs = fig.add_gridspec(1, 2)

ax_3d = fig.add_subplot(gs[0, 0], projection='3d')
ax_ori = fig.add_subplot(gs[0, 1], projection='3d') # 姿态球

# 绘制参考用的半透明单位球背景
u_s = np.linspace(0, 2 * np.pi, 60)
v_s = np.linspace(0, np.pi, 60)
x_s = np.outer(np.cos(u_s), np.sin(v_s))
y_s = np.outer(np.sin(u_s), np.sin(v_s))
z_s = np.outer(np.ones(np.size(u_s)), np.cos(v_s))
ax_ori.plot_wireframe(x_s, y_s, z_s, color='gray', alpha=0.1, linewidth=0.5)

colors = {"my_design": "red", "standard": "blue"}
labels = {"my_design": "My Design (3-Mag +++)", 
          "standard": "Standard (1-Mag Stiff)"}

# 在 3D 图中只需标注 Base
for ax in [ax_3d]:
    ax.scatter([0], [0], [0], color='black', marker='X', s=200, zorder=100, label='Guidewire Base')

print("\n" + "="*60)
print(f"{'Mode':<20} | {'Convex Vol (cm³)':<16} | {'Voxel Vol (cm³)':<15} | {'Dexterity DCI (%)':<15}")
print("-" * 75)

for mode, data in results.items():
    pos = data['pos']
    ori = data['ori']
    
    # 5.1 外围体积计算 (Convex Hull)
    hull = ConvexHull(pos)
    vol_cm3 = hull.volume / 1000.0
    
    # 5.2 真实可达体积计算 (Voxel-based Volume, 分辨率 2mm)
    voxel_size = 2.0 # mm
    discrete_pos = np.floor(pos / voxel_size).astype(int)
    unique_voxels = np.unique(discrete_pos, axis=0)
    voxel_vol_cm3 = len(unique_voxels) * (voxel_size**3) / 1000.0
    
    # 1. 绘制 3D 凸包 (工作空间)
    ax_3d.plot_trisurf(pos[:,0], pos[:,1], pos[:,2], triangles=hull.simplices, color=colors[mode], alpha=0.15, edgecolor='none')
    ax_3d.scatter(pos[:,0], pos[:,1], pos[:,2], c=colors[mode], s=1, alpha=0.2) # 点缀少量散点

    # 2. 姿态覆盖指数 (Dexterity Coverage Index, DCI)
    try:
        hull_ori = ConvexHull(ori)
        # 归一化到 0~100% (单位球总表面积为 4*pi)
        dci_pct = (hull_ori.area / (4 * np.pi)) * 100.0
        
        # 使用散点和小边界线代替难看的实心三角面，视觉上更像球面斑块
        ax_ori.scatter(ori[:,0], ori[:,1], ori[:,2], c=colors[mode], s=10, alpha=0.6, label=f"{labels[mode]} ({dci_pct:.1f}%)")
        for simplex in hull_ori.simplices:
            ax_ori.plot(ori[simplex, 0], ori[simplex, 1], ori[simplex, 2], color=colors[mode], linewidth=0.8, alpha=0.7)
            
    except: dci_pct = 0.0
        
    print(f"{mode:<20} | {vol_cm3:<16.2f} | {voxel_vol_cm3:<15.2f} | {dci_pct:<15.2f}")
    
    # Legend proxies
    ax_3d.plot([], [], [], color=colors[mode], label=f"{labels[mode]} (Vol: {voxel_vol_cm3:.1f} cm³)")

# 视角与格式化
ax_3d.view_init(elev=20, azim=-60)
ax_3d.set_title("3D Reachable Workspace (Dense Envelope)")
ax_3d.set_xlabel("X (mm)"); ax_3d.set_ylabel("Y (mm)"); ax_3d.set_zlabel("Z (mm)")

ax_ori.set_title("Tip Orientation Coverage on Unit Sphere")
ax_ori.view_init(elev=30, azim=45)
ax_ori.set_xlabel("Vx"); ax_ori.set_ylabel("Vy"); ax_ori.set_zlabel("Vz")

ax_3d.legend(loc='lower center', bbox_to_anchor=(0.5, -0.15), ncol=2)
ax_ori.legend(loc='lower center', bbox_to_anchor=(0.5, -0.15), ncol=1)

plt.suptitle("Magnetic Guidewire Standardized Metrics Verification", fontsize=18)
plt.tight_layout()
plt.savefig("workspace_comparison_result.png", dpi=200)
print("="*40)
print("可视化结果已保存至: workspace_comparison_result.png")
