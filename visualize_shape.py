import numpy as np
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver
import pandas as pd
from scipy.interpolate import interp1d
import os
from datetime import datetime

# 强制使用非交互式后端以防阻塞
import matplotlib
matplotlib.use('Agg')

def run_standard_3panel_validation():
    # 0. 准备输出目录
    output_dir = "testification_shape"
    if not os.path.exists(output_dir): os.makedirs(output_dir)

    # --- 1. 物理结构参数 (锁定 155.0mm 规则) ---
    len_shaft = 0.020 # m
    len_soft = 0.035  
    len_mag = 0.010   
    # 【参数可调区：仅修改这里的值】
    modulus_soft, modulus_hard = 12e6, 200e6 
    
    segments = [
        {'length': len_shaft, 'E': modulus_hard, 'is_magnet': False}, 
        {'length': len_soft, 'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0},
        {'length': len_soft, 'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag, 'E': modulus_hard, 'is_magnet': True, 'polarity': -1.0},
        {'length': len_soft, 'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag, 'E': modulus_hard, 'is_magnet': True, 'polarity': 1.0}
    ]
    KNOWN_TOTAL_MM = 155.0
    
    # --- 2. 加载正确数据并应用 155mm 标定系数 ---
    real_csv = '/home/wen-zheng/guidewire_simulation/centerline_real.csv'
    if not os.path.exists(real_csv):
        print(f"ERROR: {real_csv} not found!")
        return
    
    df = pd.read_csv(real_csv)
    raw_x = df['x_mm'].values
    raw_z = -df['y_mm'].values # 向上为正Z
    
    # 计算当前标尺下的物理总长 (通常是 168mm 左右)
    diffs = np.diff(np.column_stack([raw_x, raw_z]), axis=0)
    current_len_mm = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
    
    # 核心：不管标尺定标是多少，强制将导丝总长定义为 155mm
    KNOWN_TOTAL_MM = 155.0
    k_scale = KNOWN_TOTAL_MM / current_len_mm
    print(f">>> 数据源校正: 读取带磁场 CSV, 标尺长度 {current_len_mm:.1f}mm -> 修正为 {KNOWN_TOTAL_MM}mm (系数: {k_scale:.4f})")

    # 应用修正系数
    exp_x = (raw_x * k_scale) / 1000.0 # m
    exp_z = (raw_z * k_scale) / 1000.0 # m
    
    # 同样修正 EPM 位置 (从标尺单位 -> 155mm 标准单位)
    with open('/home/wen-zheng/guidewire_simulation/epm_coords_val.txt', 'r') as f:
        lines = f.readlines()
        epm_x_raw = float(lines[0].split('=')[1])
        epm_y_raw = float(lines[1].split('=')[1])
    epm_pos = np.array([epm_x_raw * k_scale, 0.0, -epm_y_raw * k_scale]) / 1000.0

    # --- 3. 求解 (取消寻优，使用固定朝向) ---
    solver = DVSMagneticRodSolver(segments, dl=0.001)
    solver.base_theta = np.pi / 2
    solver.g_vec = np.array([0, 0, -9.81]) 
    
    # 计算磁矩强度 (N52 D50xH30)
    m_mag = (1.45 * (np.pi * 0.025**2 * 0.030)) / (4e-7 * np.pi) 

    # 俯仰角 alpha (XZ平面内，0=右，90=上)
    alpha_rad = np.deg2rad(40.0) 
    # 方位角 beta (向屏幕内外偏转，0=在XZ平面内，正=向屏幕外，负=向屏幕内)
    beta_rad = np.deg2rad(0.0) 

    epm_m = np.array([
        m_mag * np.cos(alpha_rad) * np.cos(beta_rad), # X分量
    m_mag * np.sin(beta_rad),                     # Y分量 (进深)
    m_mag * np.sin(alpha_rad) * np.cos(beta_rad)  # Z分量 (上下)
    ])

    
   
    sol, _ = solver.solve(epm_pos=epm_pos, epm_m=epm_m, maxiter=500)
    best_pts = solver.get_geometry(sol)
    
    # 计算 RMSE
    interp_f = interp1d(exp_x, exp_z, bounds_error=False, fill_value="extrapolate")
    target_z = interp_f(best_pts[:, 0])
    best_rmse = np.sqrt(np.mean((best_pts[:, 2] - target_z)**2))

    print(f"仿真完成! RMSE = {best_rmse*1000:.2f} mm")

    # --- 4. 生成标准三面板视图 ---
    fig = plt.figure(figsize=(24, 7))
    
    # Panel 1: 3D视图
    ax1 = fig.add_subplot(131, projection='3d')
    # 绘制导丝和磁铁
    ax1.plot(best_pts[:, 0]*1000, best_pts[:, 1]*1000, best_pts[:, 2]*1000, 'r-', linewidth=4, label='Simulation')
    ax1.scatter(epm_pos[0]*1000, epm_pos[1]*1000, epm_pos[2]*1000, c='orange', s=250, marker='s', label='N52 EPM')
    
    # 【新增】标注 Base 和 Tip
    ax1.scatter(best_pts[0,0]*1000, best_pts[0,1]*1000, best_pts[0,2]*1000, color='black', s=100, label='Base')
    ax1.text(best_pts[0,0]*1000, best_pts[0,1]*1000, best_pts[0,2]*1000, " [Base]", fontweight='bold')
    
    ax1.scatter(best_pts[-1,0]*1000, best_pts[-1,1]*1000, best_pts[-1,2]*1000, color='blue', s=80, label='Tip')
    ax1.text(best_pts[-1,0]*1000, best_pts[-1,1]*1000, best_pts[-1,2]*1000, " [Tip]", fontweight='bold', color='blue')

    ax1.set_title("3D Global Shape")
    ax1.set_xlabel("X (mm)"); ax1.set_ylabel("Y (mm)"); ax1.set_zlabel("Z (mm)")
    ax1.axis('equal'); ax1.legend()

    # Panel 2: X-Z 侧视图 (Up/Down)
    ax2 = fig.add_subplot(132)
    ax2.plot(exp_x*1000, exp_z*1000, 'ko', markersize=3, alpha=0.3, label='Real (Exp)')
    ax2.plot(best_pts[:, 0]*1000, best_pts[:, 2]*1000, 'r-', linewidth=3, label='Simulation')
    ax2.set_title(f"X-Z View (Side Projection) - RMSE: {best_rmse*1000:.2f}mm")
    ax2.set_xlabel("X (mm)"); ax2.set_ylabel("Z (mm)")
    ax2.grid(True); ax2.axis('equal'); ax2.legend()

    # Panel 3: X-Y 俯视图 (Top View - Depth)
    ax3 = fig.add_subplot(133)
    ax3.plot(best_pts[:, 0]*1000, best_pts[:, 1]*1000, 'g-', linewidth=3)
    ax3.set_title("X-Y View (Top Projection)")
    ax3.set_xlabel("X (mm)"); ax3.set_ylabel("Y (mm)")
    ax3.grid(True); ax3.axis('equal')

    # 5. 保存
    timestamp = datetime.now().strftime("%H%M")
    save_name = f"validation_standard_3panel_{timestamp}.png"
    plt.tight_layout(); plt.savefig(os.path.join(output_dir, save_name), dpi=200)
    print(f"SUCCESS: {save_name} saved.")

if __name__ == "__main__":
    run_standard_3panel_validation()
