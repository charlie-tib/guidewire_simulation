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

    # --- 1. 物理结构参数 (遵循 User Rule 指导：总长 155mm) ---
    len_shaft = 0.020 # m (硬段基座)
    len_soft = 0.035  # m (软铰链)
    len_mag = 0.010   # m (磁铁)
    
    # 【参数执行区：遵循 20MPa/200MPa 锁定值】
    modulus_soft = 12e6 
    modulus_hard = 200e6
    
    segments = [
        {'length': len_shaft, 'E': modulus_hard, 'is_magnet': False}, 
        {'length': len_soft,  'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag,   'E': modulus_hard, 'is_magnet': True,  'polarity': 1.0},
        {'length': len_soft,  'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag,   'E': modulus_hard, 'is_magnet': True,  'polarity': 1.0},
        {'length': len_soft,  'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag,   'E': modulus_hard, 'is_magnet': True,  'polarity': 1.0}
    ]
    KNOWN_TOTAL_MM = 155.0
    
    # 【手动参数区】
    # 磁铁位置：改成 [距离X, 距离Z] 即可手动覆盖；设为 None 则尝试从 epm_coords_val.txt 读取
    epm_pos_manual = np.array([191.2,-72.8]) 
    #epm_pos_manual = None

    # 磁铁朝向：Pitch (俯仰角, 0=右, 90=上, 180=左, 270=下)
    # Beta: Yaw (方位角, 0=XZ平面内)
    alpha_manual = 40
    beta_manual = 0

    # --- 2. 加载标注数据 ---
    real_csv = '/home/wen-zheng/guidewire_simulation/centerline_real_6.csv'
    epm_file = '/home/wen-zheng/guidewire_simulation/epm_coords_val.txt'
    ruler_file = '/home/wen-zheng/guidewire_simulation/ruler_calibration.txt'
    
    px_per_mm = 1.0 # Default fallback
    if os.path.exists(ruler_file):
        with open(ruler_file, 'r') as f:
            for line in f:
                if 'px_per_mm' in line:
                    px_per_mm = float(line.split('=')[1])
        print(f">>> 从标尺文件加载到: px_per_mm = {px_per_mm:.4f}")

    if not os.path.exists(real_csv):
        print(f"ERROR: Cannot find {real_csv}. Please annotate first.")
        return

    df = pd.read_csv(real_csv)
    raw_x = df['x_mm'].values
    raw_y = df['y_mm'].values
    raw_s = np.sum(np.sqrt(np.diff(raw_x)**2 + np.diff(raw_y)**2))
    
    # 物理比例校准 (对齐 155mm 总长)
    scale_factor = KNOWN_TOTAL_MM / raw_s
    print(f">>> 标注弧长: {raw_s:.2f} mm | 设计总长: {KNOWN_TOTAL_MM} mm")
    print(f">>> 校准缩放因子: {scale_factor:.4f}")

    exp_x = raw_x * scale_factor / 1000.0
    exp_z = -raw_y * scale_factor / 1000.0 # Y轴转为Z（向上为正）
    
    # --- 3. 磁铁位置判定 ---
    if epm_pos_manual is not None:
        epm_pos = np.array([epm_pos_manual[0], 0.0, epm_pos_manual[1]]) / 1000.0
        print(f">>> 采用手动磁铁位置: X={epm_pos_manual[0]:.1f}mm, Z={epm_pos_manual[1]:.1f}mm")
    elif os.path.exists(epm_file):
        with open(epm_file, 'r') as f:
            lines = f.readlines()
            epm_x_f = float(lines[0].split('=')[1]) * scale_factor
            epm_y_f = float(lines[1].split('=')[1]) * scale_factor
            epm_pos = np.array([epm_x_f, 0.0, -epm_y_f]) / 1000.0
        print(f">>> 采用标定文件位置: X={epm_x_f:.1f}mm, Z={-epm_y_f:.1f}mm")
    else:
        epm_pos = np.array([188.0, 0.0, 52.0]) / 1000.0
        print(">>> WARNING: Using Photo14 fallback EPM position.")

    # --- 4. 求解与验证 (直接使用手动输入的角度) ---
    solver = DVSMagneticRodSolver(segments, dl=0.001)
    solver.base_theta = np.pi / 2
    solver.g_vec = np.array([0, 0, -9.81]) 
    m_mag = (1.45 * (np.pi * 0.025**2 * 0.030)) / (4e-7 * np.pi) # N52 D50xH30
    #m_mag=0

    print(f"\n>>> 正在按手动输入求解: Pitch={alpha_manual}°, Yaw={beta_manual}°")
    
    alpha_r, beta_r = np.deg2rad(alpha_manual), np.deg2rad(beta_manual)
    final_m = np.array([
        m_mag * np.cos(alpha_r) * np.cos(beta_r),
        m_mag * np.sin(beta_r),
        m_mag * np.sin(alpha_r) * np.cos(beta_r)
    ])
    final_sol, _ = solver.solve(epm_pos=epm_pos, epm_m=final_m, maxiter=500)
    best_pts = solver.get_geometry(final_sol)

    # 计算 RMSE
    interp_f = interp1d(exp_x, exp_z, bounds_error=False, fill_value="extrapolate")
    z_target = interp_f(best_pts[:, 0])
    best_rmse = np.sqrt(np.mean((best_pts[:, 2] - z_target)**2)) * 1000.0
    pixel_rmse = best_rmse * px_per_mm
    print(f">>> 仿真结果: RMSE={best_rmse:.3f} mm | Pixel RMSE={pixel_rmse:.2f} px")

    # --- 5. 生成可视化 ---
    fig = plt.figure(figsize=(24, 7))
    
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(best_pts[:, 0]*1000, best_pts[:, 1]*1000, best_pts[:, 2]*1000, 'r-', linewidth=4, label='Simulation')
    ax1.scatter(epm_pos[0]*1000, epm_pos[1]*1000, epm_pos[2]*1000, c='orange', s=250, marker='s', label='EPM')
    ax1.set_title("3D Global View")
    ax1.set_xlabel("X (mm)"); ax1.set_ylabel("Y (mm)"); ax1.set_zlabel("Z (mm)")
    ax1.axis('equal'); ax1.legend()

    ax2 = fig.add_subplot(132)
    ax2.plot(exp_x*1000, exp_z*1000, 'ko', markersize=4, alpha=0.4, label='Experiment')
    ax2.plot(best_pts[:, 0]*1000, best_pts[:, 2]*1000, 'r-', linewidth=3, label='Simulation Fit')
    ax2.set_title(f"X-Z Elevation\nRMSE: {best_rmse:.2f} mm | {pixel_rmse:.1f} pixels")
    ax2.set_ylabel("Z (mm)")
    ax2.grid(True); ax2.axis('equal'); ax2.legend()

    ax3 = fig.add_subplot(133)
    ax3.plot(best_pts[:, 0]*1000, best_pts[:, 1]*1000, 'g-', linewidth=3, label='Depth Shift')
    ax3.set_title("X-Y Top-down View")
    ax3.set_xlabel("X (mm)"); ax3.set_ylabel("Y (mm)")
    ax3.grid(True); ax3.axis('equal')

    # 6. 保存
    timestamp = datetime.now().strftime("%H%M")
    save_name = f"final_validation_result_{timestamp}.png"
    plt.tight_layout(); plt.savefig(os.path.join(output_dir, save_name), dpi=200)
    print(f"SUCCESS: {save_name} saved in {output_dir}")

if __name__ == "__main__":
    run_standard_3panel_validation()
