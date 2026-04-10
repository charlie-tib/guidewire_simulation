import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from mcr_python_solver import DVSMagneticRodSolver
import os
from tqdm import tqdm

def tune_params():
    # --- 1. 物理结构参数 (对齐 visualize_shape.py) ---
    len_shaft = 0.020 
    len_soft = 0.035  
    len_mag = 0.010   
    modulus_soft = 12e6 
    modulus_hard = 200e6
    segments = [
        {'length': len_shaft, 'E': modulus_hard, 'is_magnet': False}, 
        {'length': len_soft,  'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag,   'E': modulus_hard, 'is_magnet': True,  'polarity': 1.0},
        {'length': len_soft,  'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag,   'E': modulus_hard, 'is_magnet': True,  'polarity': -1.0},
        {'length': len_soft,  'E': modulus_soft, 'is_magnet': False},
        {'length': len_mag,   'E': modulus_hard, 'is_magnet': True,  'polarity': 1.0}
    ]
    KNOWN_TOTAL_MM = 155.0

    # --- 2. 加载目标数据 (Photo 6) ---
    real_csv = 'centerline_real_6.csv'
    if not os.path.exists(real_csv):
        print(f"Error: {real_csv} not found")
        return

    df = pd.read_csv(real_csv)
    raw_x = df['x_mm'].values
    raw_y = df['y_mm'].values
    raw_s = np.sum(np.sqrt(np.diff(raw_x)**2 + np.diff(raw_y)**2))
    
    scale_factor = KNOWN_TOTAL_MM / raw_s
    exp_x = raw_x * scale_factor / 1000.0
    exp_z = -raw_y * scale_factor / 1000.0

    # --- 3. 搜索空间定义 ---
    # 基于用户目前的设定: X=245.1, Alpha=179.6, Beta=5
    x_range = np.linspace(240, 255, 15)      # 15个点
    alpha_range = np.linspace(175, 185, 11)  # 11个点
    beta_range = np.linspace(0, 10, 6)      # 6个点
    
    z_fixed = -81.3
    m_mag = (1.45 * (np.pi * 0.025**2 * 0.030)) / (4e-7 * np.pi)

    solver = DVSMagneticRodSolver(segments, dl=0.001)
    solver.base_theta = np.pi / 2
    solver.g_vec = np.array([0, 0, -9.81])

    best_rmse = float('inf')
    best_params = {}

    print(f"Starting Grid Search (Total combinations: {len(x_range)*len(alpha_range)*len(beta_range)})...")
    
    # 使用 nested loops
    for x in tqdm(x_range):
        for alpha in alpha_range:
            for beta in beta_range:
                # 构造磁矩
                alpha_r, beta_r = np.deg2rad(alpha), np.deg2rad(beta)
                epm_m = np.array([
                    m_mag * np.cos(alpha_r) * np.cos(beta_r),
                    m_mag * np.sin(beta_r),
                    m_mag * np.sin(alpha_r) * np.cos(beta_r)
                ])
                epm_pos = np.array([x, 0.0, z_fixed]) / 1000.0
                
                # 求解
                try:
                    sol, _ = solver.solve(epm_pos=epm_pos, epm_m=epm_m, maxiter=300)
                    pts = solver.get_geometry(sol)
                    
                    # 计算 RMSE
                    interp_f = interp1d(exp_x, exp_z, bounds_error=False, fill_value="extrapolate")
                    z_target = interp_f(pts[:, 0])
                    rmse = np.sqrt(np.mean((pts[:, 2] - z_target)**2)) * 1000.0
                    
                    if rmse < best_rmse:
                        best_rmse = rmse
                        best_params = {'x': x, 'alpha': alpha, 'beta': beta}
                except:
                    continue

    print("\n" + "="*30)
    print("Optimization Results:")
    print(f"Best RMSE: {best_rmse:.4f} mm")
    print(f"Best X: {best_params['x']:.2f} mm")
    print(f"Best Alpha (Pitch): {best_params['alpha']:.2f}°")
    print(f"Best Beta (Yaw): {best_params['beta']:.2f}°")
    print("="*30)

if __name__ == "__main__":
    tune_params()
