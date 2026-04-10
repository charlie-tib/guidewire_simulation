import numpy as np
import pandas as pd
from mcr_python_solver import DVSMagneticRodSolver
import time

# 工作空间采样主程序
def run_sampling(num_samples=500, mode=3):
    # 1. 结构定义
    if mode == 3:
        # 3-Magnets 交变磁化结构 (+ - +)
        segments = [
            {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0},
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': -1.0},
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
        ]
        filename = "python_workspace_results_3mag.csv"
    elif mode == 4:
        # 3-Magnets 同向磁化结构 (+ + +)
        segments = [
            {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0},
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0},
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
        ]
        filename = "python_workspace_results_3mag_pos.csv"
    elif mode == 2:
        # Fair Control: 20mm Rigid + 80mm Soft + 10mm Tip Magnet
        segments = [
            {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
            {'length': 0.080, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
        ]
        filename = "python_workspace_results_fair.csv"
    else:
        # 标准 1-Magnet 结构 (80mm Rigid + 20mm Soft + 10mm Mag)
        segments = [
            {'length': 0.080, 'E': 200e6, 'is_magnet': False}, 
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
        ]
        filename = "python_workspace_results_1mag.csv"

    solver = DVSMagneticRodSolver(segments)
    
    # 外部磁场参数 (与 SOFA 对齐)
    mu0 = 4 * np.pi * 1e-7
    br_n52 = 1.45
    epm_radius = 0.025 
    epm_height = 0.030 
    epm_volume = np.pi * (epm_radius**2) * epm_height
    m_epm_mag = (br_n52 * epm_volume) / mu0 
    m_internal_mag = 0.05 
    
    results = []
    print(f"正在启动 Mode {mode} 的 EPM 空间采样 (共 {num_samples} 次)...")

    for i in range(num_samples):
        # 采样永磁体位置 (100-150mm 全球面)
        r = np.random.uniform(0.10, 0.15) 
        phi = np.random.uniform(0, 2*np.pi)
        theta = np.random.uniform(0, np.pi) # 全球面采样
        
        epm_pos = np.array([
            r * np.sin(theta) * np.cos(phi),
            r * np.sin(theta) * np.sin(phi),
            r * np.cos(theta)
        ])
        
        # 采样永磁体姿态
        random_vec = np.random.randn(3)
        epm_m_dir = random_vec / np.linalg.norm(random_vec)
        epm_m = epm_m_dir * m_epm_mag
        
        # 求解
        sol, energy = solver.solve(epm_pos=epm_pos, epm_m=epm_m * m_internal_mag)
        
        # 提取弯曲形状特征 (Inflection Points / S-shape)
        n_seg = solver.N - 1
        thetas = sol[:n_seg]
        phis = sol[n_seg:]
        
        # 检查是否有曲率翻转 (S型特征)
        # Bending vector v = [theta * cos(phi), theta * sin(phi)]
        inflections = 0
        for j in range(1, len(thetas)):
            v_curr = np.array([thetas[j] * np.cos(phis[j]), thetas[j] * np.sin(phis[j])])
            v_prev = np.array([thetas[j-1] * np.cos(phis[j-1]), thetas[j-1] * np.sin(phis[j-1])])
            
            # 只要点积为负且弯曲幅度大于噪声 (0.01 rad)，即视为方向翻转
            if np.dot(v_curr, v_prev) < -1e-6 and (thetas[j] > 0.01 or thetas[j-1] > 0.01):
                inflections += 1
        
        pts = solver.get_geometry(sol)
        p20 = pts[20]
        p50 = pts[50]
        p80 = pts[80]
        p110 = pts[110]
        
        sigma_theta = np.sum(np.sqrt(thetas**2 + phis**2))
        
        results.append({
            'epm_x': epm_pos[0], 'epm_y': epm_pos[1], 'epm_z': epm_pos[2],
            'inflections': inflections,
            'p20_x': p20[0], 'p20_y': p20[1], 'p20_z': p20[2],
            'p50_x': p50[0], 'p50_y': p50[1], 'p50_z': p50[2],
            'p80_x': p80[0], 'p80_y': p80[1], 'p80_z': p80[2],
            'tip_x': p110[0], 'tip_y': p110[1], 'tip_z': p110[2],
            'sigma_theta': sigma_theta,
            'energy': energy,
            'tip_z': p110[2]
        })
        
        if (i+1) % 100 == 0:
            print(f"进度: {i+1}/{num_samples}")

    df = pd.DataFrame(results)
    s_shape_rate = (df['inflections'] > 0).mean() * 100
    output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs_and_data/csv_results")
    if not os.path.exists(output_dir): os.makedirs(output_dir)
    full_path = os.path.join(output_dir, filename)
    df.to_csv(full_path, index=False)
    print(f"结果已保存至: {full_path} | S-Shape 概率: {s_shape_rate:.1f}%")

if __name__ == "__main__":
    # 执行 4 组采样
    run_sampling(num_samples=500, mode=3) # +- +
    run_sampling(num_samples=500, mode=4) # + + +
    run_sampling(num_samples=500, mode=1) # Standard
    run_sampling(num_samples=500, mode=2) # Fair
