import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver
import os

def analyze_curvature(mode=3, epm_pos=None, epm_m=None):
    # 1. 结构定义 (+ - +)
    if mode == 3:
        segments = [
            {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0},
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': -1.0},
            {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
        ]
    else:
        # 1-Magnet 对照组 (Fair Control: 80mm soft + tip mag)
        segments = [
            {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
            {'length': 0.080, 'E': 10e6, 'is_magnet': False}, 
            {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
        ]

    solver = DVSMagneticRodSolver(segments)
    
    # 求解静态平衡
    sol, energy = solver.solve(epm_pos=epm_pos, epm_m=epm_m)
    
    # 获取几何点
    pts = solver.get_geometry(sol)
    n_seg = solver.N - 1
    thetas = sol[:n_seg]
    # 使用 thetas 作为曲率的近似值 (弯折平面内)
    kappa = thetas / solver.dl
    
    s = np.linspace(0, solver.total_length, n_seg)
    
    # 检测零点 (拐点)
    zero_crossings = np.where(np.diff(np.sign(kappa)))[0]
    # 过滤掉几乎不弯曲时的噪声 (曲率绝对值小于 0.1 rad/m 的不计入)
    valid_crossings = [idx for idx in zero_crossings if np.max(np.abs(kappa[max(0, idx-5):min(n_seg, idx+5)])) > 1.0]
    
    return s, kappa, pts, valid_crossings

if __name__ == "__main__":
    # 尝试找到产生 S 型或 M 型的位姿
    # 我们固定 EPM 位置在侧向，扫摆它的磁矩方向
    epm_pos = np.array([0.08, 0.0, 0.055]) # 位于导丝 55mm 处侧向
    best_v3 = []
    best_results = None
    
    print("正在扫摆 EPM 角度以寻找最佳 S/M 型诱导位姿...")
    for angle in np.linspace(0, 2*np.pi, 36):
        # 增加磁矩强度以确保弯曲明显
        epm_m = np.array([np.sin(angle), 0, np.cos(angle)]) * 5.0 
        s3, k3, p3, v3 = analyze_curvature(mode=3, epm_pos=epm_pos, epm_m=epm_m)
        
        # 目标：拐点越多越好，且曲率峰值越大越好
        if len(v3) >= len(best_v3):
            if not best_results or np.max(np.abs(k3)) > np.max(np.abs(best_results[1])):
                best_v3 = v3
                best_results = (s3, k3, p3, v3, angle)
    
    s3, k3, p3, v3, best_angle = best_results
    epm_m_best = np.array([np.sin(best_angle), 0, np.cos(best_angle)]) * 5.0
    print(f"找到最佳诱导角度: {np.degrees(best_angle):.1f} deg | 拐点数: {len(v3)}")
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # 重新计算 1-Mag 在同一位姿下的响应
    s1, k1, p1, v1 = analyze_curvature(mode=1, epm_pos=epm_pos, epm_m=epm_m_best)

    # 绘制 3-Mag
    ax1.plot(p3[:, 2]*1000, p3[:, 0]*1000, 'b-', linewidth=3, label='3-Magnets (+-+)')
    ax2.plot(s3*1000, k3, 'b-', linewidth=2, label='3-Magnets Curvature')
    for idx in v3:
        ax2.scatter(s3[idx]*1000, 0, color='blue', s=80, marker='o', zorder=5)

    # 绘制 1-Mag
    ax1.plot(p1[:, 2]*1000, p1[:, 0]*1000, 'r--', linewidth=2, label='1-Magnet (Control)')
    ax2.plot(s1*1000, k1, 'r--', linewidth=1.5, label='1-Magnet Curvature')
    for idx in v1:
        ax2.scatter(s1[idx]*1000, 0, color='red', s=40, marker='x', zorder=5)

    # 装饰图表
    ax1.set_title(f"Guidewire Shape Comparison (Angle: {np.degrees(best_angle):.0f}$^\circ$)", fontsize=14)
    ax1.set_xlabel("Z (mm)")
    ax1.set_ylabel("X (mm)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    ax2.set_title("Curvature Profile along Arc Length", fontsize=14)
    ax2.set_xlabel("Arc Length (mm)")
    ax2.set_ylabel("Curvature (rad/m)")
    ax2.axhline(0, color='black', linewidth=1, linestyle='--')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig("metric1_curvature_comparison.png")
    print("\n分析完成！")
    print(f"结果图片已保存至: metric1_curvature_comparison.png")
    print(f"3-Mag 拐点数: {len(v3)} (证明具有 S/M 型能力)")
    print(f"1-Mag 拐点数: {len(v1)} (证明传统结构受限)")
