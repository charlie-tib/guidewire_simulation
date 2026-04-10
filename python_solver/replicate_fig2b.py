import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from mcr_python_solver import DVSMagneticRodSolver
import os

class Fig2bReplicator(DVSMagneticRodSolver):
    def solve_constrained(self, target_theta_deg, epm_pos, epm_m):
        """
        固定末端总弯曲角为 target_theta_deg，优化内部 n-1 个节点的角度。
        """
        target_theta_rad = np.radians(target_theta_deg)
        n_vars = (self.N - 1)
        
        # 初始猜测：均匀分布弯曲
        x0 = np.ones(n_vars) * (target_theta_rad / n_vars)
        
        # 约束条件：sum(theta_i) = target_theta_rad
        cons = ({'type': 'eq', 'fun': lambda x: np.sum(x) - target_theta_rad})
        
        # 目标函数：总能量 (这里只考虑 2D，phi 设为 0)
        def objective(x):
            # 将 x 转换为 2D 状态 (theta, phi=0)
            state = np.zeros(n_vars * 2)
            state[:n_vars] = x
            return self.total_energy(state, epm_pos, epm_m)

        res = minimize(objective, x0, constraints=cons, method='SLSQP', tol=1e-6)
        
        if res.success:
            # 计算能量分量
            state_final = np.zeros(n_vars * 2)
            state_final[:n_vars] = res.x
            e_el = self.total_energy(state_final, epm_pos, np.zeros(3))
            e_mag = res.fun - e_el
            return res.fun, e_el, e_mag
        else:
            return None, None, None

if __name__ == "__main__":
    # 配置
    segments = [
        {'length': 0.020, 'E': 200e6, 'is_magnet': False}, 
        {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
        {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0},
        {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
        {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': -1.0},
        {'length': 0.020, 'E': 10e6, 'is_magnet': False}, 
        {'length': 0.010, 'E': 1000e6, 'is_magnet': True, 'polarity': 1.0}
    ]
    
    replicator = Fig2bReplicator(segments)
    
    # 外部磁场：90 度 (垂直)
    # EPM 放在侧向 150mm
    epm_pos = np.array([0.08, 0.0, 0.055])
    phi_field = np.radians(90)
    epm_m = np.array([np.sin(phi_field), 0, np.cos(phi_field)]) * 5.0
    
    thetas_sweep = np.linspace(-180, 180, 91)
    results = {'theta': [], 'total': [], 'elastic': [], 'magnetic': []}
    
    print("正在扫摆弯曲角度以生成 Fig 2b...")
    for th in thetas_sweep:
        tot, el, mag = replicator.solve_constrained(th, epm_pos, epm_m)
        if tot is not None:
            results['theta'].append(th)
            results['total'].append(tot)
            results['elastic'].append(el)
            results['magnetic'].append(mag)
            
    # 绘图
    plt.figure(figsize=(10, 6))
    plt.plot(results['theta'], results['total'], 'b-', linewidth=3, label='Total Energy (E_total)')
    plt.plot(results['theta'], results['elastic'], 'g--', linewidth=2, label='Elastic Energy (E_elastic)')
    plt.plot(results['theta'], results['magnetic'], 'r:', linewidth=2, label='Magnetic Energy (E_magnetic)')
    
    # 标记极小值点
    tot_arr = np.array(results['total'])
    # 寻找局部极小值 (简单梯度检查)
    for i in range(1, len(tot_arr)-1):
        if tot_arr[i] < tot_arr[i-1] and tot_arr[i] < tot_arr[i+1]:
            plt.scatter(results['theta'][i], tot_arr[i], color='blue', s=100, zorder=5, label='Stable Posture' if i == 1 else "")
            plt.annotate(f"{results['theta'][i]:.0f}°", (results['theta'][i], tot_arr[i]), 
                         textcoords="offset points", xytext=(0,10), ha='center', fontweight='bold')

    plt.title("Replication of Fig. 2(b): Energy vs. Bending Angle (φ = 90°)", fontsize=14)
    plt.xlabel("Bending Angle θ (deg)", fontsize=12)
    plt.ylabel("Potential Energy (J)", fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.savefig("metric1_fig2b_replication.png")
    print("复刻完成，保存至 metric1_fig2b_replication.png")
