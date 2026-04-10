import numpy as np
from scipy.optimize import minimize

# DVS (Discrete Variable Stiffness) 磁性导丝求解器类
# 这是一个基于能量最小化的静态平衡模拟器
class DVSMagneticRodSolver:
    def __init__(self, segments, dl=0.005):
        """
        初始化导丝结构
        segments: 包含段信息的列表 [{'length': 长度, 'E': 弹性模量, 'is_magnet': 是否为磁铁, 'polarity': 极性}]
        dl: 空间分辨率 (单位: 米)
        """
        self.segments = segments
        self.total_length = sum(s['length'] for s in segments)
        
        self.dl = dl 
        self.N = int(np.round(self.total_length / self.dl)) + 1
        
        self.E = np.zeros(self.N - 1)
        self.rho = np.zeros(self.N - 1) # 每单元密度
        self.magnet_nodes = []
        self.magnet_moments = []
        
        current_node = 0
        for seg in segments:
            n_elem = int(np.round(seg['length'] / self.dl))
            seg_end = current_node + n_elem
            if seg_end > self.N - 1: seg_end = self.N - 1
            
            # 设置杨氏模量
            self.E[current_node : seg_end] = seg['E']
            # 设置物理属性
            if seg.get('is_magnet', False):
                self.magnet_nodes.append(current_node + n_elem // 2)
                # 磁化参数：直径 1mm, 1.2T
                radius_mag = 0.0005
                vol = np.pi * (radius_mag**2) * seg['length'] 
                m_val = (1.2 * vol / (4e-7 * np.pi)) * seg.get('polarity', 1.0)
                self.magnet_moments.append(m_val)
                self.rho[current_node : seg_end] = 10000.0 # 磁铁高密度
            else:
                self.rho[current_node : seg_end] = seg.get('density', 1100.0)
            
            current_node += n_elem
                
        # 物理常数 (用于重力计算)
        self.radius = 0.0005   # 0.5mm (截面半径)
        self.I = (np.pi * self.radius**4) / 4 # 惯性矩
        self.K_bend = self.E * self.I / self.dl # 计算弯曲刚度系数 K
        
        self.g_vec = np.array([0, 0, -9.81]) 
        self.area = np.pi * self.radius**2
        self.mass_per_seg = self.rho * self.area * self.dl # 基于数组的质量分配
        
        # 初始基座方向 (默认为沿 Z 轴)
        self.base_theta = 0.0
        self.base_phi = 0.0
        
    def get_geometry(self, x):
        """
        根据求解出的角度 x，重构导丝的三维中心线坐标
        x 包含了所有节点在俯仰(thetas)和方位(phis)上的局部弯曲角
        """
        n_seg = self.N - 1
        thetas = x[:n_seg] # 俯仰分布
        phis = x[n_seg:]   # 方位分布
        
        points = np.zeros((self.N, 3))
        # 初始点位于原点 (0,0,0)
        # 核心逻辑：利用角度累加 (cumsum) 得到全局切线方向，再进行一维积分得到坐标
        global_thetas = np.cumsum(thetas) + self.base_theta
        global_phis = np.cumsum(phis) + self.base_phi
        
        t_vecs = np.stack([
            np.sin(global_thetas) * np.cos(global_phis),
            np.sin(global_thetas) * np.sin(global_phis),
            np.cos(global_thetas)
        ], axis=1)
        
        points[1:] = np.cumsum(t_vecs * self.dl, axis=0)
        return points

    def total_energy(self, x, epm_pos=None, epm_m=None, B_uniform=None):
        """
        计算系统的总势能 = 弹性能 + 磁能 + 重力势能
        优化器的目标就是让这个值最小（即达到静态平衡）
        """
        n_seg = self.N - 1
        thetas = x[:n_seg]
        phis = x[n_seg:]
        
        # 1. 计算弹性能 (Bending Energy)
        e_elastic = 0.5 * np.sum(self.K_bend * (thetas**2 + phis**2))
        
        # 2. 计算磁能 (Magnetic Potential Energy)
        e_magnetic = 0
        global_thetas = np.cumsum(thetas) + self.base_theta
        global_phis = np.cumsum(phis) + self.base_phi
        
        # 实时重构坐标 (用于计算磁场距离和重力高度)
        points = self.get_geometry(x)
        
        if epm_pos is not None and epm_m is not None:
            mu0 = 4 * np.pi * 1e-7
            for idx, target_node in enumerate(self.magnet_nodes):
                i = target_node - 1
                p_i = points[target_node]
                r_vec = p_i - epm_pos
                r_mag = np.linalg.norm(r_vec)
                r_hat = r_vec / r_mag
                B_local = (mu0 / (4.0 * np.pi)) * (3.0 * np.dot(epm_m, r_hat) * r_hat - epm_m) / (r_mag**3)
                th = global_thetas[i]; ph = global_phis[i]
                dir_i = np.array([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)])
                m_vec = self.magnet_moments[idx] * dir_i
                e_magnetic += -np.dot(m_vec, B_local)
        
        elif B_uniform is not None:
            for idx, target_node in enumerate(self.magnet_nodes):
                i = target_node - 1
                th = global_thetas[i]; ph = global_phis[i]
                dir_i = np.array([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)])
                m_vec = self.magnet_moments[idx] * dir_i
                e_magnetic += -np.dot(m_vec, B_uniform)
                
        # 3. 重力势能 (E_g = - sum (F_g dot r))
        # 修正：F_g = m * g_vec, g_vec 为 [0, 0, -9.81]。
        # 势能 U = -F_g dot r = - (m * g_vec dot r) = m * 9.81 * z.
        seg_centers = (points[:-1] + points[1:]) / 2.0
        e_gravity = -np.sum(self.mass_per_seg * np.dot(seg_centers, self.g_vec))

        return e_elastic + e_magnetic + e_gravity

    def solve(self, epm_pos=None, epm_m=None, B_uniform=None, maxiter=200, ftol=1e-4, x0=None, disp=False):
        """
        主求解函数
        x0: 可选的初始猜测向量，若不提供则默认为微小值
        disp: 是否打印求解过程
        """
        if x0 is None:
            x0 = np.zeros((self.N - 1) * 2)
        res = minimize(self.total_energy, x0, args=(epm_pos, epm_m, B_uniform), method='L-BFGS-B', options={'maxiter': maxiter, 'ftol': ftol, 'disp': disp})
        return res.x, res.fun

if __name__ == "__main__":
    # 测试代码块：验证永磁铁偶极子场计算
    test_segments = [{'length': 0.110, 'E': 1e9, 'is_magnet': True, 'polarity': 1.0}]
    solver = DVSMagneticRodSolver(test_segments)
    
    # 定义一个外部永磁铁 (放在 10cm 开外，磁矩向上)
    epm_pos = np.array([0.0, 0.10, 0.0])
    epm_m = np.array([0.0, 10.0, 0.0]) 
    
    sol, energy = solver.solve(epm_pos=epm_pos, epm_m=epm_m)
    print("求得的总能量 (EPM模式):", energy)
    print("求得的尖端坐标:", solver.get_geometry(sol)[-1])
