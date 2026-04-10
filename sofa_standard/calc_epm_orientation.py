import numpy as np

def calculate_epm_orientation(epm_pos, tip_pos, B_target):
    """
    根据目标场强方向 B_target，反算 EPM 的磁矩方向 m
    epm_pos, tip_pos: 3D 坐标 (m)
    B_target: 目标场强向量 (T)
    """
    r_vec = tip_pos - epm_pos
    r_mag = np.linalg.norm(r_vec)
    r_hat = r_vec / r_mag
    
    # 磁极子系数 mu0/(4*pi)
    mu0_4pi = 1e-7
    
    # 构造场强矩阵 A，使得 B = A * m
    # A = (mu0/4pi * 1/r^3) * (3 * r_hat * r_hat.T - I)
    I = np.eye(3)
    A = (mu0_4pi / (r_mag**3)) * (3 * np.outer(r_hat, r_hat) - I)
    
    # 反求 m = A_inv * B_target
    try:
        m_vec = np.linalg.solve(A, B_target)
        # 归一化得到朝向
        m_dir = m_vec / np.linalg.norm(m_vec)
        return m_dir
    except np.linalg.LinAlgError:
        print("矩阵奇异，可能距离太近或方向垂直")
        return None

if __name__ == "__main__":
    # 示例：EPM 在原点，Tip 在 (0.05, 0, 0)，想要在 Tip 处产生 +Y 方向的场
    epm = np.array([0, 0, 0])
    tip = np.array([0.05, 0, 0])
    B_goal = np.array([0, 0.01, 0]) # 0.01 Tesla along Y
    
    m_orient = calculate_epm_orientation(epm, tip, B_goal)
    print(f"EPM 应该朝向: {m_orient}")
    
    # 验证
    r = tip - epm
    rm = np.linalg.norm(r)
    rh = r / rm
    B_check = 1e-7 / (rm**3) * (3 * np.dot(m_orient, rh) * rh - m_orient)
    print(f"验证生成的 B 场方向: {B_check / np.linalg.norm(B_check)}")
