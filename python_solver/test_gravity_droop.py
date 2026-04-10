import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mcr_python_solver import DVSMagneticRodSolver

def test_droop():
    # 110mm 导丝，全段为 10MPa (极软)
    segments_soft = [{'length': 0.110, 'E': 10e6, 'is_magnet': False}]
    # 110mm 导丝，全段为 1000MPa (硬)
    segments_hard = [{'length': 0.110, 'E': 1000e6, 'is_magnet': False}]
    
    solver_soft = DVSMagneticRodSolver(segments_soft)
    solver_soft.base_theta = np.deg2rad(90) # 水平放置
    
    solver_hard = DVSMagneticRodSolver(segments_hard)
    solver_hard.base_theta = np.deg2rad(90) # 水平放置
    
    # 无磁场，只受重力
    print("正在计算软导丝在重力下的下垂...")
    sol_soft, _ = solver_soft.solve()
    pts_soft = solver_soft.get_geometry(sol_soft)
    
    print("正在计算硬导丝在重力下的下垂...")
    sol_hard, _ = solver_hard.solve()
    pts_hard = solver_hard.get_geometry(sol_hard)
    
    plt.figure(figsize=(10, 6))
    plt.plot(pts_soft[:, 2]*1000, pts_soft[:, 1]*1000, 'r-', label='Soft (10 MPa) Droop')
    plt.plot(pts_hard[:, 2]*1000, pts_hard[:, 1]*1000, 'b--', label='Hard (1000 MPa) Droop')
    plt.axhline(0, color='black', linewidth=1, linestyle='--')
    
    plt.title("Gravity Droop Test (Horizontal Cantilever)", fontsize=14)
    plt.xlabel("Z (mm)")
    plt.ylabel("Y (mm) - Vertical Deflection")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.gca().set_aspect('equal')
    
    plt.savefig("test_gravity_droop.png")
    print(f"测试完成！")
    print(f"软导丝尖端下垂: {pts_soft[-1, 1]*1000:.2f} mm")
    print(f"硬导丝尖端下垂: {pts_hard[-1, 1]*1000:.2f} mm")

if __name__ == "__main__":
    test_droop()
