"""
生成一个带有弯曲的血管 STL 模型的脚本
无需 numpy 依赖，增强环境兼容性
"""
import math
import os

def generate_vessel_stl(filename, num_segments=50, radius=0.005, length=0.3, curvature=2.0):
    """
    生成一个圆柱形或弯曲的血管 STL 文件
    
    参数:
        filename: 输出文件名
        num_segments: 圆周分段数
        radius: 血管半径 (米)
        length: 血管长度 (米)
        curvature: 曲率 (1/m)，0 为直管
    """
    
    print(f"正在生成血管模型: {filename}")
    print(f"  分段数: {num_segments}")
    print(f"  半径: {radius} m")
    print(f"  长度: {length} m")
    if curvature > 0:
        print(f"  曲率: {curvature} 1/m (半径 {1/curvature:.2f} m)")
    
    # 辅助函数：根据 z 计算中心点偏移 (简单圆弧)
    def get_center(z):
        if curvature <= 0:
            return 0.0, 0.0
        R_curve = 1.0 / curvature
        theta = z / R_curve
        x = R_curve * (1.0 - math.cos(theta))
        return x, 0.0

    # 绘制轴向分段
    num_z_steps = 41
    z_steps = [i * length / (num_z_steps - 1) for i in range(num_z_steps)]

    with open(filename, 'w') as f:
        f.write("solid vessel\n")
        
        for k in range(len(z_steps) - 1):
            z1, z2 = z_steps[k], z_steps[k+1]
            cx1, cy1 = get_center(z1)
            cx2, cy2 = get_center(z2)
            
            for i in range(num_segments):
                theta1 = 2 * math.pi * i / num_segments
                theta2 = 2 * math.pi * (i + 1) / num_segments
                
                # 计算两个圆环上的点
                p1_1 = [cx1 + radius * math.cos(theta1), cy1 + radius * math.sin(theta1), z1]
                p1_2 = [cx1 + radius * math.cos(theta2), cy1 + radius * math.sin(theta2), z1]
                p2_1 = [cx2 + radius * math.cos(theta1), cy2 + radius * math.sin(theta1), z2]
                p2_2 = [cx2 + radius * math.cos(theta2), cy2 + radius * math.sin(theta2), z2]
                
                # 三角形 1
                f.write(f"  facet normal 0 0 0\n")
                f.write(f"    outer loop\n")
                f.write(f"      vertex {p1_1[0]} {p1_1[1]} {p1_1[2]}\n")
                f.write(f"      vertex {p1_2[0]} {p1_2[1]} {p1_2[2]}\n")
                f.write(f"      vertex {p2_1[0]} {p2_1[1]} {p2_1[2]}\n")
                f.write(f"    endloop\n")
                f.write(f"  endfacet\n")
                
                # 三角形 2
                f.write(f"  facet normal 0 0 0\n")
                f.write(f"    outer loop\n")
                f.write(f"      vertex {p1_2[0]} {p1_2[1]} {p1_2[2]}\n")
                f.write(f"      vertex {p2_2[0]} {p2_2[1]} {p2_2[2]}\n")
                f.write(f"      vertex {p2_1[0]} {p2_1[1]} {p2_1[2]}\n")
                f.write(f"    endloop\n")
                f.write(f"  endfacet\n")
        
        # 端盖
        for z_val, is_start in [(0, True), (length, False)]:
            cx, cy = get_center(z_val)
            norm_z = -1 if is_start else 1
            for i in range(num_segments):
                theta1 = 2 * math.pi * i / num_segments
                theta2 = 2 * math.pi * (i + 1) / num_segments
                v1 = [cx + radius * math.cos(theta1), cy + radius * math.sin(theta1), z_val]
                v2 = [cx + radius * math.cos(theta2), cy + radius * math.sin(theta2), z_val]
                f.write(f"  facet normal 0 0 {norm_z}\n")
                f.write(f"    outer loop\n")
                f.write(f"      vertex {cx} {cy} {z_val}\n")
                if is_start:
                    f.write(f"      vertex {v1[0]} {v1[1]} {v1[2]}\n")
                    f.write(f"      vertex {v2[0]} {v2[1]} {v2[2]}\n")
                else:
                    f.write(f"      vertex {v2[0]} {v2[1]} {v2[2]}\n")
                    f.write(f"      vertex {v1[0]} {v1[1]} {v1[2]}\n")
                f.write(f"    endloop\n")
                f.write(f"  endfacet\n")
        
        f.write("endsolid vessel\n")
    
    print(f"生成完成!")

if __name__ == "__main__":
    mesh_dir = "/home/wen-zheng/meshes"
    if not os.path.exists(mesh_dir):
        os.makedirs(mesh_dir)
        
    generate_vessel_stl(os.path.join(mesh_dir, "vessel.stl"), 
                        num_segments=32,
                        radius=0.005,
                        length=0.3,
                        curvature=2.0)