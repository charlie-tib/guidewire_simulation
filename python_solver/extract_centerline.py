"""
从标定照片中提取导丝形态 - 改进版
使用更精确的 ROI 和颜色筛选，聚焦到导丝本体
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt

# 加载照片
img = cv2.imread('/home/wen-zheng/guidewire_simulation/calibration_photo_3.png')
h, w = img.shape[:2]
print(f"图像尺寸: {w}x{h}")

# ============================================================
# 第一步：精确标定像素比例（用尺子刻度）
# ============================================================
# 从照片中观察：尺子在底部，刻度 "1" 到 "9" 之间
# 手动测量两个清晰刻度的像素 x 坐标：
# 刻度 "1" 约在 x=158
# 刻度 "9" 约在 x=1247
# 距离 = 80mm (从1到9)
# px_per_mm = (1247 - 158) / 80 = 13.61
px_per_mm = (1247 - 158) / 80.0
print(f"像素比例: {px_per_mm:.2f} px/mm")

# ============================================================
# 第二步：手动标注导丝中心线关键点
# ============================================================
# 从照片中仔细观察导丝的位置：
# 导丝从夹持点开始向右延伸，有明显下垂
# 夹持点（左端）：大约 x=460, y=375（导丝离开夹持边缘的位置）
# 第一个磁铁（黑色段）中心：约 x=580, y=395
# 磁铁之间的软段：可以看到轻微弯曲
# 第二个磁铁：约 x=695, y=410
# 尖端（右端）：约 x=870, y=440

# 我通过目测从照片中标注了一些关键控制点
# 这些点沿着导丝中心线（像素坐标）
key_points_px = np.array([
    [460, 375],   # 夹持点（起始）
    [490, 377],   # 第一段软质开始
    [520, 380],   # 
    [550, 385],   # 
    [575, 390],   # 第一块磁铁开始区域
    [590, 393],   # 第一块磁铁中心
    [610, 395],   # 第一块磁铁结束
    [640, 398],   # 第二段软质
    [670, 403],   # 
    [695, 408],   # 第二块磁铁附近
    [720, 413],   # 
    [750, 418],   # 第三段软质
    [780, 425],   # 
    [810, 432],   # 第三块磁铁附近
    [840, 438],   # 
    [870, 442],   # 尖端
])

# 转换为物理坐标 (mm)
# x: 从起点归零
# y: 从起点归零，向下为正（下垂方向）
phys_x = (key_points_px[:, 0] - key_points_px[0, 0]) / px_per_mm
phys_y = (key_points_px[:, 1] - key_points_px[0, 1]) / px_per_mm

# 用样条插值得到更平滑的曲线
from scipy.interpolate import CubicSpline
cs_x = np.linspace(0, phys_x[-1], 200)
cs = CubicSpline(phys_x, phys_y)
cs_y = cs(cs_x)

print(f"\n=== 测量结果 ===")
print(f"悬臂总长: {phys_x[-1]:.1f} mm")
print(f"尖端下垂量: {phys_y[-1]:.2f} mm")

# ============================================================
# 第三步：可视化验证标注
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# 左：照片 + 标注点
axes[0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
axes[0].plot(key_points_px[:, 0], key_points_px[:, 1], 'r.-', linewidth=2, markersize=8, label='Manual markers')
axes[0].set_title('Photo with Manual Markers')
axes[0].legend()

# 右：物理坐标曲线
axes[1].plot(cs_x, cs_y, 'b-', linewidth=2, label='Spline interpolation')
axes[1].plot(phys_x, phys_y, 'ro', markersize=6, label='Key points')
axes[1].invert_yaxis()
axes[1].set_xlabel('Horizontal distance (mm)')
axes[1].set_ylabel('Vertical droop (mm)')
axes[1].set_title(f'Guidewire Droop Profile (L={phys_x[-1]:.1f}mm, tip={phys_y[-1]:.1f}mm)')
axes[1].grid(True)
axes[1].legend()
axes[1].set_aspect('equal')

plt.tight_layout()
save_path = '/home/wen-zheng/guidewire_simulation/calibration_extraction_v2.png'
plt.savefig(save_path, dpi=150)
print(f"Saved: {save_path}")

# 保存物理坐标数据
np.savetxt('/home/wen-zheng/guidewire_simulation/centerline_real.csv',
           np.column_stack([cs_x, cs_y]), delimiter=',',
           header='x_mm,y_mm', comments='')
print("Centerline saved: centerline_real.csv")
