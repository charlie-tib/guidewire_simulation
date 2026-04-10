"""
交互式导丝中心线标注工具
使用方法：
1. 运行脚本后会显示照片
2. 用鼠标左键沿导丝中心线依次点击（从夹持点开始，到尖端结束）
3. 右键撤销上一个点
4. 按 'r' 键标注尺子上的两个刻度（用于 px→mm 标定）
5. 按 's' 保存并退出
6. 按 'q' 不保存退出
"""
import cv2
import numpy as np

# ============================================================
# 配置
# ============================================================
IMAGE_PATH = '/home/wen-zheng/guidewire_simulation/calibration_photo_6.png'
OUTPUT_PATH = '/home/wen-zheng/guidewire_simulation/centerline_real_6.csv'
RULER_OUTPUT = '/home/wen-zheng/guidewire_simulation/ruler_calibration.txt'

# 全局变量
points = []          # 导丝中心线的点
ruler_points = []    # 尺子刻度标定点（需要2个）
epm_point = None     # EPM 中心点
ruler_mm = 0         # 两个尺子刻度之间的实际距离 (mm)
mode = 'centerline'  # 当前模式：'centerline', 'ruler', 'epm'
img_display = None
img_original = None

def draw_all():
    """重绘所有标注"""
    global img_display
    img_display = img_original.copy()
    
    # 绘制导丝中心线点和连线 (红色)
    if len(points) > 0:
        for i, pt in enumerate(points):
            cv2.circle(img_display, pt, 4, (0, 0, 255), -1)
            if i > 0:
                cv2.line(img_display, points[i-1], pt, (0, 0, 255), 2)
        cv2.putText(img_display, "Start", (points[0][0]+10, points[0][1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # 绘制尺子标定点 (蓝色)
    for i, pt in enumerate(ruler_points):
        cv2.circle(img_display, pt, 6, (255, 0, 0), -1)
        cv2.putText(img_display, f"R{i+1}", (pt[0]+10, pt[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    if len(ruler_points) == 2:
        cv2.line(img_display, ruler_points[0], ruler_points[1], (255, 0, 0), 2)
    
    # 绘制 EPM 标记点 (橙色)
    if epm_point is not None:
        cv2.circle(img_display, epm_point, 12, (0, 165, 255), 3) # 橙色圈
        cv2.circle(img_display, epm_point, 2, (0, 165, 255), -1)
        cv2.putText(img_display, "EPM CENTER", (epm_point[0]+15, epm_point[1]-15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    # 状态栏
    status = f"Mode: {mode.upper()} | Pts: {len(points)} | Ruler: {len(ruler_points)}/2"
    if epm_point: status += " | EPM OK"
    cv2.putText(img_display, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    help_text = "Click=add | 'r'=ruler | 'e'=click EPM | 's'=save | 'q'=quit"
    cv2.putText(img_display, help_text, (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

def mouse_callback(event, x, y, flags, param):
    global mode, epm_point
    if event == cv2.EVENT_LBUTTONDOWN:
        if mode == 'centerline':
            points.append((x, y))
        elif mode == 'ruler':
            if len(ruler_points) < 2:
                ruler_points.append((x, y))
                if len(ruler_points) == 2: mode = 'centerline'
        elif mode == 'epm':
            epm_point = (x, y)
            print(f"  EPM Marker set at: ({x}, {y})")
            mode = 'centerline' # 选完自动切回
        
        draw_all()
        cv2.imshow("Annotate Guidewire", img_display)
    
    elif event == cv2.EVENT_RBUTTONDOWN:
        if mode == 'centerline' and len(points) > 0:
            points.pop()
        elif mode == 'ruler' and len(ruler_points) > 0:
            ruler_points.pop()
        elif mode == 'epm':
            epm_point = None
        draw_all()
        cv2.imshow("Annotate Guidewire", img_display)

# ============================================================
# 主逻辑
# ============================================================
img_original = cv2.imread(IMAGE_PATH)
if img_original is None:
    print(f"ERROR: Cannot load {IMAGE_PATH}")
    exit(1)

print("=" * 60)
print("  Interactive Guidewire Centerline Annotation Tool (Ext. EPM)")
print("=" * 60)
print("Instructions:")
print("  1. LEFT click guidewire centerline (start -> tip)")
print("  2. Press 'r' then click 2 ruler marks (for scale)")
print("  3. Press 'e' then click the center of the hand-held EPM")
print("  4. Press 's' to save and exit")
print("=" * 60)

draw_all()
cv2.imshow("Annotate Guidewire", img_display)
cv2.setMouseCallback("Annotate Guidewire", mouse_callback)

while True:
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('r'):
        mode = 'ruler'
        ruler_points.clear()
        print("\n--- RULER MODE: Click 2 points on the ruler ---")
    
    elif key == ord('e'):
        mode = 'epm'
        print("\n--- EPM MODE: Click the center of the external magnet ---")

    elif key == ord('s'):
        # 保存标定比例
        if len(ruler_points) < 2:
            print("\nWARNING: Ruler not calibrated! Using default 13.6 px/mm")
            px_per_mm = 13.6
        else:
            px_dist = np.sqrt((ruler_points[1][0]-ruler_points[0][0])**2 + 
                              (ruler_points[1][1]-ruler_points[0][1])**2)
            try:
                ruler_mm = float(input(f"\nEnter dist between ruler points (mm): "))
                px_per_mm = px_dist / ruler_mm
            except:
                px_per_mm = 13.6
        
        if len(points) < 2:
            print("ERROR: Need centerline points!")
            continue
        
        # 转换并保存
        pts = np.array(points, dtype=float)
        phys_x = (pts[:, 0] - pts[0, 0]) / px_per_mm
        phys_y = (pts[:, 1] - pts[0, 1]) / px_per_mm
        np.savetxt(OUTPUT_PATH, np.column_stack([phys_x, phys_y]),
                   delimiter=',', header='x_mm,y_mm', comments='')
        
        # 转换并保存 EPM 物理位置 (相对于导丝起始点)
        if epm_point:
            epm_phys_x = (epm_point[0] - points[0][0]) / px_per_mm
            epm_phys_y = (epm_point[1] - points[0][1]) / px_per_mm
            with open('/home/wen-zheng/guidewire_simulation/epm_coords_val.txt', 'w') as f:
                f.write(f"epm_x_mm={epm_phys_x:.4f}\nepm_y_mm={epm_phys_y:.4f}\n")
            print(f"EPM Coords saved: ({epm_phys_x:.1f}, {epm_phys_y:.1f}) mm")
        
        print(f"Successfully saved to {OUTPUT_PATH}")
        break
    
    elif key == ord('q'):
        print("\nExiting without saving.")
        break

cv2.destroyAllWindows()
