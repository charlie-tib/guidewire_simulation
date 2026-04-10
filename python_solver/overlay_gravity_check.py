import cv2
import numpy as np
import pandas as pd
import os

def overlay():
    img_path = '/home/wen-zheng/guidewire_simulation/calibration_photo_1.png'
    csv_path = '/home/wen-zheng/guidewire_simulation/centerline_real_gravityonly.csv'
    ruler_path = '/home/wen-zheng/guidewire_simulation/ruler_calibration.txt'
    
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        return
    if not os.path.exists(img_path):
        print(f"Error: {img_path} not found.")
        return
        
    img = cv2.imread(img_path)
    df = pd.read_csv(csv_path)
    
    # 读取比例尺
    px_per_mm = 3.2 # 默认值
    if os.path.exists(ruler_path):
        with open(ruler_path, 'r') as f:
            for line in f:
                if 'px_per_mm' in line:
                    px_per_mm = float(line.split('=')[1])
    print(f"Using px_per_mm = {px_per_mm}")

    # 基座位置分析：由 Subagent 确定的 Photo 1 基部坐标
    base_px = np.array([609, 137]) 
    print(f"Using base_px = {base_px}")

    # CSV 里的坐标 (x_mm, y_mm)
    points_mm = df[['x_mm', 'y_mm']].values
    
    # 转换为像素：Base + mm * px/mm
    # 注意：Image Y 向下为正，CSV Y 也向下为正，所以直接加
    points_px = base_px + points_mm * px_per_mm
    
    # 绘制
    for i in range(len(points_px)-1):
        pt1 = tuple(points_px[i].astype(int))
        pt2 = tuple(points_px[i+1].astype(int))
        # 红色粗线代表标注数据
        cv2.line(img, pt1, pt2, (0, 0, 255), 4)
        if i % 5 == 0:
            cv2.circle(img, pt1, 6, (0, 255, 0), -1) # 绿色节点
            
    # 特别标出 Base 以防点歪了
    cv2.circle(img, tuple(base_px), 10, (255, 0, 0), -1) 
    cv2.putText(img, "Base (0,0)", (base_px[0]+15, base_px[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
        
    save_path = '/home/wen-zheng/guidewire_simulation/gravity_overlay_check.png'
    cv2.imwrite(save_path, img)
    print(f"SUCCESS: Result saved to {save_path}")

if __name__ == "__main__":
    overlay()
