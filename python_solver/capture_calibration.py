"""
导丝标定连拍脚本
使用 Intel RealSense D405 拍摄
逻辑：按下 's' 开启 1秒一张的自动连拍，按下 'q' 退出
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

# 配置 RealSense 管线
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

try:
    pipeline.start(config)
except Exception as e:
    print(f"❌ 启动相机失败: {e}")
    exit(1)

print("等待相机自适应曝光...")
for _ in range(30):
    pipeline.wait_for_frames()

print("=" * 50)
print("📷 导丝标定 [1秒连拍模式] 工具")
print("=" * 50)
print("操作说明：")
print("  's' - 【开始】每秒自动拍摄")
print("  'q' - 【结束/退出】程序")
print("=" * 50)

save_count = 0
capturing = False
last_save_time = 0

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        
        frame = np.asanyarray(color_frame.get_data())
        current_time = time.time()
        
        # 逻辑判定：如果处于连拍模式且距离上张照片超过1秒，则保存
        if capturing and (current_time - last_save_time >= 1.0):
            save_count += 1
            filename = f"calibration_photo_{save_count}.png"
            cv2.imwrite(filename, frame)
            # 同时更新“最新图”，方便其他脚本读取
            cv2.imwrite("calibration_photo_latest.png", frame)
            last_save_time = current_time
            print(f"✅ 已自动保存: {filename} (Total: {save_count})")

        # 画面实时提示
        display = frame.copy()
        status_text = "AUTO CAPTURING..." if capturing else "STANDBY - Press 's' to start"
        color = (0, 0, 255) if capturing else (0, 255, 0)
        cv2.putText(display, status_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(display, f"Count: {save_count}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        cv2.imshow("D405 Calibration Feed", display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            if not capturing:
                capturing = True
                last_save_time = time.time() # 立即触发第一张
                print(">>> 启动每秒连拍...")
        elif key == ord('q'):
            print(">>> 停止拍摄并退出。")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print(f"\n任务结束，共保存了 {save_count} 张照片。")
