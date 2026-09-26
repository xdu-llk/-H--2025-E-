# -*- coding: utf-8 -*-
"""MaixCAM2 主程序 —— 整场流程。

只有一个主循环：矩形识别瞄准（全程），数字识别是嵌在里面的【并行任务】：

    瞄准循环
      ├── 每帧:  矩形检测 -> 把 err_x 发给云台主控
      └── 并行:  瞄准稳定 -> 开数字识别 -> 认出 -> 发底盘 -> 关掉数字模型

⚠️ 摄像头分辨率全程 416x260，**一次都不切**：
    · 矩形识别那套阈值（MIN_AREA 等）全按 416x260 标定，换分辨率就作废
    · 数字模型要 416x416 正方形 -> 对画面【居中裁 260x260 再缩放】，
      只动数据，不动相机设置

两路串口都是单向发：

    UART4  A21/A22  ->  云台主控    每帧发 err_x
    UART1  A30/A31  ->  底盘主控    只在认出数字时发 N（发完等 ACK，超时也继续）
"""

import camera_display as cd


def main():
    # AX630C 双核，给 OpenCV 用上
    cd.cv2.setUseOptimized(True)
    cd.cv2.setNumThreads(2)

    # --- 建好设备 ---
    # init_camera() 会按 CAM_W x CAM_H (416x260) 建相机，并锁曝光
    cam = cd.init_camera()
    disp = cd.display.Display()
    uart_gimbal = cd.init_uart()            # 发给云台主控
    uart_chassis = cd.init_uart_chassis()   # 发给底盘

    # --- 主循环: 瞄准 + 数字识别（数字识别在里面自动开关）---
    cd.run_aim_loop(cam, disp, uart_gimbal, uart_chassis)


if __name__ == "__main__":
    main()
