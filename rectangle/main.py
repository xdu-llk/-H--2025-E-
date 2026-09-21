# -*- coding: utf-8 -*-
"""MaixCAM2 主程序 —— 整场流程都摆在这里。

两个阶段：

    阶段 1  认数字（整场只跑一次）
            相机切 416x416 -> 跑 YOLOv5 数字模型 -> 连续多帧确认
            -> 通过第二路串口把数字 N 发给底盘 -> 切回 416x260

    阶段 2  矩形识别瞄准（全程）
            跑原来那套矩形识别 -> 每帧把 err_x 发给云台主控

两路串口是分开的，都是单向发：

    UART4  A21/A22  ->  云台主控    每帧发 err_x
    UART1  A30/A31  ->  底盘主控    只发一次数字 N

⚠️ 为什么要切分辨率：
    YOLO 训练用的是正方形输入；而矩形识别那套阈值（MIN_AREA 等）全是按
    416x260 标定出来的，换分辨率就全作废。所以两个阶段各用各的。
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

    # --- 阶段 1: 认数字，只跑一次 ---
    # 认完内部会切回 416x260，并把 N 重复发给底盘（不需要反向 ACK）
    cd.run_digit_phase(cam, uart_chassis)

    # --- 阶段 2: 瞄准，跑到底 ---
    cd.run_aim_loop(cam, disp, uart_gimbal)


if __name__ == "__main__":
    main()
