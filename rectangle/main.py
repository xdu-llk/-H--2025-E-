# -*- coding: utf-8 -*-
"""MaixCAM2 主程序 —— 上电两阶段。

    阶段 1  认数字（只跑一次，最多 10 s）
            相机切 416x416 -> YOLOv5 数字模型 -> 连续多帧确认
            -> 把 N 发给底盘主控（重复 3 s）-> 切回 416x260

    阶段 2  矩形识别瞄准（全程）
            相机保持 416x260 -> 每帧把 err_x 发给云台主控

⚠️ 为什么要切分辨率：数字模型要 416x416 正方形输入；而矩形识别那套阈值
   （MIN_AREA 等）全是按 416x260 标定出来的，换分辨率就全作废。各用各的。

两路串口都是单向发：

    UART4  A21/A22  ->  云台主控    每帧发 err_x
    UART1  A30/A31  ->  底盘主控    只在认出数字时发 N

算法与串口实现全在 no_canny.py（camera_display.py 已弃用，改用 no_canny）。
"""

import cv2

import no_canny as v


def main():
    # AX630C 双核，给 OpenCV 用上
    cv2.setUseOptimized(True)
    cv2.setNumThreads(2)

    cam = v.init_camera()                            # 按 CAM_W x CAM_H 建相机
    v.run_digit_phase(cam, v.init_uart_chassis())    # 内部切 416x416，跑完切回
    v.run_find_rects(cam)                            # 瞄准，跑到底


if __name__ == "__main__":
    main()
