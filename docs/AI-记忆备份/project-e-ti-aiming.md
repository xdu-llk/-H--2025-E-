---
name: project-e-ti-aiming
description: my_ti_control = 2025 电赛 E 题招新改编版（一维云台激光瞄准）；三块板分工、四路串口、当前进度与待办
metadata: 
  node_type: memory
  type: project
  originSessionId: 706d5937-1cdb-411d-88d4-39e050b07a43
  modified: 2026-09-21T13:37:33.111Z
---

`my_ti_control` 是**2025 电赛 E 题的招新改编版**：自动寻迹小车 + 车载一维云台激光瞄准。主控 MSPM0G3507（[[board-is-lckfb-tianmengxing]]）。

## 招新题（我们做的）vs 2025 E 题原题

| | 招新题 | E 题原题（仅参考） |
|---|---|---|
| 赛道 | 椭圆：直道 1.5m + 弯道 r=0.5m，顺时针 1→2→3→4→1 | 100×100cm 正方形，逆时针 |
| 靶 | **一条竖直中线** ⇒ 只需一维（偏航） | 靶心红点 + r=2/4/6/8/10cm 同心圆 ⇒ 需二维 |
| 要求 | (1) 一圈≤20s (2) 识别数字N(1~4)定点停车 (3) 任意姿态 2s 内击中竖线≤3cm (4)(5)(6) 2→3、4→1 段行进中打靶≤5cm | 2s/4s 击中靶心 D₁≤2cm；发挥部分沿 r=6cm 红圆画圆 |

原题 PDF 图存在 `my_ti_control/docs/e2025_*.png`（7 张，3 和 7 是重复页眉）。
抓法记在 [[nuedc-site-requires-referer]]。

## 三块板 / 四路串口

| 外设 | 引脚 | 对端 | 内容 |
|---|---|---|---|
| UART0 | PA10/PA11 | USB-TTL | 调试打印 |
| UART1 | PA8/PA9 | **QD4310 云台** | 5 字节控制包 / 10 字节反馈包，**一发一收** |
| UART2 | PB17/PB18 | 汇电籽-601 陀螺 | 6/24 字节变长帧 |
| UART3 | PB2/PB3 | MaixCam2 | 6 字节帧 `AA 55 STATUS ERR_X(int16 LE) SUM` |

MaixCam2 侧另有 **UART1（A30/A31）直连底盘主控**发数字 N（不经过我们）。
CAN 已从 SysConfig 删除；旧 CAN 驱动停在 `gimbal_can.c.disabled`。

## 当前进度（2026-09-21）

**已上板验证**：UART3 视觉链路、UART2 陀螺、UART1 云台（电机跟转、200Hz 丝滑）、
视觉闭环符号、前馈符号、激光常亮、MaixCam 两阶段程序（认数字 + 矩形识别，切分辨率后矩形识别正常）。

**待办**：`LASER_X_PX` 标定（等激光）；`GYRO_LSB_PER_DPS` 实测（占位 16.4 = ±2000dps 档，待确认）；
`AIM_GAIN_RATE` 精调（现 0.037，λ=0.3，理论值）；整车联调（等底盘）。

**已知未解决**：云台反馈收不到（`M=0`）——详见 `my_ti_control/docs/BUGS.md #1`，
**不影响打靶**（控制环靠视觉闭环，不用电机角度）。排查时把 `DEBUG_GIMBAL_SNOOP` 打开。

**How to apply**：用户对题目已有完整理解，只需辅助执行，不要重复展开方案设计。
相关：[[mspm0-sysconfig-nvic-gotcha]]、[[qdc4310-uart-one-shot-protocol]]
