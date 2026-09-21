---
name: qd4310-uart-one-shot-protocol
description: "QD4310 云台的 UART 是\"一发一收\"——发一条必须等到反馈或超时才能发下一条，两条不能背靠背"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 706d5937-1cdb-411d-88d4-39e050b07a43
  modified: 2026-09-21T13:37:49.388Z
---

QD4310 云台电机（qdrive.com.cn）**支持 CAN 和 UART 两种控制**。本项目用 UART
（CAN 需要外接收发器，暂时没有）。

**UART 数据包 = `[ID] + [CAN 包原样] + [CRC8]`**（手册 6.2.2 "中间数据格式均保持一致"）
- 控制包 5 字节：`ID, 指令类型, 控制量(2, 小端), CRC8`
- 反馈包 10 字节：`ID, 电机状态, 错误码, 电流(2), 转速(2), 角度(2), CRC8`
- CRC8：多项式 `0x07`、初值 `0x00`、异或 `0x00`、不反转 → 标准 MSB-first CRC-8
- ⚠️ **手册没写 CRC 覆盖范围**，按自然读法取"包内 CRC 之前的全部字节"

## 最关键的一条：一发一收

手册 6.2.3 明确要求：**发一条 → 等到反馈报文或接收超时 → 才能发下一条。
严禁收发异步，必须串行。**

**踩过的坑**：从 CAN 版迁移过来的代码习惯性地背靠背发两条
（`gimbal_clear_error(); gimbal_enable();`），第二条**必然**被"还在等反馈"挡掉——
结果**电机的使能指令从来没发出去过**，表现是"电机不转"。

**正确做法**：拆成状态机，每拍只推进一步，发成功了才进下一步
（见 `empty.c` 里的 `gim_step`）。**不依赖任何"猜多久够"的延时。**

## 等待窗口要覆盖整个发送周期

超时（`GIMBAL_RX_TIMEOUT_TICKS`）**必须等于发送周期**（`SEND_PERIOD_TICKS`）。
设短了会留出一段 IDLE 空档，落在里面的字节全被丢弃，帧永远攒不齐——
表现是"原始字节能收到（`RB` 在涨）但一帧都解析不出来（`M=ME=0`）"。

**How to apply**：改发送周期时，超时要跟着改。相关：[[mspm0-sysconfig-nvic-gotcha]]、
[[project-e-ti-aiming]]
