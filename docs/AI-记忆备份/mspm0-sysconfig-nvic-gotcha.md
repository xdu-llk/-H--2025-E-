---
name: mspm0-sysconfig-nvic-gotcha
description: "MSPM0 SysConfig 只生成外设级中断使能，NVIC 那一层必须自己开——漏了就是\"能发不能收\"且毫无报错"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 706d5937-1cdb-411d-88d4-39e050b07a43
  modified: 2026-09-21T13:37:41.865Z
---

**MSPM0 的 SysConfig 只生成外设级的中断使能**（`DL_UART_Main_enableInterrupt` /
`DL_MCAN_enableInterrupt`），**NVIC 那一层没有生成**，必须在自己的 init 里手动调：

```c
NVIC_EnableIRQ(UART_1_INST_INT_IRQN);   // ← 每个用中断的模块都要有
```

漏掉的后果：**中断永远不触发，数据卡在 FIFO 里，一个字节都收不到**——
而且**编译期和运行期都不报任何错**。极难查。

本项目里这个坑踩了两次（MCAN 一次、UART3 视觉一次），后来才形成惯例：
**每加一个用中断的外设，检阅清单里必须有"NVIC 使能"这一条。**

各模块的 NVIC 使能位置：
- `gimbal.c: gimbal_init()` — UART1
- `gyro_link.c: gyro_link_init()` — UART2
- `vision_link.c: vision_link_init()` — UART3
- `debug_uart.c: debug_uart_init()` — UART0

**How to apply**：新建或检阅任何用中断的外设时，第一件事就是确认
`NVIC_EnableIRQ(<外设>_INST_INT_IRQN)` 存在。相关：[[qdc4310-uart-one-shot-protocol]]
