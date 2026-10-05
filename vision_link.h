 /*
 * vision_link —— MSPM0 与 MaixCam2 之间的视觉数据链路
 *
 * 承载：UART3 (PB2 = TX, PB3 = RX)，115200 8N1，FIFO 使能，RX 阈值 1/2 full
 * 方向：MaixCam2 -> MSPM0 单向
 *
 * 帧格式（定长 6 字节）:
 *     AA 55 STATUS ERR_X_LO ERR_X_HI CHECKSUM
 *       STATUS   : 1 = 检测到靶纸, 0 = 未检测到
 *       ERR_X    : int16 **小端、有符号**
 *       CHECKSUM : 前 5 字节之和 & 0xFF
 *
 * MaixCam 侧对应 no_canny.py 的 build_uart_frame()，
 * 用 pack("<Bh", status, err_x) 打包。(camera_display.py 已弃用)
 */

#ifndef VISION_LINK_H_
#define VISION_LINK_H_

#include <stdbool.h>
#include <stdint.h>

/* 与 MaixCam 侧 STATUS_TARGET_VALID 对应 */
#define VISION_STATUS_TARGET_VALID  1u

typedef struct {
    uint8_t status;     /* 0 = 未检测到靶纸, 1 = 检测到 */
    int16_t err_x;      /* err_x = 靶心 - 激光点，单位是【原图像素】(416x260 画幅) */
} vision_msg_t;

/* 初始化: 使能 UART3 的 NVIC 中断。
 * ⚠️ 必须在 SYSCFG_DL_init() 之后调用 —— 漏调用的表现是"能发不能收",
 *    一个字节都进不来, 而且不会有任何报错。 */
void vision_link_init(void);

/* 取最近一帧。
 *
 * 有新帧返回 true 并填充 *out；自上次取走之后没有新帧则返回 false。
 * 只保留最新一帧 —— 控制环要的是"现在偏差多少"，积压的旧帧没有意义。
 *
 * ⚠️ 必须在任务上下文调用，不要在 ISR 里调（内部会关中断）。
 * ⚠️ 量纲是像素不是毫米，整定 PID 时按像素来。
 */
bool vision_link_get(vision_msg_t *out);

/* 调试计数。串口出问题时先看这两个：
 *   g_vision_overrun   FIFO 溢出次数 —— 不为 0 说明 CPU 被别处拖住了
 *   g_vision_bad_csum  校验失败次数 —— 持续增长说明有干扰或线序/波特率不对
 * 两者都恒为 0 才是健康的。 */
extern volatile uint32_t g_vision_overrun;
extern volatile uint32_t g_vision_bad_csum;

#endif /* VISION_LINK_H_ */
