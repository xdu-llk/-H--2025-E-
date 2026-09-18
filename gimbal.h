/*
 * gimbal —— QD4310 无刷云台电机驱动 (UART 版)
 *
 * 硬件
 * ---------------------------------------------------------------------------
 *   MSPM0G3507  UART1 :  TX = PA8, RX = PA9     115200 8N1
 *   接电机 SH1.0-3Pin 的 R/T/G 三线, 高电平 3.3V, 直连不用电平转换
 *   (电机的 CAN 版驱动保留在 gimbal_can.c.disabled, 当前不参与编译)
 *
 * 数据包格式 (出自 QD4310 使用手册 6.2.2 节)
 * ---------------------------------------------------------------------------
 *   控制数据包  共 5 字节
 *       [0] ID   [1] 指令类型   [2-3] 控制量(小端)   [4] CRC8
 *
 *   反馈数据包  共 10 字节
 *       [0] ID   [1] 电机状态   [2] 错误码   [3-4] 电流
 *       [5-6] 转速   [7-8] 角度   [9] CRC8
 *
 *   CRC8: 多项式 0x07, 初值 0x00, 结果异或 0x00, 输入输出均不反转。
 *   ⚠️ 手册**没有写明 CRC 覆盖哪些字节**, 这里按自然读法取"包内 CRC 之前的
 *      全部字节"(控制包 4 字节 / 反馈包 9 字节)。若实测 g_rx_err 狂涨,
 *      先怀疑这一条。
 *
 * ⚠️ 一发一收 (手册 6.2.3)
 * ---------------------------------------------------------------------------
 *   UART 不像 CAN 那样支持总线仲裁, **必须严格串行**: 发一包 -> 等电机回反馈
 *   包或超时 -> 才能发下一包。严禁收发异步。
 *   所以 gimbal_send_cmd() 在等待期间会返回 false, 上层要能接受"这一拍没发出去"。
 *
 * 角度编码
 * ---------------------------------------------------------------------------
 *   0x05 角度控制 : uint16, 0 ~ 2π rad, 绝对值, 走最近方向
 *   0x07 角度步进 : int16, -2π ~ 2π rad, 增量值   <-- 本工程用这个
 *   raw = θ / (2π) × 32767
 */

#ifndef GIMBAL_H_
#define GIMBAL_H_

#include <stdbool.h>
#include <stdint.h>

/* 电机的 UART ID。和之前配的 can.id 是同一个值 (000)。
 * ⚠️ 这个 ID 是电机侧配的, 不是我们随便定的。 */
#define GIMBAL_ID       0u

/* --- 指令类型 (控制包 Byte 1) --- */
#define GIMBAL_CMD_NOP          0x00u   /* 只为索取反馈包 */
#define GIMBAL_CMD_ENABLE       0x01u
#define GIMBAL_CMD_DISABLE      0x02u
#define GIMBAL_CMD_CURRENT      0x03u   /* int16, -10 ~ 10 A */
#define GIMBAL_CMD_SPEED        0x04u   /* int16, -1000 ~ 1000 rpm */
#define GIMBAL_CMD_ANGLE        0x05u   /* uint16, 0 ~ 2π rad (绝对) */
#define GIMBAL_CMD_LOW_SPEED    0x06u
#define GIMBAL_CMD_ANGLE_STEP   0x07u   /* int16, -2π ~ 2π rad (增量) */
#define GIMBAL_CMD_CLEAR_ERROR  0xFBu   /* 需固件 > 6.2.2, 本机 6.4.0 OK */
#define GIMBAL_CMD_SET_ZERO     0xFEu
#define GIMBAL_CMD_REBOOT       0xFFu

/* --- 工作模式 (反馈包 Byte 1 的 bit7-4) --- */
#define GIMBAL_MODE_CURRENT     0x00u
#define GIMBAL_MODE_SPEED       0x01u
#define GIMBAL_MODE_ANGLE       0x02u
#define GIMBAL_MODE_ANGLE_STEP  0x03u
#define GIMBAL_MODE_LOW_SPEED   0x04u

/* --- 错误码 (反馈包 Byte 2) ---
 * ⚠️ 手册的字节表里这一字节写的是"预留", 但紧接着又给出了错误码的位定义。
 *    两边对不上, 这里按"它就是错误码"处理。 */
#define GIMBAL_ERR_CALIBRATION  0x01u
#define GIMBAL_ERR_VOLTAGE      0x02u
#define GIMBAL_ERR_TIMEOUT      0x04u
#define GIMBAL_ERR_TEMPERATURE  0x08u

typedef struct {
    uint8_t  mode;          /* GIMBAL_MODE_* */
    bool     enabled;
    uint8_t  error;         /* GIMBAL_ERR_* 位掩码, 0 = 无错误 */
    int16_t  current_raw;   /* ±32767 对应 ±10 A */
    int16_t  speed_raw;     /* ±32767 对应 ±1000 rpm */
    uint16_t angle_raw;     /* 0~65535 对应 0~2π rad */
} gimbal_feedback_t;

/* 2π rad 对应的原始码值 */
#define GIMBAL_RAW_PER_TURN     32767.0f

/* 换算工具 (仅供显示/调试) */
static inline float gimbal_angle_rad(uint16_t raw)
{
    return (float) raw / 65535.0f * 6.28318530718f;
}
static inline float gimbal_speed_rpm(int16_t raw)
{
    return (float) raw / GIMBAL_RAW_PER_TURN * 1000.0f;
}
static inline float gimbal_current_a(int16_t raw)
{
    return (float) raw / GIMBAL_RAW_PER_TURN * 10.0f;
}

/* 初始化: 使能 UART1 的 NVIC 中断。必须在 SYSCFG_DL_init() 之后调用。
 * ⚠️ 漏调用的表现是"能发不能收", 而且不会有任何报错。 */
void gimbal_init(void);

/* 超时轮询。必须在主循环里**每 1 ms 调用一次**。
 * 负责在电机没回反馈时把"等待中"状态解除, 否则一发一收会永久卡死。 */
void gimbal_poll(void);

/* 发一条控制包。
 * 返回 false 有两种可能: 上一条的反馈还没回来(正常, 上层跳过这一拍即可),
 * 或者 UART 发不出去(异常)。 */
bool gimbal_send_cmd(uint8_t cmd, int16_t value);

/* 角度步进 (指令 0x07)。dtheta 单位 rad, 内部按 -2π~2π 夹紧。
 * 这是视觉外环 -> 云台的唯一入口。 */
bool gimbal_step_rad(float dtheta);

bool gimbal_enable(void);
bool gimbal_disable(void);
bool gimbal_set_zero(void);
bool gimbal_clear_error(void);

/* 取最近一帧反馈。有新帧返回 true 并填充 *out。任务上下文调用。 */
bool gimbal_get_feedback(gimbal_feedback_t *out);

/* 调试计数 —— 全为 0 才健康 */
extern volatile uint32_t g_gimbal_rx_count;   /* 收到的好反馈包数 */
extern volatile uint32_t g_gimbal_tx_fail;    /* 因"还在等反馈"而没发出去的次数 */
extern volatile uint32_t g_gimbal_timeout;    /* 反馈超时次数 */
extern volatile uint32_t g_gimbal_rx_err;     /* CRC错/ID不符/溢出 的帧数 */

#endif /* GIMBAL_H_ */
