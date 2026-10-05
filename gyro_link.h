/*
 * gyro_link —— 汇电籽-601 姿态模块驱动 (UART2, PB17/PB18, 115200 8N1)
 *
 * 用途: 提供**车体偏航角 ψ** 及其变化率 ψ̇ —— 陀螺装在【车体】上。
 *       云台没有自稳环(旧版的速率环/航向环已废弃): ψ 用来反推车在弧上的位置,
 *       算出靶方位角变化率作为几何前馈, 直接叠进 0x04 速度指令。
 *       详见 docs/PROJECT.md 和 guidance.h。
 *
 * 帧格式 (见模块通信协议说明书)
 * ---------------------------------------------------------------------------
 *   AA 55 | DevID | CMD | LEN | DATA... | CS
 *     DevID 默认 0x60
 *     CS    = (DevID + CMD + LEN + DATA 各字节) & 0xFF   <-- 从 DevID 起算,
 *             **不含 0xAA 0x55 两个帧头字节**
 *
 * 上报帧 (CMD = 0x01), 两种模式的数据顺序**不一样**:
 *   模式 0 全数据 (LEN=18):
 *       AccX,AccY,AccZ, GyroX,GyroY,GyroZ, Pitch,Roll,Yaw
 *   模式 1 仅姿态 (LEN=6):
 *       Yaw,Pitch,Roll                    <-- Yaw 在第一位!
 *
 *   ⚠️ 必须用**模式 0** —— 只有它带 GyroZ 原始角速度。模式 1 只有姿态角,
 *      要靠差分求角速度, 噪声大且滞后。
 *
 * 姿态角编码: 均为放大 100 倍的整数
 *   Pitch / Roll : int16, -180.00 ~ +180.00
 *   Yaw          : uint16,    0.00 ~  360.00     <-- 无符号, 与上面两个不同
 */

#ifndef GYRO_LINK_H_
#define GYRO_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#define GYRO_DEV_ID     0x60u
#define GYRO_HDR_0      0xAAu
#define GYRO_HDR_1      0x55u

/* 上报数据里的原始陀螺标度, 单位 LSB/(°/s)。
 *
 * ⚠️ 协议说明书**没有给出这个值**, 必须实测标定:
 *     把模块绕竖直轴匀速转一个已知角度 (例: 2 秒转 90° -> 45 °/s),
 *     读这段时间 GyroZ 原始值的平均值, 标度 = 平均值 / 45。
 *     过程中要匀速, 且首尾各丢掉一小段 (加减速阶段)。
 */
#define GYRO_LSB_PER_DPS    16.4f

typedef struct {
    int16_t acc_raw[3];     /* [0]=X [1]=Y [2]=Z */
    int16_t gyro_raw[3];    /* [0]=X [1]=Y [2]=Z, 原始值, 见 GYRO_LSB_PER_DPS */
    float   yaw_deg;        /*   0.00 ~ 360.00 */
    float   pitch_deg;      /* -180.00 ~ +180.00 */
    float   roll_deg;       /* -180.00 ~ +180.00 */
} gyro_msg_t;

/* 把原始陀螺值换成 °/s */
static inline float gyro_raw_to_dps(int16_t raw)
{
    return (float) raw / GYRO_LSB_PER_DPS;
}

/* 初始化: 使能 UART2 的 NVIC 中断, 并把模块切到【全数据模式】+ 启动上报。
 * 调用时机: SYSCFG_DL_init() 之后。 */
void gyro_link_init(void);

/* 每 1 ms 调一次。陀螺开机要 ~5 s 才就绪, 初始化命令会丢 —— 没收到全数据帧
 * 之前定期重发。不调的话模式切换失败, 帧里没角速度, 自稳完全失效。 */
void gyro_link_poll(void);

/* 取最近一帧上报数据。有新帧返回 true 并填充 *out。任务上下文调用。 */
bool gyro_link_get(gyro_msg_t *out);

/* 主动下发一条指令 (DevID 固定为 GYRO_DEV_ID)。data 可为 NULL 当 len=0。
 * 返回 false 表示参数非法。 */
bool gyro_link_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len);

/* 调试计数 */
extern volatile uint32_t g_gyro_rx_count;    /* 收到的上报帧数 */
extern volatile uint32_t g_gyro_bad_csum;    /* 校验失败帧数 */
extern volatile uint32_t g_gyro_bad_frame;   /* 帧结构非法(长度超限等)次数 */

#endif /* GYRO_LINK_H_ */
