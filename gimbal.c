/* gimbal.c —— QD4310 UART 驱动, 见 gimbal.h */

#include "ti_msp_dl_config.h"
#include "gimbal.h"

/* 控制包: ID, 指令, 控制量LO, 控制量HI, CRC8 */
#define GIMBAL_TX_LEN        5u
/* 反馈包: ID, 状态, 错误码, 电流(2), 转速(2), 角度(2), CRC8 */
#define GIMBAL_RX_LEN        10u

#define GIMBAL_CRC_POLY      0x07u


/* 等反馈的超时, 单位是 gimbal_poll() 的调用次数 (主循环 1 ms 一次)。
 *  115200 下一来一回约 1.4 ms
 *
 * ⚠️ 设成**和发送周期一样长** (见 empty.c 的 SEND_PERIOD_TICKS), 让等待窗口
 *    覆盖整个周期。这样几乎永远处于"等待中", 任何到达的字节都会被攒起来。
 *
 * 原来用较短的值 (4 ms / 10 ms), 本意是"等不到就放行, 好发下一条"。但那会
 * 留出一段 IDLE 时间, 落在里面的字节全被丢弃 —— 如果电机的回复慢到那一段,
 * 帧就永远攒不齐 (表现为 g_gimbal_rx_bytes 在涨、而 M 和 ME 都是 0)。
 *
 * ⚠️ 不能大于发送周期, 否则发不出下一条。改 SEND_PERIOD_TICKS 时这里要跟着改。 */
#define GIMBAL_RX_TIMEOUT_TICKS  5u //和发送周期一致 (empty.c 的 SEND_PERIOD_TICKS)

/* TX FIFO 满时的自旋上限, 防止 UART 时钟异常把主循环卡死 */
#define GIMBAL_TX_GUARD      200000u

typedef enum {
    GS_IDLE = 0,        /* 可以发 */
    GS_WAITING          /* 已发出, 等反馈包 */
} gimbal_state_t;

/* 以下三个由 ISR 和主循环共享, 必须 volatile */
static volatile gimbal_state_t s_state = GS_IDLE;
static volatile uint8_t        s_rxlen = 0u;
static volatile uint16_t       s_wait_ticks = 0u;

/* 只有 ISR 会碰 */
static uint8_t s_rxbuf[GIMBAL_RX_LEN];//中断接收缓冲区, 由 ISR 写, gimbal_parse() 读

/* ISR 写, 任务上下文经 gimbal_get_feedback() 读 */
static volatile gimbal_feedback_t s_fb;//电机反馈信息的结构体
static volatile bool              s_fb_ready;//主循环看到置true就拷贝 s_fb, 并置 false

volatile uint32_t g_gimbal_rx_count = 0u;
volatile uint32_t g_gimbal_tx_fail  = 0u;
volatile uint32_t g_gimbal_timeout  = 0u;
volatile uint32_t g_gimbal_rx_err   = 0u;
volatile uint32_t g_gimbal_rx_bytes = 0u;

volatile uint8_t  g_gimbal_snoop[GIMBAL_SNOOP_LEN];//环形缓冲区
volatile uint8_t  g_gimbal_snoop_pos = 0u;

/* 每收到一个字节 (无论用不用得上) 都存进环里 */
static void gimbal_snoop_put(uint8_t byte)//环形缓冲区写入函数
{
    g_gimbal_snoop[g_gimbal_snoop_pos] = byte;
    g_gimbal_snoop_pos = (uint8_t)((g_gimbal_snoop_pos + 1u) % GIMBAL_SNOOP_LEN);
}//当g_gimbal_snoop_pos达到GIMBAL_SNOOP_LEN时,会回到0,实现环形缓冲区的循环写入

/* ==========================================================================
 * CRC8
 *
 * 手册: 多项式 0x07, 初值 0x00, 结果异或 0x00, 输入输出均不反转。
 * 即标准的 MSB-first CRC-8/ATM, 无需查表。
 * ========================================================================== */
static uint8_t gimbal_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00u;
    uint8_t i;
    uint8_t bit;

    for (i = 0u; i < len; i++) {
        crc ^= data[i];
        for (bit = 0u; bit < 8u; bit++) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((uint8_t)(crc << 1) ^ GIMBAL_CRC_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;//返回计算得到的CRC8校验码
}

/* ==========================================================================
 * 接收
 * ========================================================================== */

static void gimbal_parse(void)
{
    uint8_t want = gimbal_crc8(s_rxbuf, GIMBAL_RX_LEN - 1u);

    /* 不管这包对不对, 这一轮交换都结束了 —— 必须放行下一条, 否则一发一收
     * 会永久卡死在等待状态。 */
    s_state = GS_IDLE;

    if ((s_rxbuf[0] != GIMBAL_ID) || (want != s_rxbuf[GIMBAL_RX_LEN - 1u])) {
        g_gimbal_rx_err++;
        return;
    }

    /* 反馈包的第 1~8 字节与 CAN 版完全一致 (手册 6.2.2.2 "中间数据格式均保持
     * 一致"), 只是前面多了 ID、后面多了 CRC8。 */
    s_fb.mode    = (uint8_t) ((s_rxbuf[1] >> 4) & 0x0Fu);
    s_fb.enabled = (s_rxbuf[1] & 0x01u) != 0u;
    s_fb.error   = s_rxbuf[2];

    s_fb.current_raw = (int16_t) ((uint16_t) s_rxbuf[3] |
                                  ((uint16_t) s_rxbuf[4] << 8));
    s_fb.speed_raw   = (int16_t) ((uint16_t) s_rxbuf[5] |
                                  ((uint16_t) s_rxbuf[6] << 8));
    s_fb.angle_raw   = (uint16_t) ((uint16_t) s_rxbuf[7] |
                                   ((uint16_t) s_rxbuf[8] << 8));

    s_fb_ready = true;
    g_gimbal_rx_count++;
}

/* ==========================================================================
 * 发送
 * ========================================================================== */

bool gimbal_send_cmd(uint8_t cmd, int16_t value)//cmd命令模式，后面为要发的值
{
    uint8_t buf[GIMBAL_TX_LEN];
    uint8_t i;

    /* 一发一收 (手册 6.2.3): 上一条的反馈还没到、也没超时之前, 不许再发。 */
    if (s_state != GS_IDLE) {
        g_gimbal_tx_fail++;
        return false;
    }

    buf[0] = GIMBAL_ID;
    buf[1] = cmd;
    buf[2] = (uint8_t) ((uint16_t) value & 0xFFu);//小端模式，低字节放低地址
    buf[3] = (uint8_t) (((uint16_t) value >> 8) & 0xFFu);
    buf[4] = gimbal_crc8(buf, GIMBAL_TX_LEN - 1u);

    /* 丢掉可能残留的接收字节。一发一收下 IDLE 期间本不该有数据进来, 但万
     * 一上一轮超时后电机又慢吞吞回了半包, 那些字节会混进这次的反馈帧里,
     * 让 CRC 一直对不上。 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        gimbal_snoop_put((uint8_t) DL_UART_Main_receiveData(UART_1_INST));
        g_gimbal_rx_bytes++;    /* 这里也必须计数, 否则 RB 是低估的 */
    }

    /* 顺序要紧: 先清长度、再切状态。反过来的话, 状态切到 WAITING 之后、
     * 长度还没清零时来一个中断, 就会把这半包数据当成新包的开头。 */
    s_rxlen      = 0u;
    s_wait_ticks = 0u;
    s_state      = GS_WAITING;

    for (i = 0u; i < GIMBAL_TX_LEN; i++) {
        uint32_t guard = 0u;

        /* FIFO 已使能 (4 级), 前 4 个字节直接进, 第 5 个等约 87 µs。
         * 所以这里最多阻塞不到 100 µs, 而不是不做 FIFO 时的 5 × 87 µs。 */
        while (DL_UART_Main_isTXFIFOFull(UART_1_INST)) {
            if (++guard > GIMBAL_TX_GUARD) {
                /* UART 时钟异常 —— 放弃这一包并放行, 别把主循环卡死 */
                s_state = GS_IDLE;
                g_gimbal_tx_fail++;
                return false;
            }
        }
        DL_UART_Main_transmitData(UART_1_INST, buf[i]);
    }

    return true;
}

bool gimbal_set_speed(float rpm)
{
    if (rpm > GIMBAL_RPM_MAX) {
        rpm = GIMBAL_RPM_MAX;
    } else if (rpm < -GIMBAL_RPM_MAX) {
        rpm = -GIMBAL_RPM_MAX;
    }

    /* raw = rpm/1000 × 32767。和 0x07 的 θ/2π×32767 不可互换 */
    return gimbal_send_cmd(GIMBAL_CMD_SPEED,
                           (int16_t) (rpm / GIMBAL_RPM_MAX * GIMBAL_RAW_PER_TURN));
}

bool gimbal_set_angle(float rad)
{
    /* 归一到 [0, 2π) */
    while (rad < 0.0f) {
        rad += 6.28318530718f;
    }
    while (rad >= 6.28318530718f) {
        rad -= 6.28318530718f;
    }

    /* ⚠️ 0x05 是【uint16】, 满量程 65535 —— 和 0x07 的 int16/32767 不一样!
     * (手册 5.3.2.1: 0.5 rad -> 0.5/(2π)×65535 = 5218)
     * gimbal_send_cmd 收 int16_t, 但字节是原样拆的, 高位不会丢。 */
    return gimbal_send_cmd(GIMBAL_CMD_ANGLE,
                           (int16_t) (uint16_t) (rad / 6.28318530718f *
                                                 GIMBAL_ANGLE_FULL));
}

bool gimbal_enable(void)
{
    return gimbal_send_cmd(GIMBAL_CMD_ENABLE, 0);
}

bool gimbal_disable(void)
{
    return gimbal_send_cmd(GIMBAL_CMD_DISABLE, 0);
}

/* ⚠️ 重设零点(0xFE): 覆盖上位机标定的零点。本工程不用, 上电回零走 0x05。 */
bool gimbal_set_zero(void)//设置零点
{
    return gimbal_send_cmd(GIMBAL_CMD_SET_ZERO, 0);
}

bool gimbal_clear_error(void)//清除错误码
{
    return gimbal_send_cmd(GIMBAL_CMD_CLEAR_ERROR, 0);
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

void gimbal_init(void)
{
    /* SysConfig 只生成外设级的中断使能, NVIC 这一层要自己开 */
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

void gimbal_poll(void)//等待超时计数器
{
    if (s_state == GS_WAITING) {
        if (++s_wait_ticks >= GIMBAL_RX_TIMEOUT_TICKS) {
            s_state      = GS_IDLE;
            s_wait_ticks = 0u;
            g_gimbal_timeout++;
        }
    }
}

bool gimbal_get_feedback(gimbal_feedback_t *out)
{
    uint32_t primask;//保存中断状态
    bool     ok = false;

    if (out == NULL) {
        return false;
    }

    /* 关中断拷贝, 防止 6 个字段被 ISR 在中间改掉 */
    primask = __get_PRIMASK();//获取当前中断状态
    __disable_irq();//关闭中断
    if (s_fb_ready) {
        *out       = *(const gimbal_feedback_t *) &s_fb;
        s_fb_ready = false;
        ok         = true;
    }
    __set_PRIMASK(primask);//恢复中断状态

    return ok;
}

void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
                uint8_t byte = DL_UART_Main_receiveData(UART_1_INST);

                /* 先记录再判断 —— 不管这字节用不用得上, 它都证明"线上有数据"。
                 * 这是排查"电机到底发没发"最直接的证据。 */
                g_gimbal_rx_bytes++;
                gimbal_snoop_put(byte);

                /* 一发一收: 没在等反馈时收到的字节一概丢弃。
                 * 不做长度判断的话, 一旦帧对不齐就会一直错下去。 */
                if (s_state != GS_WAITING) {
                    continue;
                }
                if (s_rxlen < GIMBAL_RX_LEN) {
                    s_rxbuf[s_rxlen++] = byte;
                    if (s_rxlen >= GIMBAL_RX_LEN) {
                        gimbal_parse();
                    }
                }
            }
            break;

        case DL_UART_IIDX_OVERRUN_ERROR:
            g_gimbal_rx_err++;
            break;

        default:
            break;
    }
}
