/* gyro_link.c —— 见 gyro_link.h */

#include "ti_msp_dl_config.h"
#include "gyro_link.h"

#define GYRO_MAX_DATA       18u                     /* 模式 0 的 DATA 最长 */
#define GYRO_FRAME_MAX      (6u + GYRO_MAX_DATA)    /* 含帧头与校验共 24 字节 */

#define GYRO_CMD_REPORT     0x01u                   /* 模块上报数据 */
#define GYRO_CMD_REPORT_CTRL 0x0Au                  /* 启动/停止上报 */
#define GYRO_CMD_SET_MODE   0x0Bu                   /* 0=全数据 1=仅姿态 */

#define GYRO_TX_DELAY_CYCLES 3200u                  /* 115200 无校验, 见 SDK UART.md */

static volatile gyro_msg_t s_msg;
static volatile bool       s_ready;

volatile uint32_t g_gyro_rx_count  = 0u;
volatile uint32_t g_gyro_bad_csum  = 0u;
volatile uint32_t g_gyro_bad_frame = 0u;

/* ==========================================================================
 * 切帧状态机
 *
 * 帧长是变长的 (LEN 字段决定), 所以必须逐段推进而不是定长收满。
 * s_buf 布局: [0]=0xAA [1]=0x55 [2]=DevID [3]=CMD [4]=LEN
 *             [5..4+LEN]=DATA  [5+LEN]=CS
 * ========================================================================== */
typedef enum {
    GS_HDR_0 = 0,
    GS_HDR_1,
    GS_DEVID,
    GS_CMD,
    GS_LEN,
    GS_DATA,
    GS_CS
} gyro_state_t;

static gyro_state_t s_st  = GS_HDR_0;
static uint8_t      s_buf[GYRO_FRAME_MAX];
static uint8_t      s_pos = 0;      /* 已存入 s_buf 的字节数 */
static uint8_t      s_len = 0;      /* 本帧 DATA 长度 */

static void gyro_parse(void)
{
    uint8_t cmd = s_buf[3];
    uint8_t len = s_buf[4];
    uint8_t sum = 0u;
    uint8_t i;

    /* 校验: 从 DevID(下标 2) 累加到 DATA 末尾, **不含帧头也不含校验本身** */
    for (i = 2u; i < (5u + len); i++) {
        sum += s_buf[i];
    }
    if (sum != s_buf[5u + len]) {
        g_gyro_bad_csum++;
        return;
    }

    if (cmd != GYRO_CMD_REPORT) {
        return;                     /* ACK 等其它帧本驱动不处理 */
    }

    if (len == GYRO_MAX_DATA) {
        /* 模式 0: AccX,AccY,AccZ, GyroX,GyroY,GyroZ, Pitch,Roll,Yaw */
        for (i = 0u; i < 3u; i++) {
            s_msg.acc_raw[i] = (int16_t)((uint16_t) s_buf[5u + 2u * i] |
                                         ((uint16_t) s_buf[6u + 2u * i] << 8));
            s_msg.gyro_raw[i] = (int16_t)((uint16_t) s_buf[11u + 2u * i] |
                                          ((uint16_t) s_buf[12u + 2u * i] << 8));
        }
        s_msg.pitch_deg = (float) (int16_t)((uint16_t) s_buf[17] |
                                            ((uint16_t) s_buf[18] << 8)) / 100.0f;
        s_msg.roll_deg  = (float) (int16_t)((uint16_t) s_buf[19] |
                                            ((uint16_t) s_buf[20] << 8)) / 100.0f;
        s_msg.yaw_deg   = (float) (uint16_t)((uint16_t) s_buf[21] |
                                             ((uint16_t) s_buf[22] << 8)) / 100.0f;
        s_ready = true;
        g_gyro_rx_count++;
    } else if (len == 6u) {
        /* 模式 1: Yaw,Pitch,Roll —— ⚠️ 顺序与模式 0 不同, Yaw 在第一位 */
        s_msg.yaw_deg   = (float) (uint16_t)((uint16_t) s_buf[5] |
                                             ((uint16_t) s_buf[6] << 8)) / 100.0f;
        s_msg.pitch_deg = (float) (int16_t)((uint16_t) s_buf[7] |
                                            ((uint16_t) s_buf[8] << 8)) / 100.0f;
        s_msg.roll_deg  = (float) (int16_t)((uint16_t) s_buf[9] |
                                            ((uint16_t) s_buf[10] << 8)) / 100.0f;
        s_ready = true;
        g_gyro_rx_count++;
    } else {
        g_gyro_bad_frame++;         /* 本驱动只认这两个长度 */
    }
}

static void gyro_feed(uint8_t byte)
{
    switch (s_st) {
        case GS_HDR_0:
            if (byte == GYRO_HDR_0) {
                s_buf[0] = byte;
                s_pos    = 1u;
                s_st     = GS_HDR_1;
            }
            break;

        case GS_HDR_1:
            if (byte == GYRO_HDR_1) {
                s_buf[1] = byte;
                s_pos    = 2u;
                s_st     = GS_DEVID;
            } else if (byte == GYRO_HDR_0) {
                s_buf[0] = byte;        /* AA AA ... 第二个才是新帧头 */
                s_pos    = 1u;
            } else {
                s_st  = GS_HDR_0;
                s_pos = 0u;
            }
            break;

        case GS_DEVID:
            if (byte == GYRO_DEV_ID) {
                s_buf[s_pos++] = byte;
                s_st           = GS_CMD;
            } else if (byte == GYRO_HDR_0) {
                s_buf[0] = byte;        /* 没对上设备号, 但可能是个新帧头 */
                s_pos    = 1u;
                s_st     = GS_HDR_1;
            } else {
                s_st  = GS_HDR_0;
                s_pos = 0u;
            }
            break;

        case GS_CMD:
            s_buf[s_pos++] = byte;
            s_st           = GS_LEN;
            break;

        case GS_LEN:
            if (byte > GYRO_MAX_DATA) {     /* 长度非法, 整帧丢弃重新找帧头 */
                g_gyro_bad_frame++;
                s_st  = GS_HDR_0;
                s_pos = 0u;
                break;
            }
            s_len          = byte;
            s_buf[s_pos++] = byte;
            s_st           = (s_len == 0u) ? GS_CS : GS_DATA;
            break;

        case GS_DATA:
            s_buf[s_pos++] = byte;
            if (s_pos >= (5u + s_len)) {    /* DATA 收满 (下标 5..4+len) */
                s_st = GS_CS;
            }
            break;

        case GS_CS:
            s_buf[s_pos] = byte;            /* 校验和落在下标 5+len */
            gyro_parse();
            s_st  = GS_HDR_0;
            s_pos = 0u;
            break;

        default:
            s_st  = GS_HDR_0;
            s_pos = 0u;
            break;
    }
}

/* ==========================================================================
 * 发送
 * ========================================================================== */

bool gyro_link_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t buf[GYRO_FRAME_MAX];
    uint8_t sum = 0u;
    uint8_t i;
    uint8_t n;

    if (len > GYRO_MAX_DATA) {
        return false;
    }

    buf[0] = GYRO_HDR_0;
    buf[1] = GYRO_HDR_1;
    buf[2] = GYRO_DEV_ID;
    buf[3] = cmd;
    buf[4] = len;
    for (i = 0u; i < len; i++) {
        buf[5u + i] = data[i];
    }

    for (i = 2u; i < (5u + len); i++) {     /* DevID..DATA, 不含帧头 */
        sum += buf[i];
    }
    buf[5u + len] = sum;

    n = 6u + len;
    for (i = 0u; i < n; i++) {
        /* 用非阻塞发送 + 间隔延时, 不用 transmitDataBlocking —— 后者在
         * UART 时钟异常时会死等 (见 SDK peripherals/UART.md)。 */
        DL_UART_Main_transmitData(UART_2_INST, buf[i]);
        delay_cycles(GYRO_TX_DELAY_CYCLES);
    }

    return true;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

void gyro_link_init(void)
{
    const uint8_t mode_full = 0x00u;    /* 0 = 全数据模式 (含原始陀螺) */
    const uint8_t rep_on    = 0x01u;    /* 1 = 启动上报 */

    /* SysConfig 只生成外设级中断使能, NVIC 这一层要自己开 */
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    /* 必须先切全数据模式: 速率环要的是 GyroZ 原始角速度, 仅姿态模式给不了 */
    gyro_link_send_cmd(GYRO_CMD_SET_MODE, &mode_full, 1u);
    gyro_link_send_cmd(GYRO_CMD_REPORT_CTRL, &rep_on, 1u);
}

bool gyro_link_get(gyro_msg_t *out)
{
    uint32_t primask;
    bool     ok = false;

    if (out == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_ready) {
        *out     = *(const gyro_msg_t *) &s_msg;
        s_ready  = false;
        ok       = true;
    }
    __set_PRIMASK(primask);

    return ok;
}

void UART_2_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_IIDX_RX:
            /* FIFO 已使能, 必须一次排空 */
            while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) {
                gyro_feed(DL_UART_Main_receiveData(UART_2_INST));
            }
            break;

        case DL_UART_IIDX_OVERRUN_ERROR:
            g_gyro_bad_frame++;
            break;

        default:
            break;
    }
}
