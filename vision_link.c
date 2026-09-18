/* vision_link.c —— 见 vision_link.h */

#include "ti_msp_dl_config.h"
#include "vision_link.h"

#define VISION_FRAME_LEN   6u
#define VISION_HEADER_0    0xAAu
#define VISION_HEADER_1    0x55u

/* ISR 写，任务上下文经 vision_link_get() 读 */
static volatile vision_msg_t s_rx_msg; 
static volatile bool         s_rx_ready;

volatile uint32_t g_vision_overrun  = 0;
volatile uint32_t g_vision_bad_csum = 0;

/* --- 切帧状态机 ----------------------------------------------------------
 * 6 字节定长 + AA 55 帧头 + 累加和，三者互相印证。
 *
 * 校验失败时**不整帧丢弃，而是把后 5 字节左移重新过一遍解析器**。
 * 原因：载荷里恰好出现 AA 55 会造成误同步 —— ERR_X 的低字节完全可能是
 * 0xAA，紧随其后是 0x55 就凑出了假帧头。左移重扫能在下一帧真正到来之前
 * 就把状态机纠正回来；整帧清空则要白等一帧。
 * 重扫最多 5 字节、凑不满 6 字节，所以递归深度只有 1 层。
 * ---------------------------------------------------------------------- */
typedef enum {
    VS_HDR_0 = 0,   /* 等第一个帧头字节 */
    VS_HDR_1,       /* 已收到 AA，等 55 */
    VS_PAYLOAD      /* 帧头已齐，收剩余字节 */
} vision_parse_state_t;

static vision_parse_state_t s_state = VS_HDR_0;
static uint8_t s_buf[VISION_FRAME_LEN];
static uint8_t s_pos = 0;

static void vision_feed(uint8_t byte)
{
    switch (s_state) {
        case VS_HDR_0:
            if (byte == VISION_HEADER_0) {
                s_buf[0] = byte;
                s_pos    = 1;
                s_state  = VS_HDR_1;
            }
            break;

        case VS_HDR_1:
            if (byte == VISION_HEADER_1) {
                s_buf[1] = byte;
                s_pos    = 2;
                s_state  = VS_PAYLOAD;
            } else if (byte == VISION_HEADER_0) {
                /* AA AA ... —— 第二个 AA 才是新帧头的起点 */
                s_buf[0] = byte;
                s_pos    = 1;
            } else {
                s_state = VS_HDR_0;
                s_pos   = 0;
            }
            break;

        case VS_PAYLOAD: {
            uint8_t sum;

            s_buf[s_pos++] = byte;
            if (s_pos < VISION_FRAME_LEN) {
                break;      /* 还没收满 */
            }

            sum = 0;
            for (uint8_t i = 0; i < VISION_FRAME_LEN - 1u; i++) {
                sum += s_buf[i];
            }

            if (sum == s_buf[VISION_FRAME_LEN - 1u]) {
                s_rx_msg.status = s_buf[2];
                s_rx_msg.err_x  = (int16_t)((uint16_t) s_buf[3] |
                                            ((uint16_t) s_buf[4] << 8));
                s_rx_ready      = true;

                s_state = VS_HDR_0;
                s_pos   = 0;
            } else {
                uint8_t tail[VISION_FRAME_LEN - 1u];

                g_vision_bad_csum++;

                for (uint8_t i = 1; i < VISION_FRAME_LEN; i++) {
                    tail[i - 1u] = s_buf[i];
                }
                s_state = VS_HDR_0;
                s_pos   = 0;
                for (uint8_t i = 0; i < VISION_FRAME_LEN - 1u; i++) {
                    vision_feed(tail[i]);
                }
                return;     /* 状态已由重扫定好，不要再往下走 */
            }
            break;
        }

        default:
            s_state = VS_HDR_0;
            s_pos   = 0;
            break;
    }
}

bool vision_link_get(vision_msg_t *out)
{
    uint32_t primask;
    bool     ok = false;

    if (out == NULL) {
        return false;
    }

    /* 关中断拷贝，防止 err_x 的两个字节被 ISR 在中间改掉（读撕裂） */
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_rx_ready) {
        out->status = s_rx_msg.status;
        out->err_x  = s_rx_msg.err_x;
        s_rx_ready  = false;
        ok          = true;
    }
    __set_PRIMASK(primask);

    return ok;
}

void vision_link_init(void)
{
    /* SysConfig 只生成**外设级**的中断使能 (DL_UART_Main_enableInterrupt),
     * NVIC 这一层没有生成, 必须自己开。
     *
     * ⚠️ 漏掉这一句的表现是【一个字节都收不到】—— 数据到了 UART3 的 FIFO,
     *    但中断永远不触发, 解析器一次都不会被调用。而且编译期、运行期
     *    **都不会报任何错**。本工程的 MCAN 和 UART2 都踩过这个坑。 */
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
}

void UART_3_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_3_INST)) {
        case DL_UART_IIDX_RX:
            /* FIFO 已使能，必须一次排空。只读一个字节的话，剩下的几级会很快
             * 被后续字节填满并溢出 —— 那比不开 FIFO 丢得更多。 */
            while (!DL_UART_Main_isRXFIFOEmpty(UART_3_INST)) {
                vision_feed(DL_UART_Main_receiveData(UART_3_INST));
            }
            break;

        case DL_UART_IIDX_OVERRUN_ERROR:
            g_vision_overrun++;
            break;

        default:
            break;
    }
}
