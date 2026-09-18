/* debug_uart.c —— 见 debug_uart.h */

#include "ti_msp_dl_config.h"
#include "debug_uart.h"

#if DEBUG_PRINT_ENABLE

/* 环形缓冲, 大小必须是 2 的幂 (下标靠掩码回绕) */
#define DBG_TX_BUF_SIZE     256u
#define DBG_TX_BUF_MASK     (DBG_TX_BUF_SIZE - 1u)

static volatile uint8_t  s_txbuf[DBG_TX_BUF_SIZE];
static volatile uint16_t s_head = 0u;   /* 生产者(printf)写的位置 */
static volatile uint16_t s_tail = 0u;   /* 消费者(TX 中断)读的位置 */
static volatile bool     s_tx_active = false;   /* TX 中断链是否已经跑起来 */

/* 启动发送链: 先把第一个字节塞进数据寄存器, 然后打开 TX 中断, 后面每发完
 * 一个字节中断一次, 从缓冲里取下一个。
 *
 * 关中断是为了把 "检查 s_tx_active" 和 "置位 s_tx_active" 做成原子操作 ——
 * 否则会和 ISR 里 "缓冲区空就关闭中断" 那段撞车, 结果是新的字节永远发不出去。 */
static void debug_uart_kick(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    if ((!s_tx_active) && (s_tail != s_head)) {
        s_tx_active = true;
        DL_UART_Main_transmitData(UART_0_INST, s_txbuf[s_tail]);
        s_tail = (uint16_t)((s_tail + 1u) & DBG_TX_BUF_MASK);
        DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_TX);
    }

    __set_PRIMASK(primask);
}

/* 覆盖 TI 运行库的底层输出函数, 把 stdout/stderr 接到 UART0。
 *
 * ⚠️ 覆盖的是 write(), **不是 fputc/putchar** —— TI 的 C 库调的是 write(),
 *    改 fputc 那个老套路在这里不生效 (见 SDK peripherals/UART.md)。
 *
 * 这里只做"塞进环形缓冲", 不做任何等待。 */
int write(int fd, const char *buf, unsigned count)
{
    unsigned i;

    if ((fd != 1) && (fd != 2)) {       /* 只处理 stdout / stderr */
        return -1;
    }

    for (i = 0u; i < count; i++) {
        uint16_t next = (uint16_t)((s_head + 1u) & DBG_TX_BUF_MASK);

        if (next == s_tail) {
            break;                      /* 缓冲满, 丢弃剩余字符 */
        }
        s_txbuf[s_head] = (uint8_t) buf[i];
        s_head          = next;
    }

    debug_uart_kick();

    /* 即使丢了字符也报"全部写完" —— 返回真值会让 C 库重试, 那就白等了 */
    return (int) count;
}

void debug_uart_init(void)
{
    /* SysConfig 里勾了 "Transmit", 生成代码会在初始化时就把 TX 中断打开。
     * 但那时还没东西要发, TXIFG 立刻就是置位的, 会白白进一次中断。
     * 先关掉, 等第一次 write() 需要时再开。 */
    DL_UART_Main_disableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_TX);

    /* SysConfig 只生成外设级中断使能, NVIC 这一层要自己开 */
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_IIDX_TX:
            if (s_tail != s_head) {
                DL_UART_Main_transmitData(UART_0_INST, s_txbuf[s_tail]);
                s_tail = (uint16_t)((s_tail + 1u) & DBG_TX_BUF_MASK);
            } else {
                /* 缓冲空了 -> **必须关掉 TX 中断**。TXIFG 是"数据寄存器空"的
                 * 电平时标志, 不关的话它会一直触发, 变成死循环 ISR。 */
                DL_UART_Main_disableInterrupt(UART_0_INST,
                                              DL_UART_MAIN_INTERRUPT_TX);
                s_tx_active = false;
            }
            break;

        default:
            break;
    }
}

#endif /* DEBUG_PRINT_ENABLE */
