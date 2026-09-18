/*
 * debug_uart —— 把 printf 重定向到调试串口 UART0 (PA10/PA11, 115200 8N1)
 *
 * 接线: UART0_TX (PA10) -> USB转TTL 的 RX
 *       UART0_RX (PA11) <- USB转TTL 的 TX   (本驱动不用接收, 可只接 TX/GND)
 *       GND 共地
 *
 * 非阻塞
 * ---------------------------------------------------------------------------
 * printf 只把字符塞进一个 256 字节的环形缓冲就返回 (**微秒级**), 真正的发送
 * 交给 UART0 的 TX 中断在后台慢慢搬。所以它不会再拖慢 1 ms 的主循环节拍。
 *
 * 缓冲满时**丢弃**多出来的字符 —— 调试输出丢几个字无所谓, 反正是给人看的,
 * 总比把控制环卡住强。
 *
 * 用法: main() 里 SYSCFG_DL_init() 之后调用 debug_uart_init(), 然后 printf
 *       就能用了。忘了调用也不会崩, 只是收不到输出。
 */

#ifndef DEBUG_UART_H_
#define DEBUG_UART_H_

/* 1 = printf 走 UART0;  0 = 关闭。
 * 关掉之后 printf 交给 TI 运行库的默认实现 (调试器 CIO), 不会报错。
 * 现在是非阻塞的, 所以调试完**不是必须关** —— 留着也就多占一点 CPU。 */
#define DEBUG_PRINT_ENABLE      1

/* 调试信息打印周期, 单位是主循环节拍 (1 ms)。1000 = 每秒一行。 */
#define DEBUG_PERIOD_TICKS      1000u

/* 初始化: 使能 UART0 的 NVIC 中断。必须在 SYSCFG_DL_init() 之后调用。 */
void debug_uart_init(void);

#endif /* DEBUG_UART_H_ */
