/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * 瞄准模块主程序 —— MSPM0G3507  (2025 电赛 E 题招新改编版)
 *
 *   职责: MaixCam2 --(UART3, 6字节帧)--> MSPM0 --(CAN, 1Mbps)--> QD4310 云台
 *         汇电籽-601 --(UART2)---------^  (角速度反馈 -> 速率环)
 *   靶面是一条竖直中线, 光斑高度不敏感, 所以只需一维(偏航)对准。
 *   底盘(循迹/电机/定点停车)由另一块主控负责, 不在本程序范围内。
 *
 * 控制结构
 * ---------------------------------------------------------------------------
 *   云台内部自带角度环, 所以主控只做**外环**, 增量式:
 *
 *       Δθ = 视觉项 + 速率项
 *          = AIM_GAIN_RATE × err_x × dt  −  GYRO_RATE_K × ω_gimbal × dt
 *
 *   视觉项 (慢, 绝对基准):  dθ/dt = K·f·(θ_ref − θ), 一阶收敛不过冲。
 *       每拍收敛比例 λ = K·f·T, 特征方程 z² − z + λ = 0:
 *       λ<0.25 无振荡 / λ∈(0.25,1) 有振铃。
 *
 *   速率项 (快, 抗扰动):  陀螺在云台上, 测到的就是云台角速度 ω_gimbal。
 *       按增益 K 反馈构成速率环, 车体转动当场被按住, 不必等视觉发现偏差。
 *       没它时斜坡输入的稳态跟随误差 = ω·τ, 弯道上超过 5cm 指标。
 *
 *   ⚠️ 速率环增益有硬上限 (K < 1.0), 由陀螺数据率决定 —— 性能天花板。
 *      推导见 docs/云台自稳与速率环.md
 * ---------------------------------------------------------------------------
 */

/* ==========================================================================
 * ⚠️⚠️ 首次上电必做: 符号自检   (跳过这一步可能直接撞限位)
 * ---------------------------------------------------------------------------
 * 视觉外环的符号取决于【机械安装方向 + 电机正方向】, 在纸面上定不了, 是个
 * 掷硬币。推一遍物理:
 *
 *     云台俯视【逆时针】转  ->  相机往左看  ->  画面内容整体右移
 *     ->  center_x 变大  ->  err_x 变大
 *
 * 所以如果 QD4310 的"正转"在你的安装下就是俯视逆时针, 那么
 * "err_x > 0 就发正 Δθ" 构成【正反馈】—— 云台会一路跑到限位。
 *
 * 安全流程:
 *   1. 先把 AIM_GAIN_RATE 临时改小一个数量级 (例: 0.05 -> 0.005)
 *   2. 云台空载或手扶着, 上电
 *   3. 让靶纸出现在视野里, 盯调试串口的 E 字段, 看误差是【收敛】还是【发散】
 *   4. 发散 -> 把 MaixCam 侧 camera_display.py 里的 ERR_X_SIGN 改成 -1
 *      (改那边而不是改这里, 因为 ERR_X_SIGN 本来就是为这个准备的)
 *   5. 确认收敛后, 再把 AIM_GAIN_RATE 调回正常值
 *
 * ⚠️ 别一上来就用正常增益跑: 正反馈 + λ≈0.45, 几百毫秒就能撞限位。
 *
 * 陀螺速率环的符号同理, 见下面 GYRO_RATE_SIGN 的说明。
 * ========================================================================== */

#include "ti_msp_dl_config.h"
#include "debug_uart.h"
#include "gimbal.h"
#include "gyro_link.h"
#include "vision_link.h"

#include <stdio.h>

/* 打开后每秒多打一行 UART1 收到的原始字节 (十六进制)。
 * 用于排查"云台反馈收不到"—— 详见 docs/BUGS.md #1。
 * 平时关着 (0), 免得刷屏。 */
#define DEBUG_GIMBAL_SNOOP      0

/* ==========================================================================
 * 时间基准
 * ========================================================================== */

/* 主循环节拍。1 ms @ MCLK 32 MHz。整个控制逻辑都挂在这个节拍上。 */
#define LOOP_TICK_CYCLES        32000u
#define LOOP_TICK_SEC           0.001f

/* 发送周期。5 ms = 200 Hz。
 *
 * ⚠️ 2026-09-20 从 30 ms 缩到 5 ms, 为了消除"步进阻塞感":
 *    原来 33 Hz 发指令, 电机收到一个增量 -> 转过去 -> 停住 -> 等 28.6 ms
 *    再收下一个, 肉眼可见一顿一顿。
 *    拆细之后每秒转的总角度不变 (增益是速率形式), 但目标轨迹从 33 级台阶
 *    变成 200 级, 明显平滑。
 *
 * 代价: UART 负载 200 × 15 字节 = 3000 B/s, 115200 下约 26%。可接受。
 * 往返约 1.4 ms < 5 ms, 所以"一发一收"不会挡住这个节拍。
 *
 * 发送仍然是【事件驱动】优先 —— 收到新视觉帧立刻发; 这个周期是兜底, 负责
 * 视觉帧之间 (以及视觉断流时) 继续推进。 */
#define SEND_PERIOD_TICKS       5u
#define SEND_PERIOD_SEC         ((float) SEND_PERIOD_TICKS * LOOP_TICK_SEC)

/* 多久没收到反馈/陀螺数据就认为掉线。单位节拍。 */
#define GIMBAL_LOST_TICKS       300u   /* 300 ms */
#define GYRO_LOST_TICKS         100u   /* 100 ms —— 陀螺上报率远高于此 */

/* 云台健康检查开关 (基于反馈报文)。
 *
 * ⚠️ 现在设为 0 (关)。原因: 云台反馈收不到 (见 docs/BUGS.md #1), gim_lost 会
 *    一直涨, 于是每 300 ms 就重发一次"清错误 + 使能" —— 白白挤掉两条瞄准指令,
 *    还会反复给电机发使能。联调时会看到每 300 ms 抖一下, 容易误判成控制问题。
 *
 * 等反馈那个 bug 修好 (M 能正常涨) 之后, 改回 1 打开。 */
#define GIMBAL_HEALTH_CHECK     0

/* ==========================================================================
 * 云台初始化/恢复序列
 * ---------------------------------------------------------------------------
 * UART 是【一发一收】的: 发出一条之后, 状态会停在"等反馈", 最长 10 ms 才
 * 超时放行。所以**两条指令不能背靠背发** —— 第二条必然被 gimbal_send_cmd
 * 的"还在等反馈"挡掉, 而且不会报任何错。
 *
 * (CAN 版没这个问题: "发送缓冲忙" 60 µs 就清了, 中间隔 0.2 ms 就够。
 *  换 UART 之后那段逻辑直接失效 —— 表现就是电机的使能指令从来发不出去。)
 *
 * 所以拆成状态: 每拍只尝试推进【一步】, 发成功了才进下一步, 发不出去下拍再试。
 * ========================================================================== */
#define GIM_STEP_IDLE       0u
#define GIM_STEP_CLEAR      1u      /* 该发"清错误" */
#define GIM_STEP_ENABLE     2u      /* 该发"使能" */

/* 视觉掉线判定。MaixCam 约 35 fps (28.6 ms 一帧), 100 ms ≈ 连丢 3~4 帧。
 * ⚠️ 必须有这一条: 没有它的话 vision_link_get() 一直返回 false 时, err_valid
 *    和 err_x 会保持上一次的值 —— 云台会拿着一个**陈旧的误差**继续转下去,
 *    直到撞限位。(云台和陀螺两路本来就有掉线检测, 只有视觉漏了。) */
#define VISION_LOST_TICKS       100u

#define DEG2RAD                 0.017453292519943295f

/* ==========================================================================
 * 视觉外环
 * ========================================================================== */

/* 视觉外环增益, rad/(像素·秒):  Δθ = AIM_GAIN_RATE × err_x × dt
 *
 * ⚠️ 必须写成"速率"形式 —— 写成"每帧多少 rad"的话, 发送周期一改增益就变。
 *
 * 取值反推自  AIM_GAIN_RATE = λ / (f · T_视觉):
 *     λ      每拍消掉多少误差。当前 0.29 —— >0.25 就开始有振铃
 *     f      像素焦距 ≈ 271。**推算值**(OS04D10 按 416x260 居中裁剪), 没实测
 *     T_视觉 视觉帧周期 28.5 ms —— 是【视觉帧率】, 不是发送周期!
 *
 * ⚠️ f 错了环路增益就错。校验法: 靶纸水平平移 50mm @1m, err_x 应变化
 *    约 13.5 px。对不上就按实际比例改 f。 */
#define AIM_GAIN_RATE           0.037f

/* 死区, 像素。err_x 是量化过的像素值, 零附近有 ±1~2 px 的抖动,
 * 不设死区的话积分器会追着噪声随机游走。
 * 2 px ≈ 0.38° (按 f=300), 在 1 m 处是 6.7 mm —— 远小于 3 cm 指标, 尽管取。
 * ⚠️ 死区只作用于**视觉项**; 速率项不受它影响 —— 云台被扰动就得压住, 跟视觉
 *    误差在不在死区里没关系。 */
#define AIM_DEADBAND_PX         2.0f

/* 转角速率上限, rad/s。重新捕获目标时 err_x 可能有几百像素, 不限速会甩一下。
 * 1.67 rad/s ≈ 95°/s, 既保护机械又不影响 2 s 内收敛。
 * 注意它限的是**合成后**的总增量, 速率项也被一起限住 —— 速率项正常工作时量
 * 很小 (35°/s = 0.61 rad/s), 不会顶到这个上限。 */
#define AIM_MAX_RATE_RAD_S      1.67f

/* ==========================================================================
 * 陀螺速率环
 * ---------------------------------------------------------------------------
 * 陀螺装在云台上, 测到的是云台自身的角速度, 按增益反馈回去抵抗扰动。
 * (这项以前叫"前馈" —— 叫错了, 它其实是反馈、是闭环。)
 *
 * ⚠️ GYRO_RATE_K 必须 < 1.0: 增益 1.0 + 10 ms 延迟(陀螺 100 Hz) 会临界自激,
 *    频率约 50 Hz —— 就是那个剧烈震荡。推导见 docs/云台自稳与速率环.md
 * ========================================================================== */

/* 轴。Z 轴朝上 -> 2 */
#define GYRO_RATE_AXIS      2u

/* 环路增益, 必须 < 1.0。调大压低视觉环增益, 追靶迟钝就补 AIM_GAIN_RATE。
 *
 * ⚠️ 实测记录 (2026-09-23):
 *      K=0.3  震 / K=0  不震  → 确认是速率环
 *      但 K<1 理论上该稳, 实际 0.3 就震 → 有效环路增益比理论大 (plant 增益 ≠ 1)
 *      正在二分找稳定上限。 */
#define GYRO_RATE_K         0.15f

/* 符号。反了会"车一转云台就朝同方向猛甩"。验证: 手转车身, 云台应反向补偿。 */
#define GYRO_RATE_SIGN      (-1.0f)

/* 一阶低通, 每陀螺帧一次。1.0 = 关闭。
 * ⚠️ 先关着! α=0.8(≈26Hz) 在 50 Hz 处引入 63° 相位滞后 —— 而本环路的
 *    病根就是滞后, 开滤波等于把 GYRO_RATE_K 降下来的收益吃回去。
 *    等 K 调到稳定之后, 若噪声确实碍事, 再从这个值往 1.0 方向小步试。 */
#define GYRO_LPF_ALPHA      1.0f

/* ==========================================================================
 * 外环
 * ========================================================================== */

/* 算并发出这一步转角。
 *   err_valid / err_x_px : 视觉项输入 (err_valid=false 时视觉项按 0 处理)
 *   yaw_rate_dps         : 云台偏航角速度, °/s (【已滤波】, 见 GYRO_LPF_ALPHA)
 *   rate_valid           : 陀螺数据是否有效 (掉线时置 false, 免得用陈旧值猛补)
 * 返回 false 表示这一步没有任何分量, 没发。 */
static bool aim_step(bool err_valid, float err_x_px,
                     bool rate_valid, float yaw_rate_dps)
{
    const float max_step = AIM_MAX_RATE_RAD_S * SEND_PERIOD_SEC;
    float       dtheta   = 0.0f;

    /* --- 视觉项: 慢, 负责绝对基准 --- */
    if (err_valid &&
        ((err_x_px < -AIM_DEADBAND_PX) || (err_x_px > AIM_DEADBAND_PX))) {
        dtheta += AIM_GAIN_RATE * err_x_px * SEND_PERIOD_SEC;
    }

    /* --- 速率项: 快, 抵抗扰动 ---
     * 不受死区约束, 也不受"是否看到靶"约束 —— 丢靶时云台照样该稳住自己,
     * 这正是"自稳"的含义。 */
    /* ⚠️ 增益必须 < 1.0, 否则临界/发散 —— 推导见 GYRO_RATE_K 那一段 */
    if (rate_valid) {
        dtheta += GYRO_RATE_SIGN * GYRO_RATE_K *
                  (yaw_rate_dps * DEG2RAD) * SEND_PERIOD_SEC;
    }

    if (dtheta == 0.0f) {
        return false;               /* 两项都是 0, 没必要发 */
    }

    if (dtheta > max_step) {
        dtheta = max_step;
    } else if (dtheta < -max_step) {
        dtheta = -max_step;
    }

    return gimbal_step_rad(dtheta);
}

/* ==========================================================================
 * 主程序
 * ========================================================================== */

/* ==========================================================================
 * 激光控制
 * ---------------------------------------------------------------------------
 * PA2 接激光笔的 MOS 管栅极。GPIO 在 SysConfig 里配成 Output + 上电 Cleared +
 * 内部下拉 —— 所以在程序接管之前, 栅极不会被拉高, 激光不会意外点亮。
 *
 * 当前策略: **上电即常亮**(视觉部分一开就跟着开)。
 * 若以后要按区段开关 (例: 只在 2→3、4→1 两段亮), 改这里的调用点即可。
 * ========================================================================== */
static void laser_set(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_LASER_PORT, GPIO_LASER_LASER_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_LASER_PORT, GPIO_LASER_LASER_PIN);
    }
}

int main(void)
{
    uint16_t send_tick = 0;
    uint16_t gim_lost = 0;      /* 距上次收到云台反馈的节拍数 */
    uint16_t gyro_lost = 0;     /* 距上次收到陀螺数据的节拍数 */
    uint16_t vis_lost = 0;      /* 距上次收到视觉帧的节拍数 */
#if DEBUG_PRINT_ENABLE
    uint16_t dbg_tick = 0;
    uint32_t vcount = 0;        /* 视觉帧计数, 只为调试打印 */
    uint16_t vis_gap_max = 0;   /* 本报告周期内最大的视觉帧间隔 (ms) */
    uint8_t  di = 0;            /* 字节转储的循环下标 */
    float    rabs_sum = 0.0f;   /* 本报告周期内 |角速度| 的累加和 */
    uint16_t rabs_cnt = 0;      /* 参与累加的样本数 */
#endif

    float    err_x     = 0.0f;
    bool     err_valid = false;
    float    yaw_rate  = 0.0f;  /* °/s */
    float    yaw_rate_lpf = 0.0f;  /* 低通后的角速度, 速率环喂这个 */
    bool     need_send = false; /* 本拍收到了新视觉帧 -> 立刻发, 不等定时 */
    uint8_t  gim_step  = GIM_STEP_IDLE;  /* 云台初始化序列的推进状态 */

    SYSCFG_DL_init();
#if DEBUG_PRINT_ENABLE
    debug_uart_init();
#endif
    gimbal_init();
    gyro_link_init();
    vision_link_init();

    laser_set(true);        /* 当前策略: 常亮 */

    /* 开机触发一次"清错误 -> 使能"序列。真正的发送由主循环逐步推进,
     * 原因见下面 gim_step 的说明。 */
    gim_step = GIM_STEP_CLEAR;

    while (1) {
        vision_msg_t      vmsg;
        gimbal_feedback_t gfb;
        gyro_msg_t        gmsg;

        /* --- 云台超时轮询: 必须每 1 ms 调一次 ---
         * UART 版是【一发一收】的: 发出去后要等反馈才允许发下一条。如果电机
         * 不回, 没有这个轮询就会永久卡在等待状态, 再也发不出任何指令。 */
        gimbal_poll();

        /* --- 视觉 --- */
        if (vision_link_get(&vmsg)) {
#if DEBUG_PRINT_ENABLE
            /* 用【上一次】的 vis_lost 当帧间隔 —— 它就是"距上一帧过了多少 ms"。
             * 必须在清零之前读。取一个报告周期内的最大值, 用来抓帧率抖动。 */
            if (vis_lost > vis_gap_max) {
                vis_gap_max = vis_lost;
            }
            vcount++;
#endif
            vis_lost  = 0;
            need_send = true;       /* 有新帧 -> 这一拍立刻发出去 */
            if (vmsg.status == VISION_STATUS_TARGET_VALID) {
                err_x     = (float) vmsg.err_x;
                err_valid = true;
            } else {
                /* 丢靶。⚠️ err_x 恒为 0 是"无数据"不是"已对准", 必须用
                 * err_valid 区分开。视觉项归零后, 速率项继续维持自稳。 */
                err_valid = false;
            }
        } else if (vis_lost < 0xFFFFu) {
            vis_lost++;
        }

        /* 视觉掉线 -> 把误差判为无效, 别拿陈旧值继续驱动云台。
         * (放在这里而不是塞进 if 里, 是因为"没收到帧"这个分支也要判。) */
        if (vis_lost >= VISION_LOST_TICKS) {
            err_valid = false;
        }

        /* --- 陀螺 --- */
        if (gyro_link_get(&gmsg)) {
            gyro_lost = 0;
            yaw_rate  = gyro_raw_to_dps(gmsg.gyro_raw[GYRO_RATE_AXIS]);
            /* 一阶低通。⚠️ 每个【陀螺帧】只跑一次, 不是每个控制拍 ——
             * 陀螺 100 Hz、主循环 1 kHz, 按拍跑的话同一个值会被反复滤波,
             * 等效截止频率直接变成 10 倍。 */
            yaw_rate_lpf += GYRO_LPF_ALPHA * (yaw_rate - yaw_rate_lpf);
#if DEBUG_PRINT_ENABLE
            /* 累加 |角速度|, 报告时给出本周期平均值 —— 标定 GYRO_LSB_PER_DPS 要用。
             * 取绝对值是因为手动转的时候正负都有, 要看的是"转得多快"。 */
            rabs_sum += (yaw_rate < 0.0f) ? -yaw_rate : yaw_rate;
            rabs_cnt++;
#endif
        } else if (gyro_lost < 0xFFFFu) {
            gyro_lost++;
        }

        /* --- 推进云台初始化序列 ---
         * 每拍只推进一步: 发成功了才进下一步, 发不出去(还在等反馈)就下拍再试。
         * 主循环 1 ms 一拍、往返约 1.4 ms, 所以两条指令几拍之内就能发完。 */
        if (gim_step == GIM_STEP_CLEAR) {
            if (gimbal_clear_error()) {
                gim_step = GIM_STEP_ENABLE;
            }
        } else if (gim_step == GIM_STEP_ENABLE) {
            if (gimbal_enable()) {
                gim_step = GIM_STEP_IDLE;
            }
        }

        /* --- 云台反馈 --- */
        if (gimbal_get_feedback(&gfb)) {
            gim_lost = 0;

            if (gfb.error != 0u) {
                /* 有错误码 -> 重走"清错误 + 使能"。序列已经在跑就别打断 */
                if (gim_step == GIM_STEP_IDLE) {
                    gim_step = GIM_STEP_CLEAR;
                }
            } else if (!gfb.enabled) {
                if (gim_step == GIM_STEP_IDLE) {
                    gim_step = GIM_STEP_ENABLE;     /* 只需使能 */
                }
            }
        } else if (gim_lost < 0xFFFFu) {
            gim_lost++;
        }

        /* --- 发送: 事件驱动 + 定时兜底 ---
         *
         * 原来纯定时 (30 ms) 发, 视觉帧 28.6 ms 一帧, 两个周期很接近但不相等,
         * 于是 err_x 从"到达"到"发出去"的等待时间在 0~28.6 ms 之间飘 (平均
         * 约 14 ms), 再叠加发送本身的零阶保持 (平均 15 ms) —— 相当于给环路
         * **额外加了约 30 ms 延迟**, 把相位裕度吃掉一大半。
         *
         * 改成收到新帧就立刻发之后, 这一项延迟降到 0~1 ms (主循环 1 ms 节拍)。
         * send_tick 只在没有新帧时才有机会累加, 所以视觉正常时定时分支根本
         * 不会触发, 它纯粹是视觉断流时的保活。 */
        if (need_send || (++send_tick >= SEND_PERIOD_TICKS)) {
            send_tick = 0;
            need_send = false;

#if GIMBAL_HEALTH_CHECK
            if ((gim_step == GIM_STEP_IDLE) && (gim_lost >= GIMBAL_LOST_TICKS)) {
                /* 反馈断了一阵 —— 云台掉线或被失能。触发一次"清错误 + 使能"。
                 * 只置状态、不在这里发, 因为一发一收下两条不能背靠背发。 */
                gim_lost = 0;
                gim_step = GIM_STEP_CLEAR;
            } else
#endif
            if (gim_step != GIM_STEP_IDLE) {
                /* 初始化序列正在跑, 这一拍不占发送机会, 让给序列 */
            } else {
                /* 陀螺掉线时把速率项置无效, 别拿陈旧角速度继续猛补 */
                bool rate_ok = (gyro_lost < GYRO_LOST_TICKS);

                if (!aim_step(err_valid, err_x, rate_ok, yaw_rate_lpf)) {
                    /* 视觉在死区内 + 车没转 —— 发 NOP 保活, 反馈才不会断
                     * (指令 0x00 就是"不改变任何东西, 只为索取反馈报文")。 */
                    gimbal_send_cmd(GIMBAL_CMD_NOP, 0);
                }
            }
        }

#if DEBUG_PRINT_ENABLE
        /* 上板 bring-up 用的状态行。⚠️ 会阻塞约 3.5 ms, 正式跑之前把
         * debug_uart.h 里的 DEBUG_PRINT_ENABLE 改成 0 (详见该文件说明)。
         *
         *   V  视觉帧数    —— 不涨 = MaixCam 没发 / UART3 接线错
         *   G  陀螺帧数    —— 不涨 = 陀螺没发 / UART2 接线错
         *   M  云台反馈数  —— 不涨 = CAN 没通 / 云台没使能
         *   BO Bus-Off 次数—— 不为 0 = CAN 接线、终端电阻、或收发器 TX/RX 接反
         *   E  当前 err_x (像素)
         *   R  当前偏航角速度 (°/s)  —— 调 GYRO_RATE_K 就看这个
         *   K/A 当前烧进去的 GYRO_RATE_K / GYRO_LPF_ALPHA (×100)。
         *       有它才能确认板子上跑的是哪一版参数。
         */
        if (++dbg_tick >= DEBUG_PERIOD_TICKS) {
            dbg_tick = 0;
            printf("K=%d A=%d | V=%lu vc=%lu vo=%lu vg=%u | G=%lu M=%lu TO=%lu ME=%lu RB=%lu | E=%ld R=%ld |R|=%ld\r\n",
                   (int) (GYRO_RATE_K * 100.0f),      /* 当前烧进去的速率环增益 ×100 */
                   (int) (GYRO_LPF_ALPHA * 100.0f),   /* 低通系数 ×100, 100 = 关闭 */
                   (unsigned long) vcount,
                   (unsigned long) g_vision_bad_csum,
                   (unsigned long) g_vision_overrun,
                   (unsigned) vis_gap_max,
                   (unsigned long) g_gyro_rx_count,
                   (unsigned long) g_gimbal_rx_count,
                   (unsigned long) g_gimbal_timeout,
                   (unsigned long) g_gimbal_rx_err,
                   (unsigned long) g_gimbal_rx_bytes,
                   (long) err_x,
                   (long) yaw_rate,
                   (long) ((rabs_cnt > 0u) ? (rabs_sum / (float) rabs_cnt) : 0.0f));
            vis_gap_max = 0;        /* 每个报告周期重新统计 */
            rabs_sum = 0.0f;
            rabs_cnt = 0;

#if DEBUG_GIMBAL_SNOOP
            /* UART1 最近收到的原始字节 (最旧 -> 最新)。看电机到底发的是什么:
             * 如果线上真是一帧一帧的, 这行里能直接看出帧结构 (ID/状态/角度/CRC)。
             * 排查"云台反馈收不到"时把它打开 —— 见 docs/BUGS.md #1。 */
            printf("  RX1:");
            for (di = 0u; di < GIMBAL_SNOOP_LEN; di++) {
                uint8_t idx = (uint8_t) ((g_gimbal_snoop_pos + di) %
                                         GIMBAL_SNOOP_LEN);
                printf(" %02x", g_gimbal_snoop[idx]);
            }
            printf("\r\n");
#else
            (void) di;
#endif
        }
#endif

        delay_cycles(LOOP_TICK_CYCLES);
    }
}
