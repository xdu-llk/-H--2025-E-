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
 *         汇电籽-601 --(UART2)---------^  (角速度前馈)
 *   靶面是一条竖直中线, 光斑高度不敏感, 所以只需一维(偏航)对准。
 *   底盘(循迹/电机/定点停车)由另一块主控负责, 不在本程序范围内。
 *
 * 控制结构
 * ---------------------------------------------------------------------------
 *   云台内部自带角度环, 所以主控只做**外环**, 而且是增量式的:
 *
 *       Δθ = 视觉项 + 前馈项
 *          = AIM_GAIN_RATE × err_x × dt  −  ω_body × dt
 *
 *   视觉项 (慢, 负责绝对基准):
 *       dθ/dt = K·f·(θ_ref − θ) = (1/τ)(θ_ref − θ),  τ = 1/(K·f)
 *       一阶收敛、不过冲。
 *
 *   前馈项 (快, 负责抵消车身转动):
 *       云台轴角需跟踪 θ_shaft = β − ψ  (β: 车→靶方位角, ψ: 车身偏航角)
 *       两边求导:  dθ_shaft/dt = dβ/dt − dψ/dt
 *       陀螺直接测的就是 dψ/dt, 所以把它减掉。
 *
 *   ⚠️ 为什么前馈是必需的, 而不是锦上添花:
 *       只有视觉项时, 对斜坡输入的稳态跟随误差 = ω · τ。
 *       车在弯道上 ω ≈ 35°/s, τ ≈ 67ms -> 误差 2.4°,
 *       在 1.25m 处折合 5.2cm —— 已经超过 5cm 指标。
 *       弯道上 dψ/dt 是主分量(约 35°/s), dβ/dt 只有约 13°/s;
 *       前馈把那 35°/s 消掉之后, 视觉环只需追 13°/s, 残余误差降到约 1.9cm。
 *
 *   采样周期 T = SEND_PERIOD_SEC, 每拍收敛比例 λ = K·f·T。
 *   带一延迟的闭环特征方程 z² − z + λ = 0:
 *       λ < 0.25     -> 两个实根, 无振荡
 *       λ ∈ (0.25,1) -> 共轭复根, 模 √λ, 稳定但有振铃
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
 * 陀螺前馈的符号同理, 见下面 GYRO_FF_SIGN 的说明。
 * ========================================================================== */

#include "ti_msp_dl_config.h"
#include "debug_uart.h"
#include "gimbal.h"
#include "gyro_link.h"
#include "vision_link.h"

#include <stdio.h>

/* ==========================================================================
 * 时间基准
 * ========================================================================== */

/* 主循环节拍。1 ms @ MCLK 32 MHz。整个控制逻辑都挂在这个节拍上。 */
#define LOOP_TICK_CYCLES        32000u
#define LOOP_TICK_SEC           0.001f

/* 发送的**兜底周期**, 30 ms ≈ 33 Hz。
 *
 * 正常情况下发送是【事件驱动】的 —— 收到一帧视觉就立刻发 (见主循环)。
 * 这个周期只在两种情况下起作用:
 *   1. 视觉断流 (MaixCam 被关掉/掉线) 时维持 CAN 通信和陀螺前馈
 *   2. 视觉帧率偶尔掉到 33 Hz 以下时补一条
 *
 * 为什么必须保留这个兜底: QD4310 是应答模式, 收到控制报文才回反馈报文。
 * 完全靠视觉驱动的话, 视觉一停云台就既不收指令也不回反馈, 状态全瞎
 * (手册 5.3.3 也推荐定时触发)。 */
#define SEND_PERIOD_TICKS       30u
#define SEND_PERIOD_SEC         ((float) SEND_PERIOD_TICKS * LOOP_TICK_SEC)

/* 多久没收到反馈/陀螺数据就认为掉线。单位节拍。 */
#define GIMBAL_LOST_TICKS       300u   /* 300 ms */
#define GYRO_LOST_TICKS         100u   /* 100 ms —— 陀螺上报率远高于此 */

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

/* 外环增益, 单位 rad / (像素·秒)
 *
 *     Δθ_vision = AIM_GAIN_RATE × err_x × dt
 *
 * 与像素焦距 f、收敛时间常数 τ 的关系:   AIM_GAIN_RATE = 1 / (τ · f)
 * 与每拍收敛比例 λ 的关系:               λ = AIM_GAIN_RATE · f · SEND_PERIOD_SEC
 *
 * ⚠️ 必须定义成"速率"而不是"每帧多少弧度":
 *    增量式指令下, 真正决定环路增益的是【每帧增量 ÷ 发送周期】。
 *    按"每帧 +0.0015 rad"写的话, 把发送周期从 30ms 改成 5ms, 环路增益直接
 *    ×6, 控制器就振荡了。定义成速率之后, 改频率不影响闭环。
 *
 * 标定 f: 靶纸固定在工作距离, 让云台转一个已知角度 Δθ, 读 err_x 的变化 Δpx,
 *         则 f = Δpx / Δθ。例: 转 0.1 rad 靶心移动 30 px -> f = 300 px。
 *
 * ⚠️ 下面 0.05 是按 f ≈ 300 px 反推的:
 *        τ = 1/(0.05 × 300) ≈ 67 ms
 *        λ = 0.05 × 300 × 0.03 = 0.45   <-- 偏激进, 会有轻微振铃
 *    上板标定 f 之后, 建议先把 λ 压到 0.25~0.3 试, 稳了再往上加。
 *    例: f=300, 想要 λ=0.25 -> AIM_GAIN_RATE = 0.25/(300 × 0.03) ≈ 0.028 */
#define AIM_GAIN_RATE           0.05f

/* 死区, 像素。err_x 是量化过的像素值, 零附近有 ±1~2 px 的抖动,
 * 不设死区的话积分器会追着噪声随机游走。
 * 2 px ≈ 0.38° (按 f=300), 在 1 m 处是 6.7 mm —— 远小于 3 cm 指标, 尽管取。
 * ⚠️ 死区只作用于**视觉项**; 前馈项不受它影响 —— 车在转就得补偿, 跟视觉误差
 *    在不在死区里没关系。 */
#define AIM_DEADBAND_PX         2.0f

/* 转角速率上限, rad/s。重新捕获目标时 err_x 可能有几百像素, 不限速会甩一下。
 * 1.67 rad/s ≈ 95°/s, 既保护机械又不影响 2 s 内收敛。
 * 注意它限的是**合成后**的总增量, 前馈也被一起限住 —— 前馈正常工作时量很小
 * (35°/s = 0.61 rad/s), 不会顶到这个上限。 */
#define AIM_MAX_RATE_RAD_S      1.67f

/* ==========================================================================
 * 陀螺前馈
 * ========================================================================== */

/* 用哪个轴做前馈。按模块的实际安装方向改:
 * 0 = X, 1 = Y, 2 = Z。模块 Z 轴竖直朝上时用 2。 */
#define GYRO_FF_AXIS            2u

/* 前馈符号。
 * 符号约定: 车身逆时针(从上往下看)转时, 陀螺该轴读数为正, 则取 −1。
 * ⚠️ 装反了会越补越偏, 表现为"车一转云台就往同一个方向猛甩"。
 *    上板验证法: 手拿着车绕竖直轴慢慢转, 看云台是不是**朝反方向**补偿;
 *    如果跟着一起转, 就把这里改成 +1.0f。 */
#define GYRO_FF_SIGN            (-1.0f)

/* ==========================================================================
 * 外环
 * ========================================================================== */

/* 算并发出这一步转角。
 *   err_valid / err_x_px : 视觉项输入 (err_valid=false 时视觉项按 0 处理)
 *   yaw_rate_dps         : 车身偏航角速度, °/s (陀螺有效时传入)
 *   ff_valid             : 陀螺数据是否有效 (掉线时置 false, 免得用陈旧值猛补)
 * 返回 false 表示这一步没有任何分量, 没发。 */
static bool aim_step(bool err_valid, float err_x_px,
                     bool ff_valid, float yaw_rate_dps)
{
    const float max_step = AIM_MAX_RATE_RAD_S * SEND_PERIOD_SEC;
    float       dtheta   = 0.0f;

    /* --- 视觉项: 慢, 负责绝对基准 --- */
    if (err_valid &&
        ((err_x_px < -AIM_DEADBAND_PX) || (err_x_px > AIM_DEADBAND_PX))) {
        dtheta += AIM_GAIN_RATE * err_x_px * SEND_PERIOD_SEC;
    }

    /* --- 前馈项: 快, 抵消车身转动 ---
     * 不受死区约束, 也不受"是否看到靶"约束 —— 丢靶时云台照样该补偿车身转动,
     * 这正是"自稳"的含义。 */
    if (ff_valid) {
        dtheta += GYRO_FF_SIGN * (yaw_rate_dps * DEG2RAD) * SEND_PERIOD_SEC;
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
#endif

    float    err_x     = 0.0f;
    bool     err_valid = false;
    float    yaw_rate  = 0.0f;  /* °/s */
    bool     need_send = false; /* 本拍收到了新视觉帧 -> 立刻发, 不等定时 */
    uint8_t  gim_step  = GIM_STEP_IDLE;  /* 云台初始化序列的推进状态 */

    SYSCFG_DL_init();
#if DEBUG_PRINT_ENABLE
    debug_uart_init();
#endif
    gimbal_init();
    gyro_link_init();
    vision_link_init();

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
                 * err_valid 区分开。视觉项归零后, 前馈项继续维持自稳。 */
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
            yaw_rate  = gyro_raw_to_dps(gmsg.gyro_raw[GYRO_FF_AXIS]);
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

            if ((gim_step == GIM_STEP_IDLE) && (gim_lost >= GIMBAL_LOST_TICKS)) {
                /* 反馈断了一阵 —— 云台掉线或被失能。触发一次"清错误 + 使能"。
                 * 只置状态、不在这里发, 因为一发一收下两条不能背靠背发。 */
                gim_lost = 0;
                gim_step = GIM_STEP_CLEAR;
            } else if (gim_step != GIM_STEP_IDLE) {
                /* 初始化序列正在跑, 这一拍不占发送机会, 让给序列 */
            } else {
                /* 陀螺掉线时把前馈置无效, 别拿陈旧角速度继续猛补 */
                bool ff_ok = (gyro_lost < GYRO_LOST_TICKS);

                if (!aim_step(err_valid, err_x, ff_ok, yaw_rate)) {
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
         *   R  当前偏航角速度 (°/s)
         */
        if (++dbg_tick >= DEBUG_PERIOD_TICKS) {
            dbg_tick = 0;
            printf("V=%lu vc=%lu vo=%lu vg=%u | G=%lu M=%lu TO=%lu ME=%lu | E=%ld R=%ld\r\n",
                   (unsigned long) vcount,
                   (unsigned long) g_vision_bad_csum,
                   (unsigned long) g_vision_overrun,
                   (unsigned) vis_gap_max,
                   (unsigned long) g_gyro_rx_count,
                   (unsigned long) g_gimbal_rx_count,
                   (unsigned long) g_gimbal_timeout,
                   (unsigned long) g_gimbal_rx_err,
                   (long) err_x,
                   (long) yaw_rate);
            vis_gap_max = 0;        /* 每个报告周期重新统计 */
        }
#endif

        delay_cycles(LOOP_TICK_CYCLES);
    }
}
