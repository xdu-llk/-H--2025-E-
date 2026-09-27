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
 * 控制结构: 速率环
 * ---------------------------------------------------------------------------
 *   w_cmd = VIS_KP·err_x + LOS_FF·(x1/f) + RATE_SIGN·K·ω_陀螺
 *           └ 位置 P ┘     └ 速率前馈 ┘     └──── 自稳 ────┘
 *   限幅 ±W_MAX_DPS → ÷DPS_PER_RPM → 0x04 速度模式
 *
 *   位置 P : 偏多少补多少
 *   速率 FF: 靶在以多快的角速度跑, 直接按那个速度转 —— 消掉 P-only 的稳态滞后
 *   自稳   : 压掉车体转动带来的扰动
 *
 *   ⚠️ 陀螺在【云台】上, 自稳项是反馈环不是前馈, 且测不到小车【平移】。
 *      "瞄准一个点"仍靠视觉。详见 docs/云台自稳与速率环.md
 *   历史: 更早是【航向环】(IMU 角度闭环 + 0x07 角度步进), 已整体替换。
 * ---------------------------------------------------------------------------
 */

/* ==========================================================================
 * ⚠️ 首次上电必做: 符号自检 (云台无限位, 反了会一直转不停)
 * ---------------------------------------------------------------------------
 * ① 视觉符号: 靶纸放视野里, 盯调试行 E —— 收敛就对; 发散就把 MaixCam 侧
 *    camera_display.py 的 ERR_X_SIGN 改成 -1。
 * ② 自稳符号: 手转小车, 盯调试行 W —— 该【朝反方向】变; 同方向变就翻 RATE_SIGN。
 *    ⚠️ 现在的 -1.0f 是陀螺【搬上云台之前】标定的, 必须重验。
 *
 * 单独验①: RATE_HOLD_ONLY 置 1, 云台就只自稳、不追靶。
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
 * 视觉外环 (位置 P + 速率前馈)
 * ========================================================================== */
/* 位置 P, (°/s)/px。0.80 = 旧 AIM_GAIN_RATE(0.014 rad) × 57.3, 视觉带宽不变 */
#define VIS_KP_DPS_PER_PX       1.0f   //

/* 像素焦距, px/rad。⚠️ 推算值, 没实测。速率前馈【绝对依赖】它 —— 错了前馈
 * 就整体偏一个比例, 没法靠调 P 吸收。校验: 靶纸平移 50mm@1m 应使 err_x 变 13.5px */
#define F_PX                    271.0f

/* 卡尔曼速度估计 (px/s) -> 视线角速度 (°/s) */
#define PXS_TO_DPS              (57.2957795f / F_PX)

/* 速率前馈增益。1.0 = 理论值。靶在以多快的角速度跑, 直接按那个速度转 ——
 * 不等位置项把误差积起来, 所以【暂态】快得多、超调小。
 * ⚠️ 消不掉稳态误差: 稳态下 err_x 不变 -> x1 归零 -> 这一项自动归零。
 *    弯道的稳态误差是靠 RATE_K 那个几何关系兜的, 不是靠这里。 */
#define LOS_FF_GAIN             1.0f

/* 死区, px。只作用于视觉 P; 速率前馈和自稳项都不进死区。
 * ⚠️ 指标: 光斑 0.5cm @最远 1.25m ≈ 1.1 px (f=271) —— 死区必须小于它,
 *    否则光斑永远进不到靶心。1.0 正好卡在指标上, 再紧就压到卡尔曼的噪声底下了。
 *    实测若抖得厉害, 说明噪声比预期大, 退回 1.5 并把 KF_R 调大。 */
#define AIM_DEADBAND_PX         0.5f

/* 速度指令上限, °/s = 20 rpm。比搜索(46°/s)有余量, 又远小于电机 1000 rpm */
#define W_MAX_DPS               120.0f

/* ==========================================================================
 * 视觉误差 2 态卡尔曼滤波   状态 = [误差(px), 变化率(px/s)]
 * ---------------------------------------------------------------------------
 * err_x 有 ±3 px 抖动, 直接乘 VIS_KP 会让速度指令跟着抖。
 * 用卡尔曼而非取平均: 它多估一个变化率状态、能外推现在 —— 速率前馈要的就是它。
 * ========================================================================== */

/* 观测噪声方差。实测 σ≈2 -> 理论 R=4; 但 R=4 时阶跃滞后 57ms(两个视觉帧),
 * 降到 1.0 滞后大减而峰值几乎不变, 多出来的抖动由死区兜住 */
#define KF_R            1.0f

/* 过程噪声。越大越信观测。Q_RATE 给大点, 让速率前馈跟得上 */
#define KF_Q_POS        0.5f
#define KF_Q_RATE       10.0f

/* 丢这么久(ms)就复位滤波器, 免得重新捕获时旧状态跳变 */
#define KF_RESET_TICKS  200u

/* ==========================================================================
 * 找靶 (视觉在线, 但没识别到靶纸时)
 * ---------------------------------------------------------------------------
 * 丢靶后记住【最后一次有效 err_x 的方向】, 以固定角速度扫回去。自稳项照常工作。
 * ⚠️ 单向扫。靶纸在身后(超过 368°)就找不回来 —— 那要升级成往返扫。
 * ========================================================================== */

/* 扫描速率, °/s。曝光 5ms 时运动模糊只有 0.23° ≈ 1px */
#define SEARCH_DPS          46.0f

/* 丢靶前误差小于这个就不搜 —— 靶纸就在附近(比如被人遮挡), 乱搜反而跑远 */
#define SEARCH_MIN_ERR_PX   5.0f

/* 连续丢这么多帧才启动搜索。没它的话单帧检测失败就以 46°/s 猛推 (视觉在
 * err_x=10px 时才该推 0.8 °/s, 差五十多倍) -> 冲过靶心 -> 反向再推 -> 停不下来。
 * 门槛没到时瞄准项是 0、自稳项照常, 云台只是原地保持指向 —— 所以等得起, 取大点。 */
#define SEARCH_CONFIRM_FRAMES   10u

/* 最多推这么多拍 (5ms/拍 -> 8 秒 = 368°, 扫满一圈)。找到就退出, 8s 是最坏情况 */
#define SEARCH_MAX_TICKS    1600u

/* 上电扫描方向。-1 = 反方向。
 * ⚠️ 2026-09-27 实测: +1 扫的方向反了 (跑道上靶子在顺时针很小的角度内)。 */
#define BOOT_SEARCH_DIR     (-1.0f)

/* ==========================================================================
 * 速率环 (陀螺在云台上 -> 是反馈环)
 * ---------------------------------------------------------------------------
 *   w_cmd = RATE_SIGN·K·ω_云台  +  瞄准项
 *           └──── 自稳 ────┘
 * 陀螺测的是云台自己的角速度: ω_云台 = ω_扰动/(1+K)。K=1 把扰动减半。
 * ⚠️ 陀螺测不到小车【平移】; 想要真正的前馈必须把陀螺挪到车体上。
 * ⚠️ 历史: 用 0x07 做纯速率环时 K=0.3 就震。换 0x04 少了一层角度环动力学,
 *    但 K 能不能上到 1 以上必须实测。
 * ========================================================================== */

/* 陀螺轴。Z 轴朝上 -> 2 */
#define GYRO_RATE_AXIS      2u

/* 自稳增益。K=1 理论上把车体转动抵消一半 (ω_云台 = ω_车体/(1+K))。
 * ⚠️ 文档估算: 陀螺 100 Hz (τ≈10ms) 下 K 必须 < 1 —— 1.0 正好卡在边缘。
 *    实测 1.8 会震。上板先试 1.0; 震了就减半 0.5; 还震就 0.3。 */
#define RATE_K              1.0f

/* 自稳项符号。⚠️ 原 -1.0 是【陀螺搬上云台之前】标定的, 换位置必须重验 */
#define RATE_SIGN           (-1.0f)

/* 一阶低通, 每陀螺帧一次。1.0 = 关闭 */
#define GYRO_LPF_ALPHA      1.0f

/* 1 rpm = 6 °/s */
#define DPS_PER_RPM         6.0f

/* 1 = 只自稳不追靶 (单独验自稳符号用) */
#define RATE_HOLD_ONLY      0

/* ==========================================================================
 * 外环
 * ========================================================================== */

/* --- 视觉误差滤波器状态 (2 态卡尔曼) --- */
static float s_kf_x0 = 0.0f;    /* 误差 (px) */
static float s_kf_x1 = 0.0f;    /* 误差变化率 (px/s) */
static float s_kf_a  = 1.0f;    /* 协方差 P = [a b; b c] */
static float s_kf_b  = 0.0f;
static float s_kf_c  = 1.0f;
static bool  s_kf_on = false;   /* false = 还没初始化, 下次观测直接采纳 */

/* 喂一个新观测, 返回滤波后的误差。dt = 距上一帧的秒数。
 * ⚠️ 目标丢失期间【不要】调用 —— 状态会一直外推跑飞。 */
static float vision_filter(float z, float dt)
{
    float a, b, c, s, k0, k1, innov;

    if (!s_kf_on) {                 /* 首次观测: 直接采纳, 速度置 0 */
        s_kf_x0 = z;
        s_kf_x1 = 0.0f;
        s_kf_a  = 1.0f;
        s_kf_b  = 0.0f;
        s_kf_c  = 1.0f;
        s_kf_on = true;
        return z;
    }

    /* --- 预测 --- */
    s_kf_x0 += s_kf_x1 * dt;
    a = s_kf_a + dt * (s_kf_b + s_kf_b) + KF_Q_POS;
    b = s_kf_b + dt * s_kf_c;
    c = s_kf_c + KF_Q_RATE;

    /* --- 更新 --- */
    s     = a + KF_R;
    k0    = a / s;
    k1    = b / s;
    innov = z - s_kf_x0;            /* 新息: 观测 - 预测 */

    s_kf_x0 += k0 * innov;
    s_kf_x1 += k1 * innov;

    s_kf_a = a - k0 * a;
    s_kf_b = b - k0 * b;
    s_kf_c = c - k1 * b;

    return s_kf_x0;
}

/* 复位滤波器。目标重新捕获时调用, 免得旧状态造成位置跳变 */
static void vision_filter_reset(void)
{
    s_kf_on = false;
}

/* 卡尔曼估的误差变化率 (px/s)。⚠️ 要在 vision_filter() 之后读 */
static float vision_rate(void)
{
    return s_kf_x1;
}

/* 最近一次发出的速度指令 (rpm, 小数)。调试行打的是 ×10 的值。
 * 验自稳符号用它: 手转小车时 W 该朝反方向变; 同方向变就是 RATE_SIGN 反了 */
static float g_last_rpm = 0.0f;

/* 算并发出这一拍的速度指令: w = 瞄准项 + 自稳项 (°/s), 限幅后换算成 rpm。
 * gyro_ok=false 时摘掉自稳项 —— 拿陈旧角速度反馈会变成正反馈跑飞。
 * 返回 false 表示这一拍没发。 */
static bool rate_cmd(float w_aim_dps, float yaw_rate_dps, bool gyro_ok)
{
    float w = w_aim_dps;

    if (gyro_ok) {
        w += RATE_SIGN * RATE_K * yaw_rate_dps;
    }

    if (w > W_MAX_DPS) {
        w = W_MAX_DPS;
    } else if (w < -W_MAX_DPS) {
        w = -W_MAX_DPS;
    }

    /* °/s -> rpm。⚠️ 传小数、不要在这里取整 —— 取整会把低速分辨率砍到
     * 1 rpm = 6°/s, 最后几像素的指令就恒为 0, 走出"顿挫 + 收不进去"。
     * 编码在 gimbal_set_speed() 里做, 1 LSB = 0.18°/s。 */
    if (gimbal_set_speed(w / DPS_PER_RPM)) {
        g_last_rpm = w / DPS_PER_RPM;   /* 只在真发出去后才记; 发失败也记的话 W 证明不了任何事 */
        return true;
    }
    return false;
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

    float    err_x     = 0.0f;   /* 原始误差 (调试用) */
    float    err_x_f   = 0.0f;   /* 卡尔曼滤波后的误差 —— 控制用这个 */
    uint16_t tgt_lost  = 0;      /* 连续丢靶的帧数, 用来决定何时复位滤波器 */
    bool     err_valid = false;

    /* --- 找靶 --- */
    float    last_err_x  = 0.0f; /* 最后一次有效 err_x —— 丢靶后往哪边找 */
    uint16_t search_tick = 0;    /* 找靶已经推了多少拍 */
    bool     ever_seen   = false;/* 曾经识别到过靶纸吗 —— 区分上电找靶/丢靶找靶 */
    float    yaw_rate  = 0.0f;  /* 云台偏航角速度 (°/s) */
    float    yaw_rate_lpf = 0.0f;  /* 低通后的角速度, 自稳项喂这个 */
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
        gyro_link_poll();       /* 陀螺开机慢 ~5 s, 没就绪就重发初始化命令 */

        /* --- 视觉 --- */
        if (vision_link_get(&vmsg)) {
            /* 距上一帧的秒数 —— 卡尔曼的预测步要用。必须在 vis_lost 清零前算。
             * 帧间隔异常(丢帧/启动)时退回标称值, 免得速度估计被带偏。 */
            float dt_vis = (float) vis_lost * LOOP_TICK_SEC;
            if (dt_vis < 0.005f || dt_vis > 0.2f) {
                dt_vis = 0.0285f;
            }
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
                err_x = (float) vmsg.err_x;      /* 原始值, 调试用 */
                if (tgt_lost >= KF_RESET_TICKS) {
                    vision_filter_reset();       /* 丢太久, 重新捕获 -> 复位 */
                }
                err_x_f   = vision_filter(err_x, dt_vis);   /* 滤波后, 控制用 */
                err_valid = true;
                tgt_lost  = 0;
            } else {
                /* 丢靶。⚠️ err_x 恒为 0 是"无数据"不是"已对准", 必须用
                 * err_valid 区分开。视觉项归零后, 自稳项继续工作。 */
                err_valid = false;
                if (tgt_lost < 0xFFFFu) {
                    tgt_lost++;
                }
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
                /* 陀螺掉线时把自稳项摘掉, 别拿陈旧角速度继续反馈 */
                bool rate_ok = (gyro_lost < GYRO_LOST_TICKS);

                /* --- 瞄准项: 正常是视觉 P + 速率前馈, 丢靶时是搜索速率 --- */
                float w_aim = 0.0f;     /* °/s */

#if !RATE_HOLD_ONLY
                if (err_valid) {
                    ever_seen   = true;         /* 标记: 之后丢靶就按方向找 */
                    last_err_x  = err_x_f;      /* 记住方向, 丢靶后要用 */
                    search_tick = 0;

                    /* 位置项: 偏多少补多少 */
                    if ((err_x_f < -AIM_DEADBAND_PX) ||
                        (err_x_f >  AIM_DEADBAND_PX)) {
                        w_aim += VIS_KP_DPS_PER_PX * err_x_f;
                    }

                    /* 速率前馈: 靶在以多快的角速度跑, 直接按那个速度转, 不等位置项
                     * 慢慢把误差积出来。⚠️ 不进死区 —— 它是"速度", 掐了就跟丢动目标 */
                    w_aim += LOS_FF_GAIN * vision_rate() * PXS_TO_DPS;
                } else if ((vis_lost < VISION_LOST_TICKS) &&        /* 视觉还在线 */
                           (tgt_lost >= SEARCH_CONFIRM_FRAMES) &&   /* 确认真丢了 */
                           (search_tick < SEARCH_MAX_TICKS)) {      /* 还没超时 */
                    if (ever_seen &&
                        ((last_err_x >  SEARCH_MIN_ERR_PX) ||
                         (last_err_x < -SEARCH_MIN_ERR_PX))) {
                        /* --- 丢靶找靶: 按最后已知方向扫 --- */
                        w_aim = (last_err_x > 0.0f) ? SEARCH_DPS : -SEARCH_DPS;
                        search_tick++;
                    } else if (!ever_seen) {
                        /* --- 上电找靶: 从来没见过靶, 固定方向扫 --- */
                        w_aim = BOOT_SEARCH_DIR * SEARCH_DPS;
                        search_tick++;
                    }
                }
#endif
                if (!rate_cmd(w_aim, yaw_rate_lpf, rate_ok)) {
                    /* 没发出任何分量 —— 发 NOP 保活, 反馈才不会断
                     * (指令 0x00 就是"不改变任何东西, 只为索取反馈报文")。 */
                    gimbal_send_cmd(GIMBAL_CMD_NOP, 0);
                }
            }
        }

#if DEBUG_PRINT_ENABLE
        /* 上板 bring-up 用的状态行。⚠️ 会阻塞约 3.5 ms, 正式跑之前把
         * debug_uart.h 里的 DEBUG_PRINT_ENABLE 改成 0。
         *
         *   V/G/M   视觉/陀螺/云台反馈帧数 —— 不涨就是对应那路没通
         *   E / Ef  原始 / 滤波后的 err_x (px), 控制用 Ef
         *   R       偏航角速度 (°/s), ⚠️ 打印的是 ×10
         *   K       当前 RATE_K ×100 —— 确认板子跑的是哪版
         *   W       本拍发出的速度指令 (rpm), ⚠️ 打印的是 ×10 的值
         *           验自稳符号看它: 手转小车该朝反方向变; 同方向变 = RATE_SIGN 反了
         *   Vf      卡尔曼估的误差变化率 (px/s ×10), 速率前馈用的就是它。
         *           靶匀速平移时 Vf 稳定在非零值、Ef 接近 0; 若 Ef 一直差一截
         *           不收敛, 说明前馈不够 (调 F_PX 或 LOS_FF_GAIN)
         */
        if (++dbg_tick >= DEBUG_PERIOD_TICKS) {
            dbg_tick = 0;
            printf("K=%d | V=%lu vc=%lu vo=%lu vg=%u | G=%lu M=%lu TO=%lu ME=%lu RB=%lu | E=%ld Ef=%ld R=%ld |R|=%ld | W=%ld Vf=%ld | TX=%02x %02x %02x %02x %02x\r\n",
                   (int) (RATE_K * 100.0f),           /* 自稳增益 K ×100 */
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
                   (long) err_x_f,                              /* Ef: 滤波后 (控制用) */
                   (long) (yaw_rate * 10.0f),                   /* R: °/s ×10 */
                   (long) (((rabs_cnt > 0u) ? (rabs_sum / (float) rabs_cnt) : 0.0f) * 10.0f),
                   (long) (g_last_rpm * 10.0f),                 /* W: 速度指令 (rpm ×10) */
                   (long) (vision_rate() * 10.0f),              /* Vf: 误差变化率 (px/s ×10) */
                   (unsigned) g_gimbal_tx[0], (unsigned) g_gimbal_tx[1],
                   (unsigned) g_gimbal_tx[2], (unsigned) g_gimbal_tx[3],
                   (unsigned) g_gimbal_tx[4]);                  /* TX: 最近发给电机的整帧 */
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
