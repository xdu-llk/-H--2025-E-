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
 *   云台内部自带角度环, 主控在外面再套一层【航向环】, 增量式发指令:
 *
 *       目标航向 yaw_ref ──→ 角度环 PI ──→ 目标角速度 w ──阻尼──→ Δθ
 *                              ↑ 反馈 = IMU 连续航向 yaw_total
 *
 *   目标航向 (慢):  由视觉 err_x 累加而来 —— 决定"该朝哪看"。
 *
 *   角度环 (快):    把 yaw_total 拉到 yaw_ref, 带积分消掉稳态误差。
 *       ⚠️ 为什么不用纯速率环: 它要让 ω=0, 带宽必须高, 而 100 Hz 陀螺 +
 *          10 ms 延迟撑不住 —— 实测 K=0.15 就震。角度环只需"守住方向",
 *          带宽要求低得多, 这个延迟可以容忍。
 *
 *   阻尼:           陀螺角速度乘一个小系数抑制振铃。大了会自激。
 *
 *   详见 docs/云台自稳与速率环.md
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
#define AIM_GAIN_RATE           0.005f

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
#define AIM_MAX_RATE_RAD_S       1.30f

/* ==========================================================================
 * 视觉误差 2 态卡尔曼滤波
 * ---------------------------------------------------------------------------
 * err_x 有 ±3 px 抖动, 直接累加进 yaw_ref 会让云台抖。
 *
 * 为什么用卡尔曼而不是"取平均": 它多估一个【变化率】状态, 能做预测,
 * 平滑的同时滞后小得多 —— 平均/低通只能"平滑过去", 它能"外推现在"。
 *
 * 状态 x = [误差(px), 误差变化率(px/s)]   观测 = 误差(px)
 * ========================================================================== */

/* 观测噪声方差 = σ²。
 * 实测靶纸静止时 err_x 在 ±3 px 跳 -> σ≈2 -> 理论上 R=4。
 * ⚠️ 但仿真发现 R=4 时滤波器【阶跃滞后 57 ms】(= 2 个视觉帧),
 *    是视觉追踪过冲的主要贡献者之一。
 *    降到 2.0 -> 滞后减半到 28 ms, 而峰值只从 11.0 涨到 10.8(几乎不变)。
 *    多出来的抖动由 AIM_DEADBAND_PX 兜住。 */
#define KF_R            1.0f

/* 过程噪声。越大越信观测(跟得快、但噪声大)。
 * Q_POS 管误差, Q_RATE 管变化率 —— 变化率给大点, 让速度估计跟得上 */
#define KF_Q_POS        0.5f
#define KF_Q_RATE       10.0f

/* 目标丢失超过这么久(ms)就复位滤波器, 免得重新捕获时旧状态造成跳变 */
#define KF_RESET_TICKS  200u

/* ==========================================================================
 * 航向环 (IMU 角度闭环) —— 参考江南大学方案
 * ---------------------------------------------------------------------------
 * 江南: 角度PID(反馈=IMU角度) -> 速度PID(反馈=IMU角速度) -> 力矩
 * 我们电机内部自带角度环, 所以等价成: 角度环 PI + 阻尼 -> Δθ
 *
 * ⚠️ 为什么扔掉纯速率环: 它要让 ω=0, 带宽必须高, 100 Hz 陀螺 + 10 ms 延迟
 *    撑不住 —— 实测 K=0.15 就震。角度环只需"守住方向", 延迟不致命。
 *    详见 docs/云台自稳与速率环.md
 * ========================================================================== */

/* 陀螺轴。Z 轴朝上 -> 2 */
#define GYRO_RATE_AXIS      2u

/* 角度环 P, 单位 1/s。Kp=4 -> 时间常数 250 ms (远大于 10 ms 延迟, 安全) */
#define YAW_KP              8.0f

/* 角度环 I, 单位 1/s²。消掉"车匀速转"时的稳态误差 —— 江南 Ki=0.8 同理 */
#define YAW_KI              0.0f

/* 积分限幅, rad/s */
#define YAW_I_LIMIT         0.2f

/* 阻尼系数 (原来的 GYRO_RATE_K)。只做阻尼, 别大。
 * ⚠️ 实测: 纯速率环时 0.3 震 / 0.15 轻微震 / 0 不震 */
#define YAW_KD              0.10f

/* 陀螺速率项的符号 (阻尼用)。反了会"车一转云台就朝同方向猛甩" */
#define GYRO_RATE_SIGN      (-1.0f)

/* ⚠️ 模块的 Yaw 与它自己的 GyroZ 【符号相反】—— 2026-09-23 实测:
 *        手转云台时 gZ 持续为正, 而 yaw100 持续下跌。
 *    我们的控制律要求两者同向, 所以累加时翻一下。
 *    不翻的话角度环会变成【正反馈】-> 云台一直转、回不到目标。
 *    ⚠️ 副作用: 调试行的 Y 显示的是翻转后的值, 和模块原始 Yaw 差个正负号。 */
#define YAW_SIGN            (-1.0f)

/* 阶段开关: 1 = 只测自稳(yaw_ref 固定, 不接视觉) / 0 = 接视觉
 * 先跑 1, 确认"手转车身云台能守住方向", 再改 0 */
#define YAW_HOLD_ONLY       0

/* 一阶低通, 每陀螺帧一次。1.0 = 关闭 (滤波会加滞后, 先别开) */
#define GYRO_LPF_ALPHA      1.0f

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

/* 角度环的积分状态 */
static float g_yaw_i = 0.0f;

/* 算并发出这一步转角。
 *
 * 陀螺有效时走【航向环】: 角度环(PI) 把 yaw_total 拉到 yaw_ref, 再用陀螺
 * 角速度做阻尼。陀螺掉线时退回【纯视觉】直接发增量 —— 因为 yaw_total 会
 * 冻住, 继续用角度环等于拿陈旧值硬顶。
 *
 *   yaw_ref_rad   : 目标航向 (rad, 连续)
 *   yaw_total_rad : IMU 连续航向 (rad)
 *   yaw_rate_dps  : 云台偏航角速度 (°/s)
 *   gyro_ok       : 陀螺数据是否有效
 *   err_valid / err_x_px : 兜底用的视觉误差
 * 返回 false 表示这一拍没发。 */
static bool aim_step(float yaw_ref_rad, float yaw_total_rad,
                     float yaw_rate_dps, bool gyro_ok,
                     bool err_valid, float err_x_px)
{
    float dtheta;

    if (!gyro_ok) {
        /* --- 兜底: 纯视觉 --- */
        g_yaw_i = 0.0f;         /* 积分清零, 免得陀螺恢复时甩一下 */
        if (!err_valid ||
            ((err_x_px >= -AIM_DEADBAND_PX) && (err_x_px <= AIM_DEADBAND_PX))) {
            return false;
        }
        dtheta = AIM_GAIN_RATE * err_x_px * SEND_PERIOD_SEC;
    } else {
        /* --- 航向环: 角度 PI + 速度阻尼 --- */
        float e = yaw_ref_rad - yaw_total_rad;   /* 两边都连续, 不用回绕处理 */
        float w = YAW_KP * e + g_yaw_i;          /* 目标角速度, rad/s */
        w += GYRO_RATE_SIGN * YAW_KD * (yaw_rate_dps * DEG2RAD);

        if (w > AIM_MAX_RATE_RAD_S) {
            w = AIM_MAX_RATE_RAD_S;
        } else if (w < -AIM_MAX_RATE_RAD_S) {
            w = -AIM_MAX_RATE_RAD_S;
        }

        /* 积分放在限幅【之后】, 免得饱和期间继续累积 */
        g_yaw_i += YAW_KI * e * SEND_PERIOD_SEC;
        if (g_yaw_i > YAW_I_LIMIT) {
            g_yaw_i = YAW_I_LIMIT;
        } else if (g_yaw_i < -YAW_I_LIMIT) {
            g_yaw_i = -YAW_I_LIMIT;
        }

        dtheta = w * SEND_PERIOD_SEC;
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

    float    err_x     = 0.0f;   /* 原始误差 (调试用) */
    float    err_x_f   = 0.0f;   /* 卡尔曼滤波后的误差 —— 控制用这个 */
    uint16_t tgt_lost  = 0;      /* 连续丢靶的帧数, 用来决定何时复位滤波器 */
    bool     err_valid = false;
    float    yaw_rate  = 0.0f;  /* °/s */
    float    yaw_rate_lpf = 0.0f;  /* 低通后的角速度, 速率环喂这个 */
    float    yaw_total  = 0.0f;   /* IMU 连续航向 (rad), 由回绕累加得到 */
    float    yaw_prev   = 0.0f;   /* 上一帧原始航向 (deg), 算回绕用 */
    bool     yaw_inited = false;  /* 首帧只记基准, 不累加 */
    float    yaw_ref    = 0.0f;   /* 目标航向 (rad)。阶段1固定 0 = 锁定开机朝向 */
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
                 * err_valid 区分开。视觉项归零后, 航向环继续维持自稳。 */
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

            /* --- 航向连续化: 模块给 0~360 回绕, 累加成连续角 ---
             * 用【差分累加】而不是直接取绝对值, 这样偶尔漏一帧也不丢信息。 */
            {
                float now = gmsg.yaw_deg;
                if (!yaw_inited) {
                    yaw_prev   = now;      /* 首帧只记基准, yaw_total 从 0 起算 */
                    yaw_inited = true;
                } else {
                    float d = now - yaw_prev;
                    if (d > 180.0f) {
                        d -= 360.0f;       /* 359° -> 1° 是前进, 不是倒退 358° */
                    } else if (d < -180.0f) {
                        d += 360.0f;
                    }
                    yaw_total += YAW_SIGN * d * DEG2RAD;
                    yaw_prev   = now;
                }
            }
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

#if !YAW_HOLD_ONLY
                /* 阶段 2: 视觉驱动目标航向 —— 用【滤波后】的误差累加到 yaw_ref */
                if (err_valid &&
                    ((err_x_f < -AIM_DEADBAND_PX) || (err_x_f > AIM_DEADBAND_PX))) {
                    yaw_ref += AIM_GAIN_RATE * err_x_f * SEND_PERIOD_SEC;
                }
#endif
                if (!aim_step(yaw_ref, yaw_total, yaw_rate_lpf, rate_ok,
                              err_valid, err_x)) {
                    /* 没发出任何分量 —— 发 NOP 保活, 反馈才不会断
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
         *   E  当前 err_x (像素, 原始)     Ef 滤波后 —— 控制用的是 Ef
         *      两者对比就能看出卡尔曼压掉了多少抖动
         *   R  当前偏航角速度 (°/s) —— ⚠️ 打印的是 ×10 的值 (R=35 表示 3.5°/s)
         *   |R| 本周期平均 |角速度|, 同样 ×10
         *   P/D 当前烧进去的 YAW_KP(×10) / YAW_KD(×100) —— 确认板子跑的是哪版
         *
         *   Y  连续航向 (°), 由陀螺 Yaw 回绕累加而来。手转车身它会跟着变,
         *      云台自稳成功的话它【应该基本不动】。
         *   e  角度误差 = yaw_ref − yaw_total (°)。自稳成功时 e 应收敛到 0 附近。
         */
        if (++dbg_tick >= DEBUG_PERIOD_TICKS) {
            dbg_tick = 0;
            printf("P=%d D=%d | V=%lu vc=%lu vo=%lu vg=%u | G=%lu M=%lu TO=%lu ME=%lu RB=%lu | E=%ld Ef=%ld R=%ld |R|=%ld | Y=%ld e=%ld\r\n",
                   (int) (YAW_KP * 10.0f),            /* 角度环 P ×10 */
                   (int) (YAW_KD * 100.0f),           /* 阻尼系数 ×100 */
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
                   (long) (yaw_total / DEG2RAD),                /* Y: 连续航向 (°) */
                   (long) ((yaw_ref - yaw_total) / DEG2RAD));   /* e: 角度误差 (°) */
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
