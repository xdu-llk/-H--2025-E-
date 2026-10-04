

/*
 * 瞄准模块主程序 —— MSPM0G3507  (2025 电赛 E 题招新改编版)
 *
 *   职责: MaixCam2 --(UART3, 6字节帧)--> MSPM0 --(UART1, 115200)--> QD4310 云台
 *         汇电籽-601 --(UART2, 100Hz)--^  (装在【车体】上, 测车体偏航角)
 *   一维(偏航)对准。底盘(循迹/电机)由另一块主控负责。
 *
 * 控制结构 —— 云台【开环】, 没有任何云台角度反馈
 * ---------------------------------------------------------------------------
 *     速度指令(0x04) = 几何前馈 + 视觉 P      两路都是 rad/s, 直接相加
 *
 *   几何前馈: 陀螺测车体偏航角 psi -> 反推车在弧上的位置 -> 靶的方位角
 *             变化率。见 guidance.h。
 *   视觉 P:   只管静态残差, 越靠近靶心速度越小。
 *
 *   ⚠️ QD4310 的速度是【转子相对定子】, 而定子螺栓固定在车架上 —— 车头
 *      自己转的那一份必须在指令里扣掉, 否则云台会跟着车头跑。
 * ---------------------------------------------------------------------------
 */



#include "ti_msp_dl_config.h"
#include "debug_uart.h"
#include "gimbal.h"
#include "guidance.h"
#include "gyro_link.h"
#include "vision_link.h"

#include <math.h>
#include <stdio.h>

/* 打开后每秒多打一行 UART1 收到的原始字节 (十六进制)。
 * 用于排查"云台反馈收不到"—— 详见 docs/BUGS.md #1。
 */
#define DEBUG_GIMBAL_SNOOP      0

/* ==========================================================================
 * 时间基准
 * ========================================================================== */

/* 主循环节拍。1 ms @ MCLK 32 MHz。整个控制逻辑都挂在这个节拍上。 */
#define LOOP_TICK_CYCLES        32000u
#define LOOP_TICK_SEC           0.001f

/* 发送周期。5 ms = 200 Hz。陀螺 100 Hz (10 ms), 5 ms 正好每 2 拍一个新角速度,
 * 前馈节奏均匀。发送事件驱动优先(新视觉帧立刻发), 这个周期是兜底。
 * ⚠️ 改这里必须同步改 gimbal.c 的 GIMBAL_RX_TIMEOUT_TICKS —— 两者必须相等。 */
#define SEND_PERIOD_TICKS       5u

/* 多久没收到反馈/陀螺数据就认为掉线。单位节拍。 */
#define GIMBAL_LOST_TICKS       300u   /* 300 ms */
#define GYRO_LOST_TICKS         100u   /* 100 ms —— 陀螺上报率远高于此 */

/* 云台健康检查开关 (基于反馈报文)。
 * 等反馈那个 bug 修好 (M 能正常涨) 之后, 改回 1 打开。 */
#define GIMBAL_HEALTH_CHECK     0

/* ==========================================================================
 * 云台初始化/恢复序列
 * ---------------------------------------------------------------------------
 * UART 是【一发一收】的: 发出一条之后, 状态会停在"等反馈", 最长 4 ms 才
 * 超时放行。 —— 第二条必然被 gimbal_send_cmd
 * 的"还在等反馈"挡掉, 而且不会报任何错。
 * 所以拆成状态: 每拍只尝试推进【一步】, 发成功了才进下一步, 发不出去下拍再试。
 * ========================================================================== */
#define GIM_STEP_IDLE       0u
#define GIM_STEP_CLEAR      1u      /* 该发"清错误" */
#define GIM_STEP_ENABLE     2u      /* 该发"使能" */
#define GIM_STEP_ZERO       3u      /* 该发"设零点" —— 上电人工摆正后定基准 */

/* 视觉掉线判定。MaixCam 约 55 fps (18 ms 一帧), 100 ms ≈ 连丢 5~6 帧。
 * ⚠️ 必须有这一条: 没有它的话 vision_link_get() 一直返回 false 时, err_valid
 *    和 err_x 会保持上一次的值 —— 云台会拿着一个**陈旧的误差**继续转下去,
 */
#define VISION_LOST_TICKS       100u

#define DEG2RAD                 0.017453292519943295f

/* ==========================================================================
 * 视觉外环
 * ========================================================================== */

/* 视觉外环增益, rad/(像素·秒):  Δθ = AIM_GAIN_RATE × err_x × dt
 *
 * dt是发送周期，所以每s的速率增加是相同的
 *
 * 取值反推自  AIM_GAIN_RATE = λ / (f · T_视觉):
 *     λ      每个视觉帧误差衰减的指数 = 环路穿越频率 ÷ 视觉帧率。
 *            >0.25 开始有振铃 (穿越频率别超采样率的 1/4)。
 *            当前 0.023×290×0.018 = 0.120, 余量 2 倍。
 *     f      像素焦距 = 290 (2026-09-28 实测)。
 *     T_视觉 视觉帧周期 18 ms (55 fps) —— 是【视觉帧率】, 不是发送周期!
 *
 * ⚠️ f 错了环路增益就错。校验法: 靶纸水平平移 50mm @1m, err_x 应变化
 *    约 14.5 px (= 50/1000 × 290)。 */
#define AIM_GAIN_RATE           0.023f

/* 死区, 像素。卡尔曼已把抖动压到亚像素, 0.5px @1m ≈ 2mm */
#define AIM_DEADBAND_PX         0.5f

/* 速度指令上限, rad/s。1.67 ≈ 95°/s */
#define W_MAX_RAD_S             1.67f

/* 1 rpm = 6°/s */
#define RAD_S_PER_RPM           0.10471975512f

/* 电机正转方向 vs 几何/视觉的约定 (都以"逆时针为正")。
 * 手册 5.3.2.1: 发正速度"沿正方向(逆时针)转动"; 上位机调参实测一致。
 * 而 guidance 里的 atan2 也是逆时针为正 ⇒ 两者同约定 ⇒ 取 +1。
 *
 * 几何的定位角、几何前馈、视觉 P 三者共用同一个约定 —— 要么全对要么全反,
 * 所以只用这一个符号统一翻。⚠️ 实测转反了就把它取反。 */
#define MOTOR_DIR_SIGN          (+1.0f)

/* 分级开关, 上台分步验证用:
 *   0 = 只发 0 速度(验通信)
 *   1 = 只几何前馈(断视觉, 验几何和符号)
 *   2 = 只视觉 P(断前馈, 验视觉符号)
 *   3 = 全部 */
#define AIM_STAGE               3

/* --- 进/出弯检测 ---
 * psi 分不清"左弯末尾"和"右弯开头"(都是 180°), 只能用 psi_dot:
 *   弯道 |psi_dot| ≈ v/R ≈ 1 rad/s     直道 ≈ 0 (只有循迹修正)
 * 迟滞: 进弯用高阈值、出弯用低阈值, 各连续 CURVE_DEBOUNCE 拍才认。 */
#define CURVE_ON_RADS       0.50f
#define CURVE_OFF_RADS      0.20f
#define CURVE_DEBOUNCE      20u     /* 5 ms 一拍 -> 100 ms */

/* 进弯定位 (0x05) 保持 60 拍 (5 ms/拍 = 300 ms), 然后交给前馈+视觉。
 * 电机内部位置环很快, 300 ms 够转完 143° 并稳定落位。 */
#define SYNC_MAX_TICKS  60u

/* ==========================================================================
 * 视觉误差 2 态卡尔曼滤波
 * ---------------------------------------------------------------------------
 * err_x 有 ±3 px 抖动, 直接喂进速度指令会让云台抖。
 *
 * 为什么用卡尔曼而不是"取平均": 它多估一个【变化率】状态, 能做预测,
 * 平滑的同时滞后小得多 —— 平均/低通只能"平滑过去", 它能"外推现在"。
 *
 * 状态 x = [误差(px), 误差变化率(px/s)]   观测 = 误差(px)
 * ========================================================================== */

/* 观测噪声方差 = σ²。
 * 实测靶纸静止时 err_x 在 ±3 px 跳 -> σ≈2 -> 理论上 R=4。
 * ⚠️ R 越大滞后越大: k0 = a/(a+R) 就是"每帧采纳多少观测"。
 *    现在 R=1 + Q_RATE=400 -> 跟得很快、滞后很小。
 *    代价是残留抖动变大, 由 AIM_DEADBAND_PX(0.5) 兜住。 */
#define KF_R            1.0f

/* 过程噪声。越大越信观测(跟得快、但噪声大)。
 * Q_POS 管误差, Q_RATE 管变化率 —— 变化率给大点, 让速度估计跟得上 */
#define KF_Q_POS        0.5f
#define KF_Q_RATE       400

/* 目标丢了超过这么久(ms)才复位滤波器 —— 视觉 P 是无记忆的, 而卡尔曼的
 * 速度状态 x1 在长时间丢靶后完全过期, 不复位会甩一下。 */
#define KF_RESET_MS     500u

/* ==========================================================================
 * 陀螺 (装在车体上, 测的是车体偏航角)
 * ========================================================================== */

/* 陀螺轴。Z 轴朝上 -> 2 */
#define GYRO_RATE_AXIS      2u

/* 一阶低通, 每陀螺帧一次 (100 Hz)。1.0 = 关闭 */
#define GYRO_LPF_ALPHA      1.0f

/* 陀螺原始角速度 (GyroZ) 与【车体偏航角 psi】的符号关系。
 * psi 是从模块 Yaw 差分累加的 (顺时针为正), 而模块的 Yaw 与 GyroZ 【反号】——
 * 由旧版航向环的 YAW_SIGN=-1 / GYRO_RATE_SIGN=-1 一起推出 (两个都实测过):
 *   阻尼要成立 ⇒ GYRO_RATE_SIGN = s, 而两者都是 -1 ⇒ s = -1。
 * ⚠️ 前馈漏了这个负号 -> 云台往反方向补 -> 直接跑飞。
 * (进/出弯检测取 |ψ̇|, 不受符号影响。) */
#define GYRO_RATE_SIGN      (-1.0f)
#define PSI_RATE()          (GYRO_RATE_SIGN * yaw_rate_lpf * DEG2RAD)

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

/* 本拍算出来的速度指令 (rad/s), 只给调试行看 */
static float s_last_w = 0.0f;

/* 算并发出这一拍的速度指令 (0x04)。
 *
 *   w_ff      几何前馈, rad/s —— 一直生效, 不依赖视觉
 *   err_valid 看到靶才叠加视觉 P
 *
 * 返回 false 表示这一拍没发出去。 */
static bool aim_send(float w_ff, bool err_valid, float err_x_f)
{
    float w;

#if AIM_STAGE == 0
    /* 只发 0 速度 —— 验证通信和电机速度环 */
    (void) w_ff; (void) err_valid; (void) err_x_f;
    w = 0.0f;
#else
    w = w_ff;

#if (AIM_STAGE == 2) || (AIM_STAGE == 3)
    if (err_valid &&
        ((err_x_f < -AIM_DEADBAND_PX) || (err_x_f > AIM_DEADBAND_PX))) {
        w += AIM_GAIN_RATE * err_x_f;
    }
#else
    (void) err_valid; (void) err_x_f;
#endif

    if (w > W_MAX_RAD_S) {
        w = W_MAX_RAD_S;
    } else if (w < -W_MAX_RAD_S) {
        w = -W_MAX_RAD_S;
    }
#endif

    w *= MOTOR_DIR_SIGN;        /* 见 MOTOR_DIR_SIGN */
    s_last_w = w;

    return gimbal_set_speed(w / RAD_S_PER_RPM);
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
    uint16_t lost_ms   = 0;      /* 距上次看到靶过了多少 ms (卡尔曼复位判据) */
    bool     err_valid = false;

    /* --- 车体偏航 (陀螺) --- */
    float    yaw_rate     = 0.0f; /* °/s, 瞬时 */
    float    yaw_rate_lpf = 0.0f; /* °/s, 低通后 —— 前馈用这个 */
    float    psi_rad      = 0.0f; /* 车体偏航角 (rad), 回绕差分累加 */
    float    psi_prev     = 0.0f; /* 上一帧原始 Yaw (deg), 算回绕用 */
    bool     psi_inited   = false;/* 首帧只记基准, 不累加 */
    bool     in_curve     = false;/* 当前在弯道上? 直道不瞄 */
    uint16_t curve_cnt    = 0;    /* |psi_dot| 连续超阈值的拍数, 去抖用 */
    bool     curve_sync   = false;/* 进弯后还在用 0x05 定位中 */
    uint16_t sync_cnt     = 0;    /* 定位发了多少拍, 超时兜底用 */
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
                dt_vis = 0.018f;
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
                /* 丢了一段再重捕获就复位滤波器 —— 旧的 x1 是陈旧的,
                 * 不复位的话新息巨大, 会给 x1 一大脚 -> 甩一下。 */
                if (lost_ms >= KF_RESET_MS) {
                    vision_filter_reset();
                }
                err_x_f   = vision_filter(err_x, dt_vis);   /* 滤波后, 控制用 */
                err_valid = true;
            } else {
                /* 丢靶。⚠️ err_x 恒为 0 是"无数据"不是"已对准", 必须用
                 * err_valid 区分开, 否则会把 0 当成"已经瞄好了"。 */
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

        /* 距上次看到靶过了多少 ms —— 滤波器复位的判据。
         * ⚠️ 用时间不用帧数: 视觉帧率随场景变, 帧数没有确定的时间含义。 */
        if (err_valid) {
            lost_ms = 0;
        } else if (lost_ms < 0xFFFFu) {
            lost_ms++;
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
                if (!psi_inited) {
                    psi_prev   = now;      /* 首帧只记基准, psi 从 0 起算 */
                    psi_inited = true;
                } else {
                    float d = now - psi_prev;
                    if (d > 180.0f) {
                        d -= 360.0f;       /* 359° -> 1° 是前进, 不是倒退 358° */
                    } else if (d < -180.0f) {
                        d += 360.0f;
                    }
                    psi_rad  += d * DEG2RAD;
                    psi_prev  = now;
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
                gim_step = GIM_STEP_ZERO;
            }
        } else if (gim_step == GIM_STEP_ZERO) {
            /* ⚠️ 上电时人已把云台摆到车头方向 —— 把这那一刻设成 0°。
             * 之后所有 0x05 绝对角都以它为准, 不设的话基准是随机的。 */
            if (gimbal_set_zero()) {
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
         * 纯定时发的话, err_x 从"到达"到"发出去"要白等一个发送周期 (平均半个),
         * 再叠加发送本身的零阶保持 —— 相当于给环路凭空加一段延迟。
         *
         * 改成收到新帧就立刻发之后, 这一项降到 0~1 ms (主循环 1 ms 节拍)。
         * send_tick 只在没有新帧时才累加 (need_send 为真时短路跳过), 所以视觉
         * 正常 (55 fps) 时定时分支根本不会触发, 它纯粹是视觉断流时的保活。 */
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
#if AIM_STAGE == 0
                /* 台架验证: 只发 0 速度 */
                if (!gimbal_set_speed(0.0f)) {
                    gimbal_send_cmd(GIMBAL_CMD_SPEED, 0);
                }
#else
                /* psi_dot = d(psi)/dt, 带符号 —— 见 GYRO_RATE_SIGN */
                bool  rate_ok = (gyro_lost < GYRO_LOST_TICKS);
                float psi_dot = rate_ok ? PSI_RATE() : 0.0f;

                /* --- 进/出弯检测 ---
                 * psi 分不清"左弯末尾"和"右弯开头"(都是 180°), 只能用 psi_dot:
                 *   弯道 |psi_dot| ≈ v/R ≈ 1 rad/s     直道 ≈ 0
                 * 迟滞 (进弯用高阈值/出弯用低阈值) + 连续 N 拍去抖。 */
                float thr = in_curve ? CURVE_OFF_RADS : CURVE_ON_RADS;

                if (fabsf(psi_dot) > thr) {
                    if (curve_cnt < 0xFFFFu) {
                        curve_cnt++;
                    }
                } else {
                    curve_cnt = 0;
                }

                bool curve_new = false;
                if (curve_cnt >= CURVE_DEBOUNCE) {
                    curve_cnt = 0;
                    if (in_curve) {
                        in_curve = false;       /* 出弯: 下一拍起云台停住 */
                    } else {
                        in_curve  = true;       /* 进弯 */
                        curve_new = true;
                    }
                }
                if (curve_new) {
                    curve_sync = true;
                }

                if (!in_curve) {
                    /* --- 直道: 不瞄, 云台停在原地 --- */
                    if (!aim_send(0.0f, false, 0.0f)) {
                        gimbal_send_cmd(GIMBAL_CMD_SPEED, 0);
                    }
                } else if (curve_sync) {
                    /* --- 进弯: 0x05 拉到几何绝对角, 保持满 SYNC_MAX_TICKS 再交棒。
                     * 不能"发一次就走"(下一拍 0x04 会覆盖位置环, 云台基本没转),
                     * 也不要拿视觉误差提前切 —— 打断位置环反而落不稳。 --- */
                    if (gimbal_set_angle(MOTOR_DIR_SIGN * guidance_abs(psi_rad))) {
                        if (++sync_cnt >= SYNC_MAX_TICKS) {
                            curve_sync = false;
                            sync_cnt   = 0;
                        }
                    }
                } else {
                    /* --- 弯道中: 几何前馈 + 视觉 P --- */
                    float w_ff = 0.0f;
#if (AIM_STAGE == 1) || (AIM_STAGE == 3)
                    w_ff = guidance_ff(psi_rad, psi_dot);
#endif
                    if (!aim_send(w_ff, err_valid, err_x_f)) {
                        /* ⚠️ 速度模式下【不能】补 NOP —— 0x00 是"保持上一拍速度",
                         *    云台会按旧速度一直转下去。必须补一条显式 0 速度。 */
                        gimbal_send_cmd(GIMBAL_CMD_SPEED, 0);
                    }
                }
#endif
            }
        }

#if DEBUG_PRINT_ENABLE
        /* 上板 bring-up 用的状态行。⚠️ 会阻塞约 3.5 ms, 正式跑之前把
         * debug_uart.h 里的 DEBUG_PRINT_ENABLE 改成 0 (详见该文件说明)。
         *
         *   V  视觉帧数    —— 不涨 = MaixCam 没发 / UART3 接线错
         *      vc/vo 校验错/溢出次数, vg 本周期最大帧间隔 (ms, 抓帧率抖动)
         *   G  陀螺帧数    —— 不涨 = 陀螺没发 / UART2 接线错
         *   M  云台反馈数  —— 不涨 = UART1 接线 / 云台没使能
         *      TO 反馈超时, ME 帧错, RB 收到的原始字节数
         *      (RB 恒为 0 才是硬件问题; RB 在涨但 M=0 是协议问题)
         *   E  当前 err_x (像素, 原始)     Ef 滤波后 —— 控制用的是 Ef
         *      两者对比就能看出卡尔曼压掉了多少抖动
         *   R  当前偏航角速度 (°/s) —— ⚠️ 打印的是 ×10 的值 (R=35 表示 3.5°/s)
         *   |R| 本周期平均 |角速度|, 同样 ×10
         *   S  当前 AIM_STAGE, 确认板子跑的是哪一档
         *
         *   psi 车体偏航角 (°), 由陀螺 Yaw 回绕累加。手推车转弯它会跟着变。
         *   Del ∠BOA (°), 由 psi 算出的车在弧上的位置。左弯 90°→180°→90°。
         *   W   本拍发出去的速度指令 (°/s) —— 前馈 + 视觉的合成结果。
         */
        if (++dbg_tick >= DEBUG_PERIOD_TICKS) {
            dbg_tick = 0;
            printf("S=%d | V=%lu vc=%lu vo=%lu vg=%u | G=%lu M=%lu TO=%lu ME=%lu RB=%lu | E=%ld Ef=%ld R=%ld |R|=%ld | psi=%ld Del=%ld W=%ld\r\n",
                   (int) AIM_STAGE,
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
                   (long) (psi_rad / DEG2RAD),                  /* psi: 车体偏航角 (°) */
                   (long) (guidance_delta(psi_rad) / DEG2RAD),  /* Del: ∠BOA (°) */
                   (long) (s_last_w / DEG2RAD));                /* W: 指令 (°/s) */
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
