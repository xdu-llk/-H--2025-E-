

/*
 * 瞄准模块主程序 —— MSPM0G3507  (2025 电赛 E 题招新改编版)
 *
 *   职责: MaixCam2 --(UART3, 6字节帧)--> MSPM0 --(UART1, 115200)--> QD4310 云台
 *         汇电籽-601 --(UART2, 100Hz)--^  (角速度反馈 -> 航向环)
 *   一维(偏航)对准。
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
 *
 *   阻尼:           陀螺角速度乘一个小系数抑制振铃。大了会自激。
 *
 *   详见 docs/云台自稳与速率环.md
 * ---------------------------------------------------------------------------
 */



#include "ti_msp_dl_config.h"
#include "debug_uart.h"
#include "gimbal.h"
#include "gyro_link.h"
#include "vision_link.h"

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

/* 发送周期。5 ms = 200 Hz。
 * 往返约 1.4 ms < 5 ms, 所以"一发一收"不会挡住这个节拍。
 * 发送仍然是【事件驱动】优先 —— 收到新视觉帧立刻发; 这个周期是兜底, 负责
 * 视觉帧之间 (以及视觉断流时) 继续推进。 */
#define SEND_PERIOD_TICKS       5u
#define SEND_PERIOD_SEC         ((float) SEND_PERIOD_TICKS * LOOP_TICK_SEC)

/* 多久没收到反馈/陀螺数据就认为掉线。单位节拍。 */
#define GIMBAL_LOST_TICKS       300u   /* 300 ms */
#define GYRO_LOST_TICKS         100u   /* 100 ms —— 陀螺上报率远高于此 */

/* 云台健康检查开关 (基于反馈报文)。
 * 等反馈那个 bug 修好 (M 能正常涨) 之后, 改回 1 打开。 */
#define GIMBAL_HEALTH_CHECK     0

/* ==========================================================================
 * 云台初始化/恢复序列
 * ---------------------------------------------------------------------------
 * UART 是【一发一收】的: 发出一条之后, 状态会停在"等反馈", 最长 10 ms 才
 * 超时放行。 —— 第二条必然被 gimbal_send_cmd
 * 的"还在等反馈"挡掉, 而且不会报任何错。
 * 所以拆成状态: 每拍只尝试推进【一步】, 发成功了才进下一步, 发不出去下拍再试。
 * ========================================================================== */
#define GIM_STEP_IDLE       0u
#define GIM_STEP_CLEAR      1u      /* 该发"清错误" */
#define GIM_STEP_ENABLE     2u      /* 该发"使能" */

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
 *            每帧消掉 1-e^(-λ)。>0.25 开始有振铃 (采样系统经验: 穿越频率
 *            别超过采样率的 1/4)。当前 0.146, 余量只剩 1.7 倍 
 *     f      像素焦距 ≈ 271。**推算值**(OS04D10 按 416x260 居中裁剪), 没实测
 *     T_视觉 视觉帧周期 18 ms (55 fps) —— 是【视觉帧率】, 不是发送周期!
 *
 * ⚠️ f 错了环路增益就错。校验法: 靶纸水平平移 50mm @1m, err_x 应变化
 *    约 14.5 px (= 50/1000 × 290)。 */
#define AIM_GAIN_RATE           0.023f

/* 像素焦距, px/rad。2026-09-28 实测 ≈290 (之前推算的 271 偏低 7%)。
 * ⚠️ 现在【没有任何地方消费它】—— 前馈砍掉视觉那一支之后就没人用了。
 *    留着是因为它是实测值, 而且 AIM_GAIN_RATE 那段的 λ 推导要参考它。
 * 顺带: 水平视场 = 2·atan(208/290) ≈ 71°, 垂直 ≈ 48°。 */
#define F_PX                    290.0f

/* 视线角速度前馈增益。
 *
 * 补偿的是【积分器的速度误差】: yaw_ref 要以 ω 匀速涨, 就必须有恒定输入 ——
 *     dyaw_ref/dt = AIM_GAIN_RATE × err_x = ω   ⇒   err_x = ω / AIM_GAIN_RATE
 * 把斜率直接喂进去, 它就不用靠误差去攒了。
 *
 * 只前馈【云台自己的绝对角速度】(陀螺那一支)。完整的量是
 *     ω_视线 = ω_云台 + d(err_x)/dt / f        (恒等式)
 * 但第二支要过卡尔曼, 有 ~23 ms 滞后, 与即时的陀螺对不上 —— 抵消不完全就变成
 * "滞后的微分"(负阻尼), 这正是以前取 1.0 会震的原因。所以砍掉它。
 *
 * 只留陀螺仍然够: 匀速绕圈时 d(err_x)/dt ≈ 0, 两支本来就相等。
 *     ė_r = (G−1)·ω_云台 + K·err_x  ⇒  err_x = (1−G)·ω/K  ⇒ G=1 时稳态误差为 0
 * ⚠️ 上限【严格】是 1.0 —— G>1 时 (G−1)·KP 变成正极点, 发散。
 * ⚠️ GYRO_LSB_PER_DPS 错 X 倍等效于 G 要取 1/X, 这个旋钮正好吸收它。
 * 起手 0.5, 往 1.0 调, 看到振就退。 */
#define LOS_FF_GAIN             0.055

/* 死区, 像素。
    卡尔曼滤波已经将err_x_f的抖动压在了亚像素级别，所以不需要很大的死区来抑制err_x_f的抖动
    ，0.5在1m处对应2mm以内*/
#define AIM_DEADBAND_PX         0.5f

/* 转角速率上限
 * 1.67 rad/s ≈ 95°/s
 * 注意它限的是**合成后**的总增量, 速率项也被一起限住 —— 速率项正常工作时量
 * 很小 (35°/s = 0.61 rad/s), 不会顶到这个上限。 */
#define AIM_MAX_RATE_RAD_S       1.67f

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
 * ⚠️ R 越大滞后越大: k0 = a/(a+R) 就是"每帧采纳多少观测", R=4 时阶跃滞后 57 ms。
 *    现在 R=1 + Q_RATE=100 -> 按稳态增益算 90% 约 23 ms (1.3 视觉帧), 过冲极小。
 *    代价是残留抖动变大, 由 AIM_DEADBAND_PX(0.5) 兜住。 */
#define KF_R            1.0f

/* 过程噪声。越大越信观测(跟得快、但噪声大)。
 * Q_POS 管误差, Q_RATE 管变化率 —— 变化率给大点, 让速度估计跟得上 */
#define KF_Q_POS        0.5f
#define KF_Q_RATE       400

/* 目标丢失超过这么久(ms)就复位滤波器, 免得重新捕获时旧状态造成跳变 */
#define KF_RESET_MS     5000u

/* ==========================================================================
 * 找靶 (视觉在线, 但没识别到靶纸时)
 * ========================================================================== */

/* 找靶总开关。0 = 完全不找靶 (丢靶就守在当前朝向自稳, 云台不转)。 */
#define SEARCH_ENABLE       1

/* 扫描速率, rad/s ≈ 46°/s。 */
#define SEARCH_RATE         0.70f

/* 丢靶前误差小于这个就不搜 —— 靶纸就在附近(比如被人遮挡), 乱搜反而跑远 */
#define SEARCH_MIN_ERR_PX   0.0f

/* 连续丢靶超过这么久(ms)才启动找靶。
 * ⚠️ 判据一律用【时间】不用帧数 
 * ⚠️ 没它的话, 视觉单帧检测失败就立刻以 SEARCH_RATE(46°/s) 猛推 yaw_ref ——
 *    而视觉环在 err_x=10px 时本来只该推 0.1 rad/s, 差十几倍。
 *    于是: 检测一抖 -> 猛推 -> 冲过靶心 -> 反向再推 -> 停不下来。
 * 200 ms 的道理: 门槛没到时 yaw_ref 冻住, 航向环照常自稳 —— 等待期云台只是
 *    原地保持指向。而真丢靶后光扫一圈就要 8 s, 晚 200 ms 起步无所谓。 */
#define SEARCH_LOST_MS      200u

/* 一次找靶最多扫这么久。8 s × SEARCH_RATE(0.80 rad/s) = 6.4 rad ≈ 366°, 扫满一圈。 */
#define SEARCH_MAX_MS       8000u

/* 误差收进 SEARCH_LOCK_PX 后, 还要【连续停留】这么久才算真锁定, 才允许重置
 * 扫描预算。
 * ⚠️ 只判"进过范围"不行: 云台扫过靶心时 err_x 必然从 +200 穿到 -200,
 *    中途一定经过中心 —— 那一刻就重置的话扫描永远停不下来。
 * ⚠️ 也不能用"看到靶"当条件: 扫过靶时云台会连续几十帧看到靶。 */
#define SEARCH_LOCK_PX      20.0f
#define SEARCH_LOCK_MS      300u

/* 上电扫描方向。
 * ⚠️ 2026-09-27 实测: +1 扫的方向反了 (跑道上靶子在顺时针很小的角度内)。 */
#define BOOT_SEARCH_DIR     (-1.0f)

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
#define YAW_KP              40.0f

/* 角度环 I, 单位 1/s²。消掉"车匀速转"时的稳态误差 —— 江南 Ki=0.8 同理 */
#define YAW_KI              0

/* 积分限幅, rad/s */
#define YAW_I_LIMIT         0.2f

/* 阻尼系数 (原来的 GYRO_RATE_K)。只做阻尼, 别大。
 * ⚠️ 实测: 纯速率环时 0.3 震 / 0.15 轻微震 / 0 不震 */
#define YAW_KD              0.15

//正确参数
#define GYRO_RATE_SIGN      (-1.0f)  


#define YAW_SIGN            (-1.0f)

/* 阶段开关: 1 = 只测自稳(yaw_ref 固定, 不接视觉) / 0 = 接视觉
 * 先跑 1, 确认"手转车身云台能守住方向", 再改 0 */
#define YAW_HOLD_ONLY       0

/* 一阶低通, 每陀螺帧一次 (100 Hz)。1.0 = 关闭。
 * ⚠️ 0.9 的截止频率 ≈ 45 Hz, 逼近奈奎斯特 (50 Hz), 基本等于没滤。
 *    阻尼项和前馈都用它, 要真滤噪得降到 0.3~0.5 (约 6~11 Hz),
 *    代价是给这两项都加滞后。 */
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
 *   err_valid / err_x_f : 兜底用的视觉误差 (滤波后)
 * 返回 false 表示这一拍没发。 */
static bool aim_step(float yaw_ref_rad, float yaw_total_rad,
                     float yaw_rate_dps, bool gyro_ok,
                     bool err_valid, float err_x_f)
{
    const float max_step = AIM_MAX_RATE_RAD_S * SEND_PERIOD_SEC;
    float dtheta;

    if (!gyro_ok) {
        /* --- 兜底: 纯视觉 ---
         * ⚠️ 陀螺掉线时 yaw_total 会【冻住】, 航向环的反馈就失效了 ——
         *    再用它会变成"云台转了但误差不变" -> 一路转到限位。
         *    所以摘掉航向环, 退回直接用视觉误差发增量。
         *    (视觉这条信息没断, 所以至少还在追靶, 只是失去自稳) */
        g_yaw_i = 0.0f;         /* 积分清零, 免得陀螺恢复时甩一下 */
        if (!err_valid ||
            ((err_x_f >= -AIM_DEADBAND_PX) && (err_x_f <= AIM_DEADBAND_PX))) {
            return false;
        }
        dtheta = AIM_GAIN_RATE * err_x_f * SEND_PERIOD_SEC;
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

    /* 统一限速 —— 两条路径都走这里。主路径里 w 已经夹过 (冗余但无害),
     * 兜底路径没有别的保护, 全靠这一道。 */
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

    float    err_x     = 0.0f;   /* 原始误差 (调试用) */
    float    err_x_f   = 0.0f;   /* 卡尔曼滤波后的误差 —— 控制用这个 */
    uint16_t lost_ms   = 0;      /* 距上次看到靶过了多少 ms (找靶和卡尔曼复位的判据) */
    uint16_t lock_ms   = 0;      /* 误差在锁定范围内连续停留了多少 ms */
    bool     err_valid = false;

    /* --- 找靶 --- */
    float    last_err_x  = 0.0f; /* 最后一次有效 err_x —— 丢靶后往哪边找 */
    uint16_t search_ms   = 0;    /* 本次找靶已经扫了多少 ms */
    bool     ever_seen   = false;/* 曾经识别到过靶纸吗 —— 区分上电找靶/丢靶找靶 */
    bool     searching   = false;/* 上一拍真推过 yaw_ref? 认回靶时要拉平 */
    bool     gyro_was_ok = false;/* 上一拍陀螺有效? 用来抓"刚上线"那一拍 */
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
                /* 重新捕获就复位 —— 两种情况:
                 *   ① 丢了很久 (KF_RESET_MS)
                 *   ② 搜索真推过 yaw_ref: 云台已扫走几十度, 像素系的旧状态无意义
                 * 不复位的话, 重捕获那一帧的新息巨大, 会给 x1 一大脚 -> 甩一下。 */
                if (searching || (lost_ms >= KF_RESET_MS)) {
                    vision_filter_reset();
                }
                err_x_f   = vision_filter(err_x, dt_vis);   /* 滤波后, 控制用 */
                err_valid = true;
            } else {
                /* 丢靶。⚠️ err_x 恒为 0 是"无数据"不是"已对准", 必须用
                 * err_valid 区分开。视觉项归零后, 航向环继续维持自稳。 */
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

        /* 距上次看到靶过了多少 ms —— 找靶启动和滤波器复位的统一判据。
         * ⚠️ 用时间不用帧数: 视觉帧率随场景变, 帧数没有确定的时间含义。 */
        if (err_valid) {
            lost_ms = 0;
        } else if (lost_ms < 0xFFFFu) {
            lost_ms++;
        }

        /* 在锁定范围内连续停留了多久 —— 判"真锁定", 见 SEARCH_LOCK_MS */
        if (err_valid &&
            (err_x_f > -SEARCH_LOCK_PX) && (err_x_f < SEARCH_LOCK_PX)) {
            if (lock_ms < 0xFFFFu) {
                lock_ms++;
            }
        } else {
            lock_ms = 0;
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
                /* 陀螺掉线时把速率项置无效, 别拿陈旧角速度继续猛补 */
                bool rate_ok = (gyro_lost < GYRO_LOST_TICKS);
                /* 视觉在线? ⚠️ 上电时 MaixCam 要好几秒才起来 (实测 ~44 s),
                 *    这期间 vis_lost 一路涨 —— 不在线就绝不找靶, 免得盲扫。 */
                bool vis_ok  = (vis_lost  < VISION_LOST_TICKS);

                /* 陀螺刚上线: yaw_total 才从 0 起算, 而 yaw_ref 是掉线期间由
                 * 视觉积分器自己攒的 —— 两者从来没对齐过。不拉平的话
                 * e = yaw_ref 就是几十度, 航向环一接手就把云台甩到别处去。
                 * (掉线期间 aim_step 走纯视觉兜底, 压根不看 yaw_total。) */
                if (rate_ok && !gyro_was_ok) {
                    yaw_ref   = yaw_total;
                    searching = false;
                }
                gyro_was_ok = rate_ok;

#if !YAW_HOLD_ONLY
                /* --- 阶段 2: 视觉驱动目标航向 ---
                 * 看到靶 -> 立刻停搜索, 用【滤波后】的误差累加 yaw_ref。 */
                if (err_valid) {
                    ever_seen   = true;         /* 标记: 之后丢靶就按方向找 */
                    last_err_x  = err_x_f;      /* 记住方向, 丢靶后要用 */
                    /* 从搜索切回跟踪: 【无条件】拉平。搜索期间 yaw_ref 比
                     * yaw_total 领先约 SEARCH_RATE/YAW_KP ≈ 0.2 rad (11.5°),
                     * 不拉掉的话航向环会继续把云台推过去 -> 冲过靶心 -> 靶出画
                     * -> 又丢靶又扫, 就是"变缓但停不住"。 */
                    if (searching) {
                        yaw_ref   = yaw_total;
                        searching = false;
                    }
                    /* 真锁定 (在范围内连续停留够久) 才重置扫描预算 —— 见 SEARCH_LOCK_MS */
                    if (lock_ms >= SEARCH_LOCK_MS) {
                        search_ms = 0;
                    } else if (search_ms > SEND_PERIOD_TICKS) {
                        /* 看到靶就退额度, 速率与累积对齐 (本块已在 if (err_valid) 里)。
                         * 没有这条的话, 瞄准误差一直卡在 SEARCH_LOCK_PX 外面时
                         * 只扣不回, 8000 耗尽后【永远不再扫】。 */
                        search_ms -= SEND_PERIOD_TICKS;
                    } else {
                        search_ms = 0;
                    }
                    /* 速率前馈: 只喂云台自己的绝对角速度 —— 见 LOS_FF_GAIN。
                     * ⚠️ 不受死区管 —— 它是速度项不是误差项, 误差进死区时它照样要顶。
                     * ⚠️ 必须判 rate_ok: 陀螺掉线时 yaw_rate_lpf 会【冻住】,
                     *    前馈拿着陈旧值一直往 yaw_ref 上加 -> 云台匀速转下去。 */
                    if (rate_ok) {
                        yaw_ref += LOS_FF_GAIN * (yaw_rate_lpf * DEG2RAD) * SEND_PERIOD_SEC;
                    }

                    if ((err_x_f < -AIM_DEADBAND_PX) || (err_x_f > AIM_DEADBAND_PX)) {
                        yaw_ref += AIM_GAIN_RATE * err_x_f * SEND_PERIOD_SEC;
                    }
                }

                /* --- 找靶 ---
                 * ⚠️ 故意用【独立 if】而非 else if: 这样 searching 只在真推了
                 *    yaw_ref 时才置位。塞进 else if 的话, "丢靶但误差太小不搜"
                 *    也会被标成 searching, 下次看到靶就误触发上面的拉平。 */
                if (SEARCH_ENABLE &&
                    !err_valid &&
                    vis_ok &&                             /* 视觉在线 */
                    (lost_ms   >= SEARCH_LOST_MS) &&      /* 丢得够久才搜 */
                    (search_ms <  SEARCH_MAX_MS)) {       /* 本次还没扫够一圈 */
                    if (!ever_seen) {
                        /* 上电找靶: 还没见过靶, 固定方向扫 */
                        yaw_ref   += BOOT_SEARCH_DIR * SEARCH_RATE * SEND_PERIOD_SEC;
                        search_ms += SEND_PERIOD_TICKS;
                        searching  = true;
                    } else if ((last_err_x >  SEARCH_MIN_ERR_PX) ||
                               (last_err_x < -SEARCH_MIN_ERR_PX)) {
                        /* 丢靶找靶: 按最后看到靶的方向扫。
                         * ⚠️ 误差太小就不搜 —— 靶纸可能就在附近(比如被人挡住),
                         *    乱扫反而跑远。 */
                        yaw_ref   += ((last_err_x > 0.0f) ? 1.0f : -1.0f) *
                                     SEARCH_RATE * SEND_PERIOD_SEC;
                        search_ms += SEND_PERIOD_TICKS;
                        searching  = true;
                    }
                }
#endif
                if (!aim_step(yaw_ref, yaw_total, yaw_rate_lpf, rate_ok,
                              err_valid, err_x_f)) {
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
         *      vc/vo 校验错/溢出次数, vg 本周期最大帧间隔 (ms, 抓帧率抖动)
         *   G  陀螺帧数    —— 不涨 = 陀螺没发 / UART2 接线错
         *   M  云台反馈数  —— 不涨 = UART1 接线 / 云台没使能
         *      TO 反馈超时, ME 帧错, RB 收到的原始字节数
         *      (RB 恒为 0 才是硬件问题; RB 在涨但 M=0 是协议问题)
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
