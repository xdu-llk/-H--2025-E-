/*
 * guidance —— 赛道几何前馈 (H 题)
 *
 * 陀螺测车体偏航角 psi, 由 psi 反推小车在弧上的位置, 再算出靶的方位角变化率,
 * 直接作为云台的速度指令。纯函数, 不碰任何外设, 可在 PC 上单测。
 *
 * 几何 (椭圆赛道, 两个半圆各 r=0.5, 靶在椭圆中心):
 *     O = 当前半圆的圆心, A = 靶, B = 小车
 *     r = |OA| = 0.75,  R = |OB| = 0.5
 *     Δ = 在【局部系】(圆心为原点、靶放在 +x) 里 B 的极角
 *     左半圆 Δ₀ = -90°, 右半圆 Δ₀ = +90°, 转完一圈各扫过 -180°
 *
 * 云台零度 = 车头方向 (上电人工摆正时定死)。
 * 目标量是 dθ/dt, θ = 靶相对车头的方位角 —— θ 与坐标系无关, 两个半圆通用。
 */

#ifndef GUIDANCE_H_
#define GUIDANCE_H_

/* 赛道尺寸 (H 题赛题给的) */
#define GEO_R_PATH      0.5f        /* 车到半圆圆心 */
#define GEO_R_TGT       0.75f       /* 靶到半圆圆心 */

/* 按 psi 分段, 见 guidance_delta() */
#define GEO_DELTA0_LEFT   (-1.5707963f)   /* -90°: 左半圆, psi ∈ (-180°, 0]  */
#define GEO_DELTA0_RIGHT  ( 1.5707963f)   /* +90°: 右半圆, psi ∈ (-360°, -180°] */

/* Δ(psi), 单位 rad, 已按段选好 Δ0 并对 psi 取模 */
float guidance_delta(float psi_rad);

/* 靶相对车头的方位角 θ(psi), 单位 rad。调试核对用, 控制不用它 */
float guidance_abd(float psi_rad);

/* 进弯时用来把云台一把拉正的【绝对角】θ(psi), 单位 rad, 归一到 (-π, π]。
 * θ 与局部系无关, 两个半圆通用。 */
float guidance_abs(float psi_rad);

/* 前馈角速度 dθ/dt, 单位 rad/s。psi_rate 是车体偏航角速度 */
float guidance_ff(float psi_rad, float psi_rate);

#endif /* GUIDANCE_H_ */
