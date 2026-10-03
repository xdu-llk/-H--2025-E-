/* guidance.c —— 见 guidance.h */

#include "guidance.h"

#include <math.h>

#define TWO_PI  6.28318530718f
#define PI_F    3.14159265f

/* psi 归一到 [0, 2π)。跑第二圈 psi 超过 360° 时要绕回来 */
static float wrap_psi(float psi_rad)
{
    float w = fmodf(psi_rad, TWO_PI);

    if (w < 0.0f) {
        w += TWO_PI;
    }
    return w;
}

float guidance_delta(float psi_rad)
{
    float w = wrap_psi(psi_rad);

    /* 左半圆 psi ∈ [0°, 180°)、右半圆 [180°, 360°)。
     * 两段 Δ 都从 -90° 递减到 -270°, 所以只是 Δ0 不同。 */
    if (w < PI_F) {
        return GEO_DELTA0_LEFT - w;
    }
    return GEO_DELTA0_RIGHT - w;
}

float guidance_abd(float psi_rad)
{
    /* 靶在【局部系】里的方位角。跨段时会跳 180°(那是顶部直道, psi 不变、
     * 前馈输出本来就 0), 只有它的导数是连续的。调试用, 控制不用它。 */
    float d = guidance_delta(psi_rad);

    return atan2f(-GEO_R_PATH * sinf(d), GEO_R_TGT - GEO_R_PATH * cosf(d));
}

float guidance_abs(float psi_rad)
{
    float w  = wrap_psi(psi_rad);
    float h;
    float th;

    /* θ = α(局部系) − 车头(局部系)。
     *   左半圆 局部系 = 世界系   ⇒ 车头 = 180° − ψ
     *   右半圆 局部系 = 世界转180 ⇒ 车头 = (180° − ψ) + 180° ≡ −ψ */
    if (w < PI_F) {
        h = PI_F - w;
    } else {
        h = -w;
    }

    th = guidance_abd(psi_rad) - h;

    /* 归一到 (-π, π] —— 右半圆的原始值会到 +213°, 不归一就和左半圆接不上 */
    while (th >  PI_F) {
        th -= TWO_PI;
    }
    while (th < -PI_F) {
        th += TWO_PI;
    }
    return th;
}

float guidance_ff(float psi_rad, float psi_rate)
{
    float d   = guidance_delta(psi_rad);
    float c   = cosf(d);
    float ab2 = GEO_R_PATH * GEO_R_PATH + GEO_R_TGT * GEO_R_TGT
                - 2.0f * GEO_R_PATH * GEO_R_TGT * c;

    /* θ = 靶相对车头的方位角, 要的是 dθ/dt。
     *   dα/dΔ = −R(r·cosΔ − R)/|AB|²        (α = 局部系里靶的方位角)
     *   Δ̇    = −ψ̇                          (Δ = Δ0 − ψ)
     *   车头角速度 = −ψ̇                     (车头 = 180° − ψ)
     *   ω = dα/dt − 车头角速度 = ψ̇·( 1 − dα/dΔ )
     * 那个 "− 车头角速度" 是坐标变换: 速度环听的是【转子相对定子】,
     * 定子固定在车架上, 车头自己转的那一份必须扣掉。
     * |AB|² 最小 (r−R)² = 0.0625 > 0, 无奇点。
     * 直道上 ψ̇ = 0 ⇒ 输出 0, 正好交给视觉 P 去跟。 */
    float dad = -GEO_R_PATH * (GEO_R_TGT * c - GEO_R_PATH) / ab2;

    return psi_rate * (1.0f - dad);
}
