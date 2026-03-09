#include "pwm_svpwm.h"
#include <math.h>

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline uint32_t u32_clamp(uint32_t x, uint32_t lo, uint32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void minmax3(float a, float b, float c, float *mn, float *mx)
{
    float max_ab = (a > b) ? a : b;
    float min_ab = (a < b) ? a : b;

    *mx = (c > max_ab) ? c : max_ab;
    *mn = (c < min_ab) ? c : min_ab;
}

static inline uint32_t duty_to_ccr(float duty, uint32_t arr)
{
    /* For most STM32 PWM modes, CCR in [0..ARR] maps duty ~ CCR/ARR.
       Use ARR+1 mapping to reduce endpoint bias on some setups.
       If you observe 50% not exactly at 0.5, switch to (duty * arr). */
    float x = duty * (float)(arr + 1u);

    if (x < 0.0f) x = 0.0f;
    if (x > (float)(arr + 1u)) x = (float)(arr + 1u);

    uint32_t ccr = (uint32_t)(x + 0.5f); /* round */
    if (ccr > arr) ccr = arr;
    return ccr;
}

void PWM_SVPWM_Init(PWM_SVPWM_Handle *h,
                    TIM_HandleTypeDef *htim1,
                    const PWM_SVPWM_Config *cfg)
{
    if (!h || !htim1 || !cfg) return;

    h->htim = htim1;
    h->cfg  = *cfg;

    PWM_SVPWM_RefreshARR(h);
}

void PWM_SVPWM_RefreshARR(PWM_SVPWM_Handle *h)
{
    if (!h || !h->htim) return;
    h->arr = __HAL_TIM_GET_AUTORELOAD(h->htim);
}

void PWM_SVPWM_ApplyFromError(PWM_SVPWM_Handle *h,
                              float errA_volts,
                              float errB_volts,
                              float errC_volts,
                              PWM_SVPWM_Out *out_opt)
{
    if (!h || !h->htim) return;

    const float Vdc = h->cfg.vdc_volts;
    if (Vdc <= 1.0f) return; /* invalid */

    const float k = h->cfg.k_gain;

    /* 1) Phase commands from error (phase-to-neutral) */
    float ua = k * errA_volts;
    float ub = k * errB_volts;
    float uc = k * errC_volts;

    /* 2) Zero-sequence injection (same logic as MATLAB code) */
    float umin, umax;
    minmax3(ua, ub, uc, &umin, &umax);

    float temp = 0.5f * (umin + umax);

    float vaO = ua - temp;
    float vbO = ub - temp;
    float vcO = uc - temp;
    float vnO = -temp;

    /* 3) Overmod protection: keep pole commands within +/- Vdc/2 */
    const float Vhalf = 0.5f * Vdc;

    if (h->cfg.enable_scaling)
    {
        float m1 = fabsf(vaO);
        float m2 = fabsf(vbO);
        float m3 = fabsf(vcO);
        float m4 = fabsf(vnO);

        float mmax = m1;
        if (m2 > mmax) mmax = m2;
        if (m3 > mmax) mmax = m3;
        if (m4 > mmax) mmax = m4;

        if (mmax > Vhalf && mmax > 1e-6f)
        {
            float s = Vhalf / mmax;
            vaO *= s; vbO *= s; vcO *= s; vnO *= s;
        }
    }
    else
    {
        vaO = clampf(vaO, -Vhalf, Vhalf);
        vbO = clampf(vbO, -Vhalf, Vhalf);
        vcO = clampf(vcO, -Vhalf, Vhalf);
        vnO = clampf(vnO, -Vhalf, Vhalf);
    }

    /* 4) Pole voltage -> duty */
    float dutyA = 0.5f + (vaO / Vdc);
    float dutyB = 0.5f + (vbO / Vdc);
    float dutyC = 0.5f + (vcO / Vdc);
    float dutyN = 0.5f + (vnO / Vdc);

    /* 5) Duty clamp for deadtime / safety */
    dutyA = clampf(dutyA, h->cfg.duty_min, h->cfg.duty_max);
    dutyB = clampf(dutyB, h->cfg.duty_min, h->cfg.duty_max);
    dutyC = clampf(dutyC, h->cfg.duty_min, h->cfg.duty_max);
    dutyN = clampf(dutyN, h->cfg.duty_min, h->cfg.duty_max);

    /* 6) Duty -> CCR */
    uint32_t ccr1 = duty_to_ccr(dutyA, h->arr);
    uint32_t ccr2 = duty_to_ccr(dutyB, h->arr);
    uint32_t ccr3 = duty_to_ccr(dutyC, h->arr);
    uint32_t ccr4 = duty_to_ccr(dutyN, h->arr);

    /* 7) Apply to TIM1 channels */
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_3, ccr3);
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_4, ccr4);

    if (out_opt)
    {
        out_opt->ua = ua; out_opt->ub = ub; out_opt->uc = uc;
        out_opt->temp = temp;

        out_opt->vaO = vaO; out_opt->vbO = vbO; out_opt->vcO = vcO; out_opt->vnO = vnO;

        out_opt->dutyA = dutyA; out_opt->dutyB = dutyB; out_opt->dutyC = dutyC; out_opt->dutyN = dutyN;

        out_opt->ccr1 = ccr1; out_opt->ccr2 = ccr2; out_opt->ccr3 = ccr3; out_opt->ccr4 = ccr4;
    }
}