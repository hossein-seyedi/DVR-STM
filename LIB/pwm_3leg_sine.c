#include "pwm_3leg_sine.h"
#include <math.h>

#define PWM_3LEG_TWO_PI   (6.28318530718f)
#define PWM_3LEG_120_DEG  (2.09439510239f)

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void minmax3(float a, float b, float c, float *vmin, float *vmax)
{
    float max_ab = (a > b) ? a : b;
    float min_ab = (a < b) ? a : b;

    *vmax = (c > max_ab) ? c : max_ab;
    *vmin = (c < min_ab) ? c : min_ab;
}

static inline uint32_t duty_to_ccr(float duty, uint32_t arr)
{
    float x = duty * (float)(arr + 1u);

    if (x < 0.0f) x = 0.0f;
    if (x > (float)(arr + 1u)) x = (float)(arr + 1u);

    uint32_t ccr = (uint32_t)(x + 0.5f);
    if (ccr > arr) ccr = arr;

    return ccr;
}

static void pwm_3leg_update_phase_step(PWM_3Leg_Handle *h)
{
    if (!h) return;

    if (h->cfg.pwm_update_hz <= 1.0f)
    {
        h->phase_step_rad = 0.0f;
        return;
    }

    h->phase_step_rad =
        PWM_3LEG_TWO_PI * h->cfg.output_freq_hz / h->cfg.pwm_update_hz;
}

void PWM_3Leg_Init(PWM_3Leg_Handle *h,
                   TIM_HandleTypeDef *htim,
                   const PWM_3Leg_Config *cfg)
{
    if (!h || !htim || !cfg) return;

    h->htim = htim;
    h->cfg  = *cfg;

    h->phase_rad = 0.0f;

    PWM_3Leg_RefreshARR(h);
    pwm_3leg_update_phase_step(h);
}

void PWM_3Leg_RefreshARR(PWM_3Leg_Handle *h)
{
    if (!h || !h->htim) return;
    h->arr = __HAL_TIM_GET_AUTORELOAD(h->htim);
}

void PWM_3Leg_SetOutputFrequency(PWM_3Leg_Handle *h, float output_freq_hz)
{
    if (!h) return;

    if (output_freq_hz < 0.0f) output_freq_hz = 0.0f;

    h->cfg.output_freq_hz = output_freq_hz;
    pwm_3leg_update_phase_step(h);
}

void PWM_3Leg_SetModulationIndex(PWM_3Leg_Handle *h, float modulation_index)
{
    if (!h) return;

    h->cfg.modulation_index = clampf(modulation_index, 0.0f, 1.0f);
}

void PWM_3Leg_Update(PWM_3Leg_Handle *h, PWM_3Leg_Debug *dbg)
{
    if (!h || !h->htim) return;
    if (h->cfg.vdc_volts <= 1.0f) return;

    float theta = h->phase_rad;
    float vdc   = h->cfg.vdc_volts;
    float vhalf = 0.5f * vdc;

    /* Peak phase reference before common-mode injection */
    float v_phase_peak = 0.5f * vdc * clampf(h->cfg.modulation_index, 0.0f, 1.0f);

    /* Three-phase sinusoidal references */
    float va_ref = v_phase_peak * sinf(theta);
    float vb_ref = v_phase_peak * sinf(theta - PWM_3LEG_120_DEG);
    float vc_ref = v_phase_peak * sinf(theta + PWM_3LEG_120_DEG);

    float va_cmd = va_ref;
    float vb_cmd = vb_ref;
    float vc_cmd = vc_ref;
    float v_zero = 0.0f;

    /* Optional zero-sequence injection for 3-leg inverter */
    if (h->cfg.enable_zero_sequence)
    {
        float vmin, vmax;
        minmax3(va_ref, vb_ref, vc_ref, &vmin, &vmax);

        v_zero = 0.5f * (vmin + vmax);

        va_cmd = va_ref - v_zero;
        vb_cmd = vb_ref - v_zero;
        vc_cmd = vc_ref - v_zero;
    }

    /* Final clamp to available DC bus */
    va_cmd = clampf(va_cmd, -vhalf, vhalf);
    vb_cmd = clampf(vb_cmd, -vhalf, vhalf);
    vc_cmd = clampf(vc_cmd, -vhalf, vhalf);

    /* Convert pole voltage command to duty */
    float duty_a = 0.5f + (va_cmd / vdc);
    float duty_b = 0.5f + (vb_cmd / vdc);
    float duty_c = 0.5f + (vc_cmd / vdc);

    duty_a = clampf(duty_a, h->cfg.duty_min, h->cfg.duty_max);
    duty_b = clampf(duty_b, h->cfg.duty_min, h->cfg.duty_max);
    duty_c = clampf(duty_c, h->cfg.duty_min, h->cfg.duty_max);

    uint32_t ccr_a = duty_to_ccr(duty_a, h->arr);
    uint32_t ccr_b = duty_to_ccr(duty_b, h->arr);
    uint32_t ccr_c = duty_to_ccr(duty_c, h->arr);

    /* Apply only to 3 legs */
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_3, ccr_c);

    /* Advance electrical angle */
    theta += h->phase_step_rad;
    if (theta >= PWM_3LEG_TWO_PI)
    {
        theta -= PWM_3LEG_TWO_PI;
    }
    h->phase_rad = theta;

    if (dbg)
    {
        dbg->theta_rad = theta;

        dbg->va_ref = va_ref;
        dbg->vb_ref = vb_ref;
        dbg->vc_ref = vc_ref;

        dbg->v_zero = v_zero;

        dbg->va_cmd = va_cmd;
        dbg->vb_cmd = vb_cmd;
        dbg->vc_cmd = vc_cmd;

        dbg->duty_a = duty_a;
        dbg->duty_b = duty_b;
        dbg->duty_c = duty_c;

        dbg->ccr_a = ccr_a;
        dbg->ccr_b = ccr_b;
        dbg->ccr_c = ccr_c;
    }
}

void PWM_3Leg_ApplyPhaseReferences(PWM_3Leg_Handle *h,
                                   float va_ref,
                                   float vb_ref,
                                   float vc_ref,
                                   PWM_3Leg_Debug *dbg)
{
    if (!h || !h->htim) return;
    if (h->cfg.vdc_volts <= 1.0f) return;

    const float vdc   = h->cfg.vdc_volts;
    const float vhalf = 0.5f * vdc;

    /* 1) Remove measured common-mode / DC offset */
    float v_avg = (va_ref + vb_ref + vc_ref) / 3.0f;

    float va_clean = va_ref - v_avg;
    float vb_clean = vb_ref - v_avg;
    float vc_clean = vc_ref - v_avg;

    /* 2) 3-leg zero-sequence injection */
    float vmin, vmax;
    minmax3(va_clean, vb_clean, vc_clean, &vmin, &vmax);

    float v_zero = 0.5f * (vmin + vmax);

    float va_cmd = va_clean - v_zero;
    float vb_cmd = vb_clean - v_zero;
    float vc_cmd = vc_clean - v_zero;

    /* 3) Safety scaling to keep commands inside +/- Vdc/2 */
    float m1 = fabsf(va_cmd);
    float m2 = fabsf(vb_cmd);
    float m3 = fabsf(vc_cmd);

    float mmax = m1;
    if (m2 > mmax) mmax = m2;
    if (m3 > mmax) mmax = m3;

    float scale = 1.0f;
    if (mmax > vhalf && mmax > 1e-6f)
    {
        scale = vhalf / mmax;
        va_cmd *= scale;
        vb_cmd *= scale;
        vc_cmd *= scale;
    }

    /* 4) Convert phase command to duty */
    float duty_a = 0.5f + (va_cmd / vdc);
    float duty_b = 0.5f + (vb_cmd / vdc);
    float duty_c = 0.5f + (vc_cmd / vdc);

    duty_a = clampf(duty_a, h->cfg.duty_min, h->cfg.duty_max);
    duty_b = clampf(duty_b, h->cfg.duty_min, h->cfg.duty_max);
    duty_c = clampf(duty_c, h->cfg.duty_min, h->cfg.duty_max);

    uint32_t ccr_a = duty_to_ccr(duty_a, h->arr);
    uint32_t ccr_b = duty_to_ccr(duty_b, h->arr);
    uint32_t ccr_c = duty_to_ccr(duty_c, h->arr);

    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(h->htim, TIM_CHANNEL_3, ccr_c);

    if (dbg)
    {
        dbg->theta_rad = 0.0f; /* not used in this mode */

        dbg->va_ref = va_ref;
        dbg->vb_ref = vb_ref;
        dbg->vc_ref = vc_ref;

        dbg->v_zero = v_zero;

        dbg->va_cmd = va_cmd;
        dbg->vb_cmd = vb_cmd;
        dbg->vc_cmd = vc_cmd;

        dbg->duty_a = duty_a;
        dbg->duty_b = duty_b;
        dbg->duty_c = duty_c;

        dbg->ccr_a = ccr_a;
        dbg->ccr_b = ccr_b;
        dbg->ccr_c = ccr_c;
    }
}