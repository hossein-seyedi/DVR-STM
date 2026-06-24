#include "dynvolt_predictor.h"

#include <math.h>
#include <string.h>

#define DVP_PI_F                         3.14159265358979323846f
#define DVP_TWO_PI_F                     (2.0f * DVP_PI_F)
#define DVP_TWO_PI_OVER_3_F              (2.0f * DVP_PI_F / 3.0f)
#define DVP_DELTA_REBUILD_THRESHOLD_SEC  0.5e-6f
#define DVP_ONE_THIRD_F                  0.33333333333333333333f

static float DVP_AbsF32(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}

static float DVP_ClampF32(float x, float min_value, float max_value)
{
    if (x < min_value)
    {
        return min_value;
    }

    if (x > max_value)
    {
        return max_value;
    }

    return x;
}

static float DVP_Max3AbsF32(float a, float b, float c)
{
    float ma;
    float mb;
    float mc;

    ma = DVP_AbsF32(a);
    mb = DVP_AbsF32(b);
    mc = DVP_AbsF32(c);

    if (mb > ma)
    {
        ma = mb;
    }

    if (mc > ma)
    {
        ma = mc;
    }

    return ma;
}

static float DVP_Rms3F32(float a, float b, float c)
{
    return sqrtf(((a * a) + (b * b) + (c * c)) * DVP_ONE_THIRD_F);
}

static uint16_t DVP_CalcSamples(float time_sec,
                                float ts_sec,
                                uint16_t min_samples,
                                uint16_t max_samples)
{
    float n_float;
    uint32_t n_u32;

    if (ts_sec <= 0.0f)
    {
        return min_samples;
    }

    n_float = (time_sec / ts_sec) + 0.5f;

    if (n_float < (float)min_samples)
    {
        return min_samples;
    }

    n_u32 = (uint32_t)n_float;

    if (n_u32 > (uint32_t)max_samples)
    {
        n_u32 = (uint32_t)max_samples;
    }

    return (uint16_t)n_u32;
}

void DynVoltPredictor3P_GetDefaultConfig(DynVoltPredictor3P_Config *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    cfg->ts_sec = 200.0e-6f;
    cfg->td_sec = 190.0e-6f;

    cfg->f_start_hz = 25.0f;
    cfg->df_hz = 1.0f;

    cfg->vmax_volts = 80.0f;
    cfg->warmup_sec = 20.0e-3f;

    cfg->p_init = 100.0f;
    cfg->p_min = 1.0e-8f;
    cfg->p_max = 500.0f;

    cfg->enable_predictor = 1u;
    cfg->enable_adaptation = 1u;

    cfg->grid_hz = 50.0f;
    cfg->flicker_hz = 8.8028169f;
    cfg->fine_df_hz = 0.20f;

    cfg->phase_v_shift_rad = -DVP_TWO_PI_OVER_3_F;
    cfg->phase_w_shift_rad = DVP_TWO_PI_OVER_3_F;
}

static void DVP_ValidateConfig(DynVoltPredictor3P_Config *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    if (cfg->ts_sec <= 0.0f)
    {
        cfg->ts_sec = 200.0e-6f;
    }

    if (cfg->td_sec < 0.0f)
    {
        cfg->td_sec = 0.0f;
    }

    if (cfg->vmax_volts <= 0.0f)
    {
        cfg->vmax_volts = 80.0f;
    }

    if (cfg->warmup_sec < 0.0f)
    {
        cfg->warmup_sec = 0.0f;
    }

    if (cfg->p_init <= 0.0f)
    {
        cfg->p_init = 100.0f;
    }

    if (cfg->p_min <= 0.0f)
    {
        cfg->p_min = 1.0e-8f;
    }

    if (cfg->p_max <= cfg->p_min)
    {
        cfg->p_max = 500.0f;
    }

    if (cfg->grid_hz <= 0.0f)
    {
        cfg->grid_hz = 50.0f;
    }

    if (cfg->flicker_hz <= 0.0f)
    {
        cfg->flicker_hz = 8.8028169f;
    }

    if (cfg->fine_df_hz <= 0.0f)
    {
        cfg->fine_df_hz = 0.20f;
    }
}

static void DVP_BuildFrequencyBank(DynVoltPredictor3P *h)
{
    float fg;
    float ff;
    float df;

    if (h == 0)
    {
        return;
    }

    fg = h->cfg.grid_hz;
    ff = h->cfg.flicker_hz;
    df = h->cfg.fine_df_hz;

    h->freq_hz[0]  = fg - (3.0f * ff);

    h->freq_hz[1]  = fg - ff - (2.0f * df);
    h->freq_hz[2]  = fg - ff - df;
    h->freq_hz[3]  = fg - ff;
    h->freq_hz[4]  = fg - ff + df;
    h->freq_hz[5]  = fg - ff + (2.0f * df);

    h->freq_hz[6]  = fg;

    h->freq_hz[7]  = fg + ff - (2.0f * df);
    h->freq_hz[8]  = fg + ff - df;
    h->freq_hz[9]  = fg + ff;
    h->freq_hz[10] = fg + ff + df;
    h->freq_hz[11] = fg + ff + (2.0f * df);

    h->freq_hz[12] = fg + (3.0f * ff);

    /* Keep all oscillator frequencies positive and within a useful range. */
    {
        uint32_t m;

        for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
        {
            if (h->freq_hz[m] < 1.0f)
            {
                h->freq_hz[m] = 1.0f;
            }
        }
    }
}

static void DVP_RebuildFutureDelta(DynVoltPredictor3P *h, float td_pred)
{
    uint32_t m;

    if (h == 0)
    {
        return;
    }

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float angle;

        angle = DVP_TWO_PI_F * h->freq_hz[m] * td_pred;
        h->fut_cos[m] = cosf(angle);
        h->fut_sin[m] = sinf(angle);
    }

    h->last_future_delta_time_sec = td_pred;
}

static void DVP_UpdateFutureDeltaIfNeeded(DynVoltPredictor3P *h, float td_pred)
{
    uint8_t rebuild;

    if (h == 0)
    {
        return;
    }

    rebuild = 0u;

    if (h->sample_counter <= 1u)
    {
        rebuild = 1u;
    }

    if (DVP_AbsF32(td_pred - h->last_future_delta_time_sec) > DVP_DELTA_REBUILD_THRESHOLD_SEC)
    {
        rebuild = 1u;
    }

    if (rebuild != 0u)
    {
        DVP_RebuildFutureDelta(h, td_pred);
    }
}

void DynVoltPredictor3P_Init(DynVoltPredictor3P *h,
                             const DynVoltPredictor3P_Config *cfg)
{
    DynVoltPredictor3P_Config local_cfg;
    uint32_t i;
    uint32_t m;

    if (h == 0)
    {
        return;
    }

    DynVoltPredictor3P_GetDefaultConfig(&local_cfg);

    if (cfg != 0)
    {
        local_cfg = *cfg;
    }

    DVP_ValidateConfig(&local_cfg);

    memset(h, 0, sizeof(*h));

    h->cfg = local_cfg;

    h->delay_samples = DVP_CalcSamples(h->cfg.td_sec,
                                       h->cfg.ts_sec,
                                       1u,
                                       (uint16_t)(DYNVOLT_PRED_DELAY_BUF_LEN - 2u));

    h->warmup_samples = DVP_CalcSamples(h->cfg.warmup_sec,
                                        h->cfg.ts_sec,
                                        1u,
                                        60000u);

    h->buf_index = 0u;

    h->kout_state = 1.0f;
    h->kres_state = 0.0f;
    h->tcal_state = 0.0f;
    h->gamma_state = 0.50f;

    for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
    {
        h->x[i] = 0.0f;
        h->p[i] = h->cfg.p_init;
    }

    DVP_BuildFrequencyBank(h);

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float step_angle;

        step_angle = DVP_TWO_PI_F * h->freq_hz[m] * h->cfg.ts_sec;

        h->osc_cos[m] = 1.0f;
        h->osc_sin[m] = 0.0f;
        h->step_cos[m] = cosf(step_angle);
        h->step_sin[m] = sinf(step_angle);
    }

    h->phase_v_cos = cosf(h->cfg.phase_v_shift_rad);
    h->phase_v_sin = sinf(h->cfg.phase_v_shift_rad);
    h->phase_w_cos = cosf(h->cfg.phase_w_shift_rad);
    h->phase_w_sin = sinf(h->cfg.phase_w_shift_rad);

    h->last_future_delta_time_sec = -1.0f;
    DVP_RebuildFutureDelta(h, h->cfg.td_sec + h->tcal_state);

    h->dbg.delay_samples = h->delay_samples;
    h->dbg.warmup_samples = h->warmup_samples;

    h->initialized = 1u;
}

void DynVoltPredictor3P_Reset(DynVoltPredictor3P *h)
{
    DynVoltPredictor3P_Config cfg;

    if (h == 0)
    {
        return;
    }

    cfg = h->cfg;
    DynVoltPredictor3P_Init(h, &cfg);
}

static void DVP_UpdateSupervisor(DynVoltPredictor3P *h,
                                 float ea,
                                 float eb,
                                 float ec)
{
    float de_raw;
    float alpha_de;
    float e_rms_now;
    float e_max_now;
    float de_rms_now;
    float alpha_env;
    float target_score;
    float tau_flicker;
    float alpha_flicker;

    if (h == 0)
    {
        return;
    }

    if (h->sample_counter > 1u)
    {
        de_raw = (ea - h->e_prev_u) / h->cfg.ts_sec;
    }
    else
    {
        de_raw = 0.0f;
    }

    alpha_de = h->cfg.ts_sec / (300.0e-6f + h->cfg.ts_sec);
    alpha_de = DVP_ClampF32(alpha_de, 0.0f, 1.0f);

    h->de_filt_u += alpha_de * (de_raw - h->de_filt_u);
    h->e_prev_u = ea;

    e_rms_now = DVP_Rms3F32(ea, eb, ec);
    e_max_now = DVP_Max3AbsF32(ea, eb, ec);
    de_rms_now = DVP_AbsF32(h->de_filt_u);

    alpha_env = h->cfg.ts_sec / (2.0e-3f + h->cfg.ts_sec);
    alpha_env = DVP_ClampF32(alpha_env, 0.0f, 1.0f);

    h->e_rms_env += alpha_env * (e_rms_now - h->e_rms_env);
    h->e_max_env += alpha_env * (e_max_now - h->e_max_env);
    h->de_rms_env += alpha_env * (de_rms_now - h->de_rms_env);

    if ((h->e_max_env > 5.5f) && (h->e_rms_env > 3.5f))
    {
        target_score = 1.0f;
    }
    else if ((h->e_max_env < 3.5f) && (h->e_rms_env < 2.2f) && (h->de_rms_env < 600.0f))
    {
        target_score = 0.0f;
    }
    else
    {
        target_score = h->flicker_score;
    }

    if (target_score > h->flicker_score)
    {
        tau_flicker = 1.0e-3f;
    }
    else
    {
        tau_flicker = 25.0e-3f;
    }

    alpha_flicker = h->cfg.ts_sec / (tau_flicker + h->cfg.ts_sec);
    alpha_flicker = DVP_ClampF32(alpha_flicker, 0.0f, 1.0f);

    h->flicker_score += alpha_flicker * (target_score - h->flicker_score);
    h->flicker_score = DVP_ClampF32(h->flicker_score, 0.0f, 1.0f);

    /* A small gamma is used to slightly open the RLS bandwidth during flicker. */
    h->gamma_state = 0.20f + (0.80f * h->flicker_score);
    h->gamma_state = DVP_ClampF32(h->gamma_state, 0.20f, 1.0f);
}

static void DVP_DelayedSelfEvaluation(DynVoltPredictor3P *h, float ea)
{
    uint16_t due_idx;
    float due_pred;
    float eval;
    float alpha_perf;
    float eval_rms;
    float y_rms;
    float pred_rms;
    float rel_err;
    float gain_corr;
    uint8_t adapt_active;

    if (h == 0)
    {
        return;
    }

    h->dbg.valid_delay = 0u;
    h->dbg.adapt_active = 0u;

    if (h->sample_counter <= ((uint32_t)h->warmup_samples + (uint32_t)h->delay_samples + 5u))
    {
        return;
    }

    due_idx = h->buf_index;

    if (due_idx >= h->delay_samples)
    {
        due_idx = (uint16_t)(due_idx - h->delay_samples);
    }
    else
    {
        due_idx = (uint16_t)(DYNVOLT_PRED_DELAY_BUF_LEN + due_idx - h->delay_samples);
    }

    due_pred = h->pred_buf_u[due_idx];
    eval = ea - due_pred;

    alpha_perf = h->cfg.ts_sec / (4.0e-3f + h->cfg.ts_sec);

    if (alpha_perf > 0.05f)
    {
        alpha_perf = 0.05f;
    }

    h->env_eval2 += alpha_perf * ((eval * eval) - h->env_eval2);
    h->env_y2 += alpha_perf * ((ea * ea) - h->env_y2);
    h->env_pred2 += alpha_perf * ((due_pred * due_pred) - h->env_pred2);

    if (h->env_eval2 < 0.0f)
    {
        h->env_eval2 = 0.0f;
    }

    if (h->env_y2 < 0.0f)
    {
        h->env_y2 = 0.0f;
    }

    if (h->env_pred2 < 0.0f)
    {
        h->env_pred2 = 0.0f;
    }

    eval_rms = sqrtf(h->env_eval2);
    y_rms = sqrtf(h->env_y2);
    pred_rms = sqrtf(h->env_pred2);
    rel_err = eval_rms / (y_rms + 0.5f);

    adapt_active = 0u;

    if ((h->cfg.enable_adaptation != 0u) &&
        (h->sat_buf[due_idx] == 0u) &&
        (y_rms > 1.0f) &&
        (pred_rms > 1.0f) &&
        (h->flicker_score > 0.20f))
    {
        adapt_active = 1u;
    }

    if (adapt_active != 0u)
    {
        gain_corr = (eval * due_pred) / ((due_pred * due_pred) + 4.0f);
        gain_corr = DVP_ClampF32(gain_corr, -0.10f, 0.10f);
        h->kout_state += 0.02f * gain_corr;
        h->kout_state = DVP_ClampF32(h->kout_state, 0.80f, 1.25f);
    }

    h->dbg.valid_delay = 1u;
    h->dbg.eval_rms = eval_rms;
    h->dbg.signal_rms = y_rms;
    h->dbg.pred_rms = pred_rms;
    h->dbg.rel_err = rel_err;
    h->dbg.adapt_active = adapt_active;
}

static void DVP_UpdateRlsAndPredict(DynVoltPredictor3P *h,
                                    float ea,
                                    float *pred_u,
                                    float *pred_v,
                                    float *pred_w)
{
    float r_meas;
    float q_h;
    float q_dc;
    float td_pred;
    float s_val;
    float e_hat;
    float innovation;
    float inv_s;
    float pu;
    float pv;
    float pw;
    float cv_shift;
    float sv_shift;
    float cw_shift;
    float sw_shift;
    uint32_t m;
    uint32_t col;

    if ((h == 0) || (pred_u == 0) || (pred_v == 0) || (pred_w == 0))
    {
        return;
    }

    r_meas = 3.0f - (2.0f * h->gamma_state);
    r_meas = DVP_ClampF32(r_meas, 0.75f, 4.0f);

    q_h = 1.0e-4f + h->gamma_state * 1.5e-3f;
    q_dc = 1.0e-7f;

    td_pred = h->cfg.td_sec + h->tcal_state;

    if (td_pred < 0.0f)
    {
        td_pred = 0.0f;
    }

    DVP_UpdateFutureDeltaIfNeeded(h, td_pred);

    e_hat = 0.0f;
    s_val = r_meas;

    /* DC term is used only to absorb measurement offset. It is not injected. */
    {
        float p_pred;

        p_pred = h->p[0] + q_dc;
        p_pred = DVP_ClampF32(p_pred, h->cfg.p_min, h->cfg.p_max);

        e_hat += h->x[0];
        s_val += p_pred;
    }

    col = 1u;

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float c_now;
        float s_now;
        float p_c;
        float p_s;

        c_now = h->osc_cos[m];
        s_now = h->osc_sin[m];

        p_c = h->p[col] + q_h;
        p_s = h->p[col + 1u] + q_h;

        p_c = DVP_ClampF32(p_c, h->cfg.p_min, h->cfg.p_max);
        p_s = DVP_ClampF32(p_s, h->cfg.p_min, h->cfg.p_max);

        e_hat += (c_now * h->x[col]) + (s_now * h->x[col + 1u]);
        s_val += (c_now * c_now * p_c) + (s_now * s_now * p_s);

        col += 2u;
    }

    if (s_val < 1.0e-12f)
    {
        s_val = 1.0e-12f;
    }

    innovation = ea - e_hat;
    inv_s = 1.0f / s_val;

    /* Update DC state, but do not add it to the output command. */
    {
        float p_pred;
        float k;
        float p_new;

        p_pred = h->p[0] + q_dc;
        p_pred = DVP_ClampF32(p_pred, h->cfg.p_min, h->cfg.p_max);

        k = p_pred * inv_s;
        h->x[0] += k * innovation;
        h->x[0] = DVP_ClampF32(h->x[0], -3.0f, 3.0f);

        p_new = (1.0f - k) * p_pred;
        h->p[0] = DVP_ClampF32(p_new, h->cfg.p_min, h->cfg.p_max);
    }

    pu = 0.0f;
    pv = 0.0f;
    pw = 0.0f;

    cv_shift = h->phase_v_cos;
    sv_shift = h->phase_v_sin;
    cw_shift = h->phase_w_cos;
    sw_shift = h->phase_w_sin;

    col = 1u;

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float c_now;
        float s_now;
        float p_c;
        float p_s;
        float k_c;
        float k_s;
        float p_new;
        float fuc;
        float fus;
        float fvc;
        float fvs;
        float fwc;
        float fws;
        float xc;
        float xs;

        c_now = h->osc_cos[m];
        s_now = h->osc_sin[m];

        p_c = h->p[col] + q_h;
        p_s = h->p[col + 1u] + q_h;

        p_c = DVP_ClampF32(p_c, h->cfg.p_min, h->cfg.p_max);
        p_s = DVP_ClampF32(p_s, h->cfg.p_min, h->cfg.p_max);

        k_c = p_c * c_now * inv_s;
        h->x[col] += k_c * innovation;
        p_new = (1.0f - (k_c * c_now)) * p_c;
        h->p[col] = DVP_ClampF32(p_new, h->cfg.p_min, h->cfg.p_max);

        k_s = p_s * s_now * inv_s;
        h->x[col + 1u] += k_s * innovation;
        p_new = (1.0f - (k_s * s_now)) * p_s;
        h->p[col + 1u] = DVP_ClampF32(p_new, h->cfg.p_min, h->cfg.p_max);

        xc = h->x[col];
        xs = h->x[col + 1u];

        fuc = (c_now * h->fut_cos[m]) - (s_now * h->fut_sin[m]);
        fus = (s_now * h->fut_cos[m]) + (c_now * h->fut_sin[m]);

        fvc = (fuc * cv_shift) - (fus * sv_shift);
        fvs = (fus * cv_shift) + (fuc * sv_shift);

        fwc = (fuc * cw_shift) - (fus * sw_shift);
        fws = (fus * cw_shift) + (fuc * sw_shift);

        pu += (fuc * xc) + (fus * xs);
        pv += (fvc * xc) + (fvs * xs);
        pw += (fwc * xc) + (fws * xs);

        col += 2u;
    }

    *pred_u = h->kout_state * pu;
    *pred_v = h->kout_state * pv;
    *pred_w = h->kout_state * pw;
}

static void DVP_ApplyAuthorityGate(DynVoltPredictor3P *h,
                                   float ea,
                                   float eb,
                                   float ec,
                                   float *pred_u,
                                   float *pred_v,
                                   float *pred_w)
{
    float alpha_target;
    float tau_alpha;
    float alpha_step;

    if ((h == 0) || (pred_u == 0) || (pred_v == 0) || (pred_w == 0))
    {
        return;
    }

    alpha_target = h->flicker_score;

    if (h->cfg.enable_predictor == 0u)
    {
        alpha_target = 0.0f;
    }

    if (h->sample_counter < (uint32_t)h->warmup_samples)
    {
        alpha_target *= (float)h->sample_counter / (float)h->warmup_samples;
    }

    alpha_target = DVP_ClampF32(alpha_target, 0.0f, 1.0f);

    if (alpha_target > h->alpha_pred_state)
    {
        tau_alpha = 0.8e-3f;
    }
    else
    {
        tau_alpha = 4.0e-3f;
    }

    alpha_step = h->cfg.ts_sec / (tau_alpha + h->cfg.ts_sec);
    alpha_step = DVP_ClampF32(alpha_step, 0.0f, 1.0f);

    h->alpha_pred_state += alpha_step * (alpha_target - h->alpha_pred_state);
    h->alpha_pred_state = DVP_ClampF32(h->alpha_pred_state, 0.0f, 1.0f);

    *pred_u = ea + h->alpha_pred_state * ((*pred_u) - ea);
    *pred_v = eb + h->alpha_pred_state * ((*pred_v) - eb);
    *pred_w = ec + h->alpha_pred_state * ((*pred_w) - ec);
}

static uint8_t DVP_ApplySaturation(DynVoltPredictor3P *h,
                                   float *pu,
                                   float *pv,
                                   float *pw)
{
    uint8_t saturated;

    if ((h == 0) || (pu == 0) || (pv == 0) || (pw == 0))
    {
        return 0u;
    }

    saturated = 0u;

    if (*pu > h->cfg.vmax_volts)
    {
        *pu = h->cfg.vmax_volts;
        saturated = 1u;
    }
    else if (*pu < -h->cfg.vmax_volts)
    {
        *pu = -h->cfg.vmax_volts;
        saturated = 1u;
    }

    if (*pv > h->cfg.vmax_volts)
    {
        *pv = h->cfg.vmax_volts;
        saturated = 1u;
    }
    else if (*pv < -h->cfg.vmax_volts)
    {
        *pv = -h->cfg.vmax_volts;
        saturated = 1u;
    }

    if (*pw > h->cfg.vmax_volts)
    {
        *pw = h->cfg.vmax_volts;
        saturated = 1u;
    }
    else if (*pw < -h->cfg.vmax_volts)
    {
        *pw = -h->cfg.vmax_volts;
        saturated = 1u;
    }

    return saturated;
}

static void DVP_UpdateOscillators(DynVoltPredictor3P *h)
{
    uint32_t m;

    if (h == 0)
    {
        return;
    }

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float c_old;
        float s_old;
        float c_new;
        float s_new;

        c_old = h->osc_cos[m];
        s_old = h->osc_sin[m];

        c_new = (c_old * h->step_cos[m]) - (s_old * h->step_sin[m]);
        s_new = (s_old * h->step_cos[m]) + (c_old * h->step_sin[m]);

        if ((h->sample_counter & 0x7Fu) == 0u)
        {
            float norm_corr;

            norm_corr = 1.5f - 0.5f * ((c_new * c_new) + (s_new * s_new));
            c_new *= norm_corr;
            s_new *= norm_corr;
        }

        h->osc_cos[m] = c_new;
        h->osc_sin[m] = s_new;
    }
}

void DynVoltPredictor3P_Step(DynVoltPredictor3P *h,
                             float ea,
                             float eb,
                             float ec,
                             float *ea_pred,
                             float *eb_pred,
                             float *ec_pred)
{
    float pred_u;
    float pred_v;
    float pred_w;
    uint8_t saturated;

    if ((h == 0) || (h->initialized == 0u))
    {
        if (ea_pred != 0)
        {
            *ea_pred = ea;
        }

        if (eb_pred != 0)
        {
            *eb_pred = eb;
        }

        if (ec_pred != 0)
        {
            *ec_pred = ec;
        }

        return;
    }

    h->sample_counter++;

    DVP_UpdateSupervisor(h, ea, eb, ec);
    DVP_DelayedSelfEvaluation(h, ea);

    DVP_UpdateRlsAndPredict(h, ea, &pred_u, &pred_v, &pred_w);
    DVP_ApplyAuthorityGate(h, ea, eb, ec, &pred_u, &pred_v, &pred_w);

    saturated = DVP_ApplySaturation(h, &pred_u, &pred_v, &pred_w);

    h->pred_buf_u[h->buf_index] = pred_u;
    h->sat_buf[h->buf_index] = saturated;

    h->buf_index++;

    if (h->buf_index >= DYNVOLT_PRED_DELAY_BUF_LEN)
    {
        h->buf_index = 0u;
    }

    h->dbg.sample_counter = h->sample_counter;
    h->dbg.delay_samples = h->delay_samples;
    h->dbg.warmup_samples = h->warmup_samples;

    h->dbg.kout_state = h->kout_state;
    h->dbg.kres_state = h->kres_state;
    h->dbg.tcal_state = h->tcal_state;
    h->dbg.gamma_state = h->gamma_state;

    h->dbg.flicker_score = h->flicker_score;
    h->dbg.alpha_pred = h->alpha_pred_state;

    h->dbg.e_rms_env = h->e_rms_env;
    h->dbg.e_max_env = h->e_max_env;
    h->dbg.de_rms_env = h->de_rms_env;

    h->dbg.ea_pred = pred_u;
    h->dbg.eb_pred = pred_v;
    h->dbg.ec_pred = pred_w;

    h->dbg.saturated = saturated;
    h->dbg.active = (h->alpha_pred_state > 0.15f) ? 1u : 0u;

    if (ea_pred != 0)
    {
        *ea_pred = pred_u;
    }

    if (eb_pred != 0)
    {
        *eb_pred = pred_v;
    }

    if (ec_pred != 0)
    {
        *ec_pred = pred_w;
    }

    DVP_UpdateOscillators(h);
}

const DynVoltPredictor3P_Debug *DynVoltPredictor3P_GetDebug(const DynVoltPredictor3P *h)
{
    if (h == 0)
    {
        return 0;
    }

    return &h->dbg;
}
