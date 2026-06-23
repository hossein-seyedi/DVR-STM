#include "dynvolt_predictor.h"
#include <math.h>
#include <string.h>

#define DVP_PI_F                         3.14159265358979323846f
#define DVP_TWO_PI_F                     (2.0f * DVP_PI_F)
#define DVP_DELTA_REBUILD_THRESHOLD_SEC  0.5e-6f

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
    return sqrtf(((a * a) + (b * b) + (c * c)) * 0.3333333333333333f);
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

    cfg->ts_sec = 50.0e-6f;
    cfg->td_sec = 190.0e-6f;

    cfg->f_start_hz = 25.0f;
    cfg->df_hz = 1.0f;

    cfg->vmax_volts = 80.0f;
    cfg->warmup_sec = 30.0e-3f;

    cfg->p_init = 100.0f;
    cfg->p_min = 1.0e-8f;
    cfg->p_max = 500.0f;

    cfg->enable_predictor = 1u;
    cfg->enable_adaptation = 1u;
}

static void DVP_ValidateConfig(DynVoltPredictor3P_Config *cfg)
{
    if (cfg == 0)
    {
        return;
    }

    if (cfg->ts_sec <= 0.0f)
    {
        cfg->ts_sec = 50.0e-6f;
    }

    if (cfg->td_sec < 0.0f)
    {
        cfg->td_sec = 0.0f;
    }

    if (cfg->f_start_hz <= 0.0f)
    {
        cfg->f_start_hz = 25.0f;
    }

    if (cfg->df_hz <= 0.0f)
    {
        cfg->df_hz = 1.0f;
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
        float f_m;
        float w_m;
        float angle;

        f_m = h->cfg.f_start_hz + ((float)m * h->cfg.df_hz);
        w_m = DVP_TWO_PI_F * f_m;
        angle = w_m * td_pred;

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
    uint32_t ph;
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

    h->kout_state = 1.5f;
    h->kres_state = 0.20f;
    h->tcal_state = 1.0e-6f;
    h->gamma_state = 0.20f;

    h->flicker_score = 0.0f;
    h->alpha_pred_state = 0.0f;

    for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
    {
        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            h->x[ph][i] = 0.0f;
            h->p[ph][i] = h->cfg.p_init;
        }
    }

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float f_m;
        float w_m;
        float step_angle;

        f_m = h->cfg.f_start_hz + ((float)m * h->cfg.df_hz);
        w_m = DVP_TWO_PI_F * f_m;
        step_angle = w_m * h->cfg.ts_sec;

        h->osc_cos[m] = 1.0f;
        h->osc_sin[m] = 0.0f;

        h->step_cos[m] = cosf(step_angle);
        h->step_sin[m] = sinf(step_angle);
    }

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

static void DVP_UpdateDerivative(DynVoltPredictor3P *h,
                                 const float e_abc[DYNVOLT_PRED_PHASE_COUNT])
{
    uint32_t ph;
    float alpha_de;
    float de_raw;

    alpha_de = h->cfg.ts_sec / (200.0e-6f + h->cfg.ts_sec);
    alpha_de = DVP_ClampF32(alpha_de, 0.0f, 1.0f);

    for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
    {
        if (h->sample_counter > 1u)
        {
            de_raw = (e_abc[ph] - h->e_prev[ph]) / h->cfg.ts_sec;
        }
        else
        {
            de_raw = 0.0f;
        }

        h->de_filt[ph] = h->de_filt[ph] + alpha_de * (de_raw - h->de_filt[ph]);
        h->e_prev[ph] = e_abc[ph];
    }
}

static void DVP_UpdateFlickerSupervisor(DynVoltPredictor3P *h,
                                        const float e_abc[DYNVOLT_PRED_PHASE_COUNT])
{
    float e_rms_now;
    float e_max_now;
    float de_rms_now;
    float alpha_env;
    float target_score;
    float tau_flicker;
    float alpha_flicker;

    uint8_t flicker_on_candidate;
    uint8_t no_flicker_candidate;

    e_rms_now = DVP_Rms3F32(e_abc[0], e_abc[1], e_abc[2]);
    e_max_now = DVP_Max3AbsF32(e_abc[0], e_abc[1], e_abc[2]);
    de_rms_now = DVP_Rms3F32(h->de_filt[0], h->de_filt[1], h->de_filt[2]);

    alpha_env = h->cfg.ts_sec / (2.0e-3f + h->cfg.ts_sec);
    alpha_env = DVP_ClampF32(alpha_env, 0.0f, 1.0f);

    h->e_rms_env = h->e_rms_env + alpha_env * (e_rms_now - h->e_rms_env);
    h->e_max_env = h->e_max_env + alpha_env * (e_max_now - h->e_max_env);
    h->de_rms_env = h->de_rms_env + alpha_env * (de_rms_now - h->de_rms_env);

    flicker_on_candidate = 0u;

    if ((h->e_max_env > 5.5f) && (h->e_rms_env > 3.5f))
    {
        flicker_on_candidate = 1u;
    }

    if ((h->e_max_env > (0.80f * 5.5f)) && (h->de_rms_env > 1200.0f))
    {
        flicker_on_candidate = 1u;
    }

    no_flicker_candidate = 0u;

    if ((h->e_max_env < 3.5f) &&
        (h->e_rms_env < 2.2f) &&
        (h->de_rms_env < 600.0f))
    {
        no_flicker_candidate = 1u;
    }

    if (flicker_on_candidate != 0u)
    {
        target_score = 1.0f;
    }
    else if (no_flicker_candidate != 0u)
    {
        target_score = 0.0f;
    }
    else
    {
        target_score = h->flicker_score;
    }

    if (target_score > h->flicker_score)
    {
        tau_flicker = 0.8e-3f;
    }
    else
    {
        tau_flicker = 25.0e-3f;
    }

    alpha_flicker = h->cfg.ts_sec / (tau_flicker + h->cfg.ts_sec);
    alpha_flicker = DVP_ClampF32(alpha_flicker, 0.0f, 1.0f);

    h->flicker_score = h->flicker_score +
                       alpha_flicker * (target_score - h->flicker_score);

    h->flicker_score = DVP_ClampF32(h->flicker_score, 0.0f, 1.0f);
}

static void DVP_ApplyParameterBounds(DynVoltPredictor3P *h)
{
    float tcal_bound;

    h->kout_state = DVP_ClampF32(h->kout_state, 0.75f, 1.30f);
    h->kres_state = DVP_ClampF32(h->kres_state, 0.0f, 0.55f);

    tcal_bound = 0.50f * h->cfg.td_sec;

    if (tcal_bound < (2.0f * h->cfg.ts_sec))
    {
        tcal_bound = 2.0f * h->cfg.ts_sec;
    }

    if (tcal_bound > 250.0e-6f)
    {
        tcal_bound = 250.0e-6f;
    }

    h->tcal_state = DVP_ClampF32(h->tcal_state, -tcal_bound, tcal_bound);
}

static void DVP_DelayedSelfEvaluation(DynVoltPredictor3P *h,
                                      const float e_abc[DYNVOLT_PRED_PHASE_COUNT])
{
    uint16_t due_idx;
    uint32_t ph;

    float due_pred[DYNVOLT_PRED_PHASE_COUNT];
    float due_kal[DYNVOLT_PRED_PHASE_COUNT];
    float due_res[DYNVOLT_PRED_PHASE_COUNT];
    float eval_vec[DYNVOLT_PRED_PHASE_COUNT];

    float sum_eval2;
    float sum_y2;
    float sum_p2;

    float alpha_perf;
    float eval_rms;
    float y_rms;
    float pred_rms;
    float norm_signal_rms;
    float rel_err;

    float real_signal_max;
    float real_signal_level;
    uint8_t saturated_due;

    float score_abs;
    float score_rel;
    float target_gamma;
    float alpha_meta;

    uint8_t adapt_active;

    h->dbg.valid_delay = 0u;
    h->dbg.adapt_active = 0u;

    due_idx = h->buf_index;

    if (due_idx >= h->delay_samples)
    {
        due_idx = (uint16_t)(due_idx - h->delay_samples);
    }
    else
    {
        due_idx = (uint16_t)(DYNVOLT_PRED_DELAY_BUF_LEN + due_idx - h->delay_samples);
    }

    if (h->sample_counter <= ((uint32_t)h->warmup_samples + (uint32_t)h->delay_samples + 5u))
    {
        return;
    }

    h->dbg.valid_delay = 1u;

    sum_eval2 = 0.0f;
    sum_y2 = 0.0f;
    sum_p2 = 0.0f;

    for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
    {
        due_pred[ph] = h->pred_buf[ph][due_idx];
        due_kal[ph] = h->kal_buf[ph][due_idx];
        due_res[ph] = h->res_buf[ph][due_idx];

        eval_vec[ph] = e_abc[ph] - due_pred[ph];

        sum_eval2 += eval_vec[ph] * eval_vec[ph];
        sum_y2 += e_abc[ph] * e_abc[ph];
        sum_p2 += due_pred[ph] * due_pred[ph];
    }

    alpha_perf = h->cfg.ts_sec / (3.0e-3f + h->cfg.ts_sec);

    if (alpha_perf > 0.05f)
    {
        alpha_perf = 0.05f;
    }

    h->env_eval2 = h->env_eval2 + alpha_perf * ((sum_eval2 * 0.3333333333333333f) - h->env_eval2);
    h->env_y2 = h->env_y2 + alpha_perf * ((sum_y2 * 0.3333333333333333f) - h->env_y2);
    h->env_pred2 = h->env_pred2 + alpha_perf * ((sum_p2 * 0.3333333333333333f) - h->env_pred2);

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

    norm_signal_rms = y_rms;

    if (pred_rms > norm_signal_rms)
    {
        norm_signal_rms = pred_rms;
    }

    rel_err = eval_rms / (norm_signal_rms + 0.5f);

    real_signal_max = DVP_Max3AbsF32(e_abc[0], e_abc[1], e_abc[2]);
    real_signal_level = y_rms;

    if (real_signal_max > real_signal_level)
    {
        real_signal_level = real_signal_max;
    }

    saturated_due = h->sat_buf[due_idx];

    adapt_active = 0u;

    if ((real_signal_level > 1.0f) &&
        (saturated_due == 0u) &&
        (eval_rms > 0.50f) &&
        (rel_err > 0.030f) &&
        (h->flicker_score >= 0.15f) &&
        (h->cfg.enable_adaptation != 0u))
    {
        adapt_active = 1u;
    }

    score_abs = (eval_rms - 0.50f) / (5.0f - 0.50f);
    score_rel = (rel_err - 0.030f) / (0.15f - 0.030f);

    score_abs = DVP_ClampF32(score_abs, 0.0f, 1.0f);
    score_rel = DVP_ClampF32(score_rel, 0.0f, 1.0f);

    target_gamma = score_abs;

    if (score_rel > target_gamma)
    {
        target_gamma = score_rel;
    }

    if ((real_signal_level <= 1.0f) ||
        (saturated_due != 0u) ||
        (h->flicker_score < 0.15f))
    {
        target_gamma = 0.0f;
    }

    if (real_signal_level <= 1.0f)
    {
        alpha_meta = h->cfg.ts_sec / (0.5e-3f + h->cfg.ts_sec);
    }
    else
    {
        alpha_meta = h->cfg.ts_sec / (2.0e-3f + h->cfg.ts_sec);
    }

    if (alpha_meta > 0.02f)
    {
        alpha_meta = 0.02f;
    }

    h->gamma_state = h->gamma_state + alpha_meta * (target_gamma - h->gamma_state);
    h->gamma_state = DVP_ClampF32(h->gamma_state, 0.0f, 1.0f);

    if (adapt_active != 0u)
    {
        float num_gain;
        float den_gain;
        float num_res;
        float den_res;
        float num_time;
        float den_time;
        float d_kout;
        float d_kres;
        float d_t;

        num_gain = 0.0f;
        den_gain = 0.0f;
        num_res = 0.0f;
        den_res = 0.0f;
        num_time = 0.0f;
        den_time = 0.0f;

        for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
        {
            num_gain += eval_vec[ph] * due_kal[ph];
            den_gain += due_kal[ph] * due_kal[ph];

            num_res += eval_vec[ph] * due_res[ph];
            den_res += due_res[ph] * due_res[ph];

            num_time += eval_vec[ph] * h->de_filt[ph];
            den_time += h->de_filt[ph] * h->de_filt[ph];
        }

        if (den_gain > 1.0e-6f)
        {
            d_kout = num_gain / (den_gain + 1.0e-9f);
            d_kout = DVP_ClampF32(d_kout, -0.20f, 0.20f);
            h->kout_state = h->kout_state + alpha_meta * 0.45f * d_kout;
        }

        if (den_res > 1.0e-6f)
        {
            d_kres = num_res / (den_res + 1.0e-9f);
            d_kres = DVP_ClampF32(d_kres, -0.25f, 0.25f);
            h->kres_state = h->kres_state + alpha_meta * 0.35f * d_kres;
        }

        if (den_time > 1.0e5f)
        {
            d_t = num_time / (den_time + 1.0e-9f);
            d_t = DVP_ClampF32(d_t, -80.0e-6f, 80.0e-6f);
            h->tcal_state = h->tcal_state + alpha_meta * 0.60f * d_t;
        }
    }

    DVP_ApplyParameterBounds(h);

    h->dbg.eval_rms = eval_rms;
    h->dbg.signal_rms = y_rms;
    h->dbg.pred_rms = pred_rms;
    h->dbg.rel_err = rel_err;
    h->dbg.adapt_active = adapt_active;
}

static void DVP_BuildBasis(DynVoltPredictor3P *h, float td_pred)
{
    uint32_t m;
    uint32_t col;

    DVP_UpdateFutureDeltaIfNeeded(h, td_pred);

    h->basis_now[0] = 1.0f;
    h->basis_fut[0] = 1.0f;

    col = 1u;

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float c_now;
        float s_now;
        float c_delta;
        float s_delta;
        float c_fut;
        float s_fut;

        c_now = h->osc_cos[m];
        s_now = h->osc_sin[m];

        c_delta = h->fut_cos[m];
        s_delta = h->fut_sin[m];

        c_fut = (c_now * c_delta) - (s_now * s_delta);
        s_fut = (s_now * c_delta) + (c_now * s_delta);

        h->basis_now[col] = c_now;
        h->basis_now[col + 1u] = s_now;

        h->basis_fut[col] = c_fut;
        h->basis_fut[col + 1u] = s_fut;

        col += 2u;
    }
}

static void DVP_BuildBasisAndPredict(DynVoltPredictor3P *h,
                                     const float e_abc[DYNVOLT_PRED_PHASE_COUNT],
                                     float pred_abc[DYNVOLT_PRED_PHASE_COUNT],
                                     float kal_abc[DYNVOLT_PRED_PHASE_COUNT],
                                     float res_abc[DYNVOLT_PRED_PHASE_COUNT])
{
    float r_meas;
    float q_h;
    float q_dc;
    float tau_res;
    float tau_der;
    float alpha_res;
    float alpha_der;
    float kres_eff;
    float kout_eff;
    float td_pred;
    float res_limit;

    uint32_t ph;
    uint32_t i;

    r_meas = 4.0f + h->gamma_state * (1.0f - 4.0f);

    q_h = 2.0e-4f + h->gamma_state * (2.0e-3f - 2.0e-4f);
    q_dc = 1.0e-6f + h->gamma_state * (5.0e-6f - 1.0e-6f);

    tau_res = 600.0e-6f + h->gamma_state * (180.0e-6f - 600.0e-6f);
    tau_der = 900.0e-6f + h->gamma_state * (250.0e-6f - 900.0e-6f);

    alpha_res = h->cfg.ts_sec / (tau_res + h->cfg.ts_sec);
    alpha_der = h->cfg.ts_sec / (tau_der + h->cfg.ts_sec);

    alpha_res = DVP_ClampF32(alpha_res, 0.0f, 1.0f);
    alpha_der = DVP_ClampF32(alpha_der, 0.0f, 1.0f);

    kres_eff = h->kres_state + 0.25f * h->gamma_state;
    kres_eff = DVP_ClampF32(kres_eff, 0.0f, 0.75f);

    if (h->flicker_score < 0.10f)
    {
        kres_eff = 0.0f;
    }
    else if (h->flicker_score < 0.30f)
    {
        kres_eff = kres_eff * (h->flicker_score / 0.30f);
    }

    kout_eff = h->kout_state;

    td_pred = h->cfg.td_sec + h->tcal_state;

    if (td_pred < 0.0f)
    {
        td_pred = 0.0f;
    }

    DVP_BuildBasis(h, td_pred);

    res_limit = (0.20f + 0.20f * h->gamma_state) * h->cfg.vmax_volts;

    for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
    {
        float y;
        float e_hat_before;
        float innovation;
        float s_val;
        float inv_s;
        float e_kalman_future;
        float residual;
        float r_old;
        float dr_raw;
        float r_future;

        y = e_abc[ph];

        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            if (i == 0u)
            {
                h->p_work[i] = h->p[ph][i] + q_dc;
            }
            else
            {
                h->p_work[i] = h->p[ph][i] + q_h;
            }

            h->p_work[i] = DVP_ClampF32(h->p_work[i], h->cfg.p_min, h->cfg.p_max);
        }

        e_hat_before = 0.0f;

        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            e_hat_before += h->basis_now[i] * h->x[ph][i];
        }

        innovation = y - e_hat_before;

        s_val = r_meas;

        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            s_val += h->basis_now[i] * h->basis_now[i] * h->p_work[i];
        }

        if (s_val < 1.0e-12f)
        {
            s_val = 1.0e-12f;
        }

        inv_s = 1.0f / s_val;

        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            float k;
            float p_new;

            k = h->p_work[i] * h->basis_now[i] * inv_s;

            h->x[ph][i] = h->x[ph][i] + k * innovation;

            p_new = (1.0f - k * h->basis_now[i]) * h->p_work[i];
            h->p[ph][i] = DVP_ClampF32(p_new, h->cfg.p_min, h->cfg.p_max);
        }

        e_kalman_future = 0.0f;

        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            e_kalman_future += h->basis_fut[i] * h->x[ph][i];
        }

        residual = innovation;

        r_old = h->r_filt[ph];
        h->r_filt[ph] = h->r_filt[ph] + alpha_res * (residual - h->r_filt[ph]);

        dr_raw = (h->r_filt[ph] - r_old) / h->cfg.ts_sec;
        h->dr_filt[ph] = h->dr_filt[ph] + alpha_der * (dr_raw - h->dr_filt[ph]);

        r_future = h->r_filt[ph] + td_pred * h->dr_filt[ph];
        r_future = DVP_ClampF32(r_future, -res_limit, res_limit);

        kal_abc[ph] = e_kalman_future;
        res_abc[ph] = r_future;

        pred_abc[ph] = kout_eff * e_kalman_future + kres_eff * r_future;
    }
}

static void DVP_ApplyAuthorityGate(DynVoltPredictor3P *h,
                                   const float e_abc[DYNVOLT_PRED_PHASE_COUNT],
                                   float pred_abc[DYNVOLT_PRED_PHASE_COUNT])
{
    float alpha_target;
    float tau_alpha;
    float alpha_pred;
    uint32_t ph;

    alpha_target = h->flicker_score;

    if (alpha_target < 0.05f)
    {
        alpha_target = 0.0f;
    }

    if (alpha_target > 0.98f)
    {
        alpha_target = 1.0f;
    }

    if (h->cfg.enable_predictor == 0u)
    {
        alpha_target = 0.0f;
    }

    if (alpha_target > h->alpha_pred_state)
    {
        tau_alpha = 1.0e-3f;
    }
    else
    {
        tau_alpha = 4.0e-3f;
    }

    alpha_pred = h->cfg.ts_sec / (tau_alpha + h->cfg.ts_sec);
    alpha_pred = DVP_ClampF32(alpha_pred, 0.0f, 1.0f);

    h->alpha_pred_state = h->alpha_pred_state +
                          alpha_pred * (alpha_target - h->alpha_pred_state);

    h->alpha_pred_state = DVP_ClampF32(h->alpha_pred_state, 0.0f, 1.0f);

    if ((h->e_max_env < 3.5f) &&
        (h->e_rms_env < 2.2f) &&
        (h->de_rms_env < 600.0f))
    {
        if (h->flicker_score < 0.05f)
        {
            h->alpha_pred_state = 0.0f;
        }
    }

    for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
    {
        pred_abc[ph] = e_abc[ph] + h->alpha_pred_state * (pred_abc[ph] - e_abc[ph]);
    }
}

static uint8_t DVP_ApplySaturation(DynVoltPredictor3P *h,
                                   float pred_abc[DYNVOLT_PRED_PHASE_COUNT])
{
    uint32_t ph;
    uint8_t saturated;

    saturated = 0u;

    for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
    {
        if (pred_abc[ph] > h->cfg.vmax_volts)
        {
            pred_abc[ph] = h->cfg.vmax_volts;
            saturated = 1u;
        }
        else if (pred_abc[ph] < -h->cfg.vmax_volts)
        {
            pred_abc[ph] = -h->cfg.vmax_volts;
            saturated = 1u;
        }
    }

    return saturated;
}

static void DVP_UpdateOscillators(DynVoltPredictor3P *h)
{
    uint32_t m;

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
    float e_abc[DYNVOLT_PRED_PHASE_COUNT];
    float pred_abc[DYNVOLT_PRED_PHASE_COUNT];
    float kal_abc[DYNVOLT_PRED_PHASE_COUNT];
    float res_abc[DYNVOLT_PRED_PHASE_COUNT];

    float warm_gain;
    uint32_t ph;
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

    e_abc[0] = ea;
    e_abc[1] = eb;
    e_abc[2] = ec;

    DVP_UpdateDerivative(h, e_abc);
    DVP_UpdateFlickerSupervisor(h, e_abc);
    DVP_DelayedSelfEvaluation(h, e_abc);

    DVP_BuildBasisAndPredict(h, e_abc, pred_abc, kal_abc, res_abc);

    DVP_ApplyAuthorityGate(h, e_abc, pred_abc);

    saturated = DVP_ApplySaturation(h, pred_abc);

    if (h->sample_counter < (uint32_t)h->warmup_samples)
    {
        warm_gain = (float)h->sample_counter / (float)h->warmup_samples;
    }
    else
    {
        warm_gain = 1.0f;
    }

    for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
    {
        pred_abc[ph] *= warm_gain;
    }

    for (ph = 0u; ph < DYNVOLT_PRED_PHASE_COUNT; ph++)
    {
        h->pred_buf[ph][h->buf_index] = pred_abc[ph];
        h->kal_buf[ph][h->buf_index] = kal_abc[ph];
        h->res_buf[ph][h->buf_index] = res_abc[ph];
    }

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

    h->dbg.ea_pred = pred_abc[0];
    h->dbg.eb_pred = pred_abc[1];
    h->dbg.ec_pred = pred_abc[2];

    h->dbg.saturated = saturated;
    h->dbg.active = (h->flicker_score > 0.15f) ? 1u : 0u;

    if (ea_pred != 0)
    {
        *ea_pred = pred_abc[0];
    }

    if (eb_pred != 0)
    {
        *eb_pred = pred_abc[1];
    }

    if (ec_pred != 0)
    {
        *ec_pred = pred_abc[2];
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
