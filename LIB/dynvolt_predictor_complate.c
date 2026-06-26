#include "dynvolt_predictor.h"

#include <math.h>
#include <stddef.h>

static float dv_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float dv_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

static float dv_clampf(float x, float xmin, float xmax)
{
    if (x < xmin)
    {
        return xmin;
    }

    if (x > xmax)
    {
        return xmax;
    }

    return x;
}

static uint16_t dv_round_to_u16_clamped(float x, uint16_t xmin, uint16_t xmax)
{
    uint32_t y;

    if (x <= 0.0f)
    {
        y = 0u;
    }
    else
    {
        y = (uint32_t)(x + 0.5f);
    }

    if (y < (uint32_t)xmin)
    {
        y = (uint32_t)xmin;
    }

    if (y > (uint32_t)xmax)
    {
        y = (uint32_t)xmax;
    }

    return (uint16_t)y;
}

static void dv_clear_debug(DynVoltPredictor3P_Debug *dbg)
{
    if (dbg == NULL)
    {
        return;
    }

    dbg->sample_counter = 0u;
    dbg->delay_samples = 0u;
    dbg->warmup_samples = 0u;

    dbg->eval_rms = 0.0f;
    dbg->signal_rms = 0.0f;
    dbg->pred_rms = 0.0f;
    dbg->rel_err = 0.0f;

    dbg->kout_state = 0.0f;
    dbg->kout_eff = 0.0f;
    dbg->kres_state = 0.0f;
    dbg->kres_eff = 0.0f;
    dbg->tcal_state = 0.0f;
    dbg->td_pred_sec = 0.0f;
    dbg->gamma_state = 0.0f;

    dbg->R_meas = 0.0f;
    dbg->Q_h = 0.0f;
    dbg->Q_dc = 0.0f;
    dbg->tau_res = 0.0f;
    dbg->tau_der = 0.0f;

    dbg->flicker_score = 0.0f;
    dbg->quiet_score = 1.0f;
    dbg->alpha_pred = 0.0f;

    dbg->e_rms_env = 0.0f;
    dbg->e_max_env = 0.0f;
    dbg->de_rms_env = 0.0f;
    dbg->de_max_env = 0.0f;

    dbg->ea_pred = 0.0f;
    dbg->eb_pred = 0.0f;
    dbg->ec_pred = 0.0f;

    dbg->ea_kal_future = 0.0f;
    dbg->eb_kal_future = 0.0f;
    dbg->ec_kal_future = 0.0f;

    dbg->ea_res_future = 0.0f;
    dbg->eb_res_future = 0.0f;
    dbg->ec_res_future = 0.0f;

    dbg->ea_innovation = 0.0f;
    dbg->eb_innovation = 0.0f;
    dbg->ec_innovation = 0.0f;

    dbg->ea_hat_before = 0.0f;
    dbg->eb_hat_before = 0.0f;
    dbg->ec_hat_before = 0.0f;

    dbg->valid_delay = 0u;
    dbg->adapt_active = 0u;
    dbg->saturated = 0u;
    dbg->active = 0u;
}

void DynVoltPredictor3P_GetDefaultConfig(DynVoltPredictor3P_Config *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->ts_sec = 0.0005f;
    cfg->td_sec = 0.0010f;

    cfg->f_start_hz = 25.0f;
    cfg->df_hz = 1.0f;

    cfg->vmax_volts = 80.0f;
    cfg->warmup_sec = 0.03f;

    cfg->p_init = 100.0f;
    cfg->p_min = 1.0e-8f;
    cfg->p_max = 500.0f;

    cfg->enable_predictor = 1u;
    cfg->enable_adaptation = 1u;

    cfg->grid_hz = 50.0f;
    cfg->flicker_hz = 8.8028169f;
    cfg->fine_df_hz = 0.25f;
    cfg->phase_v_shift_rad = -2.0f * DYNVOLT_PRED_PI_F / 3.0f;
    cfg->phase_w_shift_rad =  2.0f * DYNVOLT_PRED_PI_F / 3.0f;
}

void DynVoltPredictor3P_Reset(DynVoltPredictor3P *h)
{
    uint32_t phase;
    uint32_t i;
    uint32_t m;
    uint32_t k;
    float p_init;

    if (h == NULL)
    {
        return;
    }

    p_init = h->cfg.p_init;

    for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
    {
        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            h->x[phase][i] = 0.0f;
            h->p[phase][i] = p_init;
        }

        h->r_filt[phase] = 0.0f;
        h->dr_filt[phase] = 0.0f;
        h->e_prev[phase] = 0.0f;
        h->de_filt[phase] = 0.0f;

        for (k = 0u; k < DYNVOLT_PRED_DELAY_BUF_LEN; k++)
        {
            h->pred_buf[phase][k] = 0.0f;
            h->kal_buf[phase][k] = 0.0f;
            h->res_buf[phase][k] = 0.0f;
        }
    }

    for (k = 0u; k < DYNVOLT_PRED_DELAY_BUF_LEN; k++)
    {
        h->sat_buf[k] = 0u;
    }

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        h->theta[m] = 0.0f;
        h->freq_hz[m] = h->cfg.f_start_hz + ((float)m * h->cfg.df_hz);
    }

    h->buf_index = 0u;
    h->delay_samples = 1u;
    h->warmup_samples = 1u;

    h->kout_state = 1.5f;
    h->kres_state = 0.20f;
    h->tcal_state = 1.0e-6f;
    h->gamma_state = 0.20f;

    h->env_eval2 = 0.0f;
    h->env_y2 = 0.0f;
    h->env_pred2 = 0.0f;

    h->quiet_score = 1.0f;
    h->e_rms_env = 0.0f;
    h->e_max_env = 0.0f;
    h->de_rms_env = 0.0f;
    h->de_max_env = 0.0f;
    h->flicker_score = 0.0f;
    h->alpha_pred_state = 0.0f;

    h->sample_counter = 0u;
    h->initialized = 1u;

    dv_clear_debug(&h->dbg);
}

void DynVoltPredictor3P_Init(DynVoltPredictor3P *h,
                             const DynVoltPredictor3P_Config *cfg)
{
    DynVoltPredictor3P_Config local_cfg;

    if (h == NULL)
    {
        return;
    }

    if (cfg == NULL)
    {
        DynVoltPredictor3P_GetDefaultConfig(&local_cfg);
    }
    else
    {
        local_cfg = *cfg;
    }

    if (local_cfg.ts_sec <= 0.0f)
    {
        local_cfg.ts_sec = 2.0e-6f;
    }

    if (local_cfg.td_sec < 0.0f)
    {
        local_cfg.td_sec = 0.0f;
    }

    if (local_cfg.f_start_hz <= 0.0f)
    {
        local_cfg.f_start_hz = 25.0f;
    }

    if (local_cfg.df_hz <= 0.0f)
    {
        local_cfg.df_hz = 1.0f;
    }

    if (local_cfg.vmax_volts <= 0.0f)
    {
        local_cfg.vmax_volts = 80.0f;
    }

    if (local_cfg.warmup_sec <= 0.0f)
    {
        local_cfg.warmup_sec = 0.03f;
    }

    if (local_cfg.p_init <= 0.0f)
    {
        local_cfg.p_init = 100.0f;
    }

    if (local_cfg.p_min <= 0.0f)
    {
        local_cfg.p_min = 1.0e-8f;
    }

    if (local_cfg.p_max <= local_cfg.p_min)
    {
        local_cfg.p_max = 500.0f;
    }

    h->cfg = local_cfg;
    DynVoltPredictor3P_Reset(h);
}

void DynVoltPredictor3P_Step(DynVoltPredictor3P *h,
                             float ea,
                             float eb,
                             float ec,
                             float *ea_pred,
                             float *eb_pred,
                             float *ec_pred)
{
    const float two_pi = 2.0f * DYNVOLT_PRED_PI_F;

    float Ts_eff;
    float Td_eff;
    float e_abc[DYNVOLT_PRED_PHASE_COUNT];
    float de_raw[DYNVOLT_PRED_PHASE_COUNT];
    float due_pred[DYNVOLT_PRED_PHASE_COUNT];
    float due_kal[DYNVOLT_PRED_PHASE_COUNT];
    float due_res[DYNVOLT_PRED_PHASE_COUNT];
    float eval_vec[DYNVOLT_PRED_PHASE_COUNT];
    float e_pred_abc[DYNVOLT_PRED_PHASE_COUNT];
    float kal_future_abc[DYNVOLT_PRED_PHASE_COUNT];
    float res_future_abc[DYNVOLT_PRED_PHASE_COUNT];
    float innovation_dbg[DYNVOLT_PRED_PHASE_COUNT];
    float e_hat_before_dbg[DYNVOLT_PRED_PHASE_COUNT];

    float h_now[DYNVOLT_PRED_STATE_COUNT];
    float h_fut[DYNVOLT_PRED_STATE_COUNT];
    float P_pred[DYNVOLT_PRED_STATE_COUNT];

    uint32_t sample_counter;
    uint16_t N_warm;
    uint16_t D_check;
    uint16_t due_idx;
    uint32_t phase;
    uint32_t i;
    uint32_t m;
    uint32_t col;

    float tau_de;
    float alpha_de;
    float e_rms_now;
    float e_max_now;
    float de_rms_now;
    float de_max_now;
    float tau_env;
    float alpha_env;
    float F_on_max;
    float F_on_rms;
    float F_off_max;
    float F_off_rms;
    float DE_on_rms;
    float DE_off_rms;
    uint8_t flicker_on_candidate;
    uint8_t no_flicker_candidate;
    float target_flicker_score;
    float tau_flicker;
    float alpha_flicker;

    uint8_t valid_delay;
    uint8_t adapt_active;
    uint8_t saturated_due;
    float sum_eval2;
    float sum_y2;
    float sum_p2;
    float mean_eval2;
    float mean_y2;
    float mean_p2;
    float tau_perf;
    float alpha_perf;
    float eval_rms;
    float y_rms;
    float pred_rms;
    float real_signal_rms;
    float real_signal_max;
    float real_signal_level;
    float norm_signal_rms;
    float rel_err;
    float signal_min;
    float err_dead_abs;
    float err_dead_rel;
    float err_bad_abs;
    float err_bad_rel;
    float score_abs;
    float score_rel;
    float target_gamma;
    float tau_meta;
    float alpha_meta;
    float num_gain;
    float den_gain;
    float dKout;
    float num_res;
    float den_res;
    float dKres;
    float num_time;
    float den_time;
    float dT_est;
    float Tcal_bound;

    float R_slow;
    float R_fast;
    float Qh_slow;
    float Qh_fast;
    float Qdc_slow;
    float Qdc_fast;
    float tau_res_slow;
    float tau_res_fast;
    float tau_der_slow;
    float tau_der_fast;
    float R_meas;
    float Q_h;
    float Q_dc;
    float tau_res;
    float tau_der;
    float Kres_eff;
    float Kout_eff;
    float Td_pred;
    float alpha_res;
    float alpha_der;
    float res_limit;

    float y;
    float P_temp;
    float e_hat_before;
    float innovation;
    float S;
    float K;
    float P_new;
    float e_kalman_future;
    float residual;
    float r_old;
    float dr_raw;
    float r_future;

    float p_rms_now;
    float p_max_now;
    float alpha_target;
    float tau_alpha;
    float alpha_pred;
    float quiet_gate_global;
    float sat_flag;
    float warm_gain;

    if (h == NULL)
    {
        if (ea_pred != NULL) { *ea_pred = ea; }
        if (eb_pred != NULL) { *eb_pred = eb; }
        if (ec_pred != NULL) { *ec_pred = ec; }
        return;
    }

    if (h->initialized == 0u)
    {
        DynVoltPredictor3P_Config cfg;
        DynVoltPredictor3P_GetDefaultConfig(&cfg);
        DynVoltPredictor3P_Init(h, &cfg);
    }

    if (h->cfg.enable_predictor == 0u)
    {
        if (ea_pred != NULL) { *ea_pred = ea; }
        if (eb_pred != NULL) { *eb_pred = eb; }
        if (ec_pred != NULL) { *ec_pred = ec; }
        h->dbg.active = 0u;
        h->dbg.saturated = 0u;
        h->dbg.ea_pred = ea;
        h->dbg.eb_pred = eb;
        h->dbg.ec_pred = ec;
        return;
    }

    Ts_eff = h->cfg.ts_sec;
    if (Ts_eff <= 0.0f)
    {
        Ts_eff = 2.0e-6f;
    }

    Td_eff = h->cfg.td_sec;
    if (Td_eff < 0.0f)
    {
        Td_eff = 0.0f;
    }

    e_abc[0] = ea;
    e_abc[1] = eb;
    e_abc[2] = ec;

    for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
    {
        de_raw[phase] = 0.0f;
        due_pred[phase] = 0.0f;
        due_kal[phase] = 0.0f;
        due_res[phase] = 0.0f;
        eval_vec[phase] = 0.0f;
        e_pred_abc[phase] = 0.0f;
        kal_future_abc[phase] = 0.0f;
        res_future_abc[phase] = 0.0f;
        innovation_dbg[phase] = 0.0f;
        e_hat_before_dbg[phase] = 0.0f;
    }

    h->sample_counter++;
    sample_counter = h->sample_counter;

    N_warm = dv_round_to_u16_clamped(h->cfg.warmup_sec / Ts_eff,
                                     1u,
                                     (uint16_t)(DYNVOLT_PRED_DELAY_BUF_LEN - 1u));

    D_check = dv_round_to_u16_clamped(Td_eff / Ts_eff,
                                      1u,
                                      (uint16_t)(DYNVOLT_PRED_DELAY_BUF_LEN - 2u));

    h->warmup_samples = N_warm;
    h->delay_samples = D_check;

    tau_de = 200.0e-6f;
    alpha_de = Ts_eff / (tau_de + Ts_eff);
    alpha_de = dv_clampf(alpha_de, 0.0f, 1.0f);

    if (sample_counter > 1u)
    {
        for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
        {
            de_raw[phase] = (e_abc[phase] - h->e_prev[phase]) / Ts_eff;
        }
    }

    for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
    {
        h->de_filt[phase] += alpha_de * (de_raw[phase] - h->de_filt[phase]);
        h->e_prev[phase] = e_abc[phase];
    }

    e_rms_now = sqrtf((e_abc[0] * e_abc[0] + e_abc[1] * e_abc[1] + e_abc[2] * e_abc[2]) / 3.0f);
    e_max_now = dv_maxf(dv_absf(e_abc[0]), dv_maxf(dv_absf(e_abc[1]), dv_absf(e_abc[2])));

    de_rms_now = sqrtf((h->de_filt[0] * h->de_filt[0] + h->de_filt[1] * h->de_filt[1] + h->de_filt[2] * h->de_filt[2]) / 3.0f);
    de_max_now = dv_maxf(dv_absf(h->de_filt[0]), dv_maxf(dv_absf(h->de_filt[1]), dv_absf(h->de_filt[2])));

    tau_env = 2.0e-3f;
    alpha_env = Ts_eff / (tau_env + Ts_eff);
    alpha_env = dv_clampf(alpha_env, 0.0f, 1.0f);

    h->e_rms_env += alpha_env * (e_rms_now - h->e_rms_env);
    h->e_max_env += alpha_env * (e_max_now - h->e_max_env);
    h->de_rms_env += alpha_env * (de_rms_now - h->de_rms_env);
    h->de_max_env += alpha_env * (de_max_now - h->de_max_env);

    F_on_max  = 5.5f;
    F_on_rms  = 3.5f;
    F_off_max = 3.5f;
    F_off_rms = 2.2f;
    DE_on_rms = 1200.0f;
    DE_off_rms = 600.0f;

    flicker_on_candidate = 0u;
    if ((h->e_max_env > F_on_max) && (h->e_rms_env > F_on_rms))
    {
        flicker_on_candidate = 1u;
    }

    if ((h->e_max_env > (0.80f * F_on_max)) && (h->de_rms_env > DE_on_rms))
    {
        flicker_on_candidate = 1u;
    }

    no_flicker_candidate = 0u;
    if ((h->e_max_env < F_off_max) &&
        (h->e_rms_env < F_off_rms) &&
        (h->de_rms_env < DE_off_rms))
    {
        no_flicker_candidate = 1u;
    }

    if (flicker_on_candidate != 0u)
    {
        target_flicker_score = 1.0f;
    }
    else if (no_flicker_candidate != 0u)
    {
        target_flicker_score = 0.0f;
    }
    else
    {
        target_flicker_score = h->flicker_score;
    }

    if (target_flicker_score > h->flicker_score)
    {
        tau_flicker = 0.8e-3f;
    }
    else
    {
        tau_flicker = 25.0e-3f;
    }

    alpha_flicker = Ts_eff / (tau_flicker + Ts_eff);
    alpha_flicker = dv_clampf(alpha_flicker, 0.0f, 1.0f);

    h->flicker_score += alpha_flicker * (target_flicker_score - h->flicker_score);
    h->flicker_score = dv_clampf(h->flicker_score, 0.0f, 1.0f);
    h->quiet_score = 1.0f - h->flicker_score;

    due_idx = h->buf_index;
    if (due_idx >= D_check)
    {
        due_idx = (uint16_t)(due_idx - D_check);
    }
    else
    {
        due_idx = (uint16_t)(due_idx + DYNVOLT_PRED_DELAY_BUF_LEN - D_check);
    }

    valid_delay = 0u;
    eval_rms = 0.0f;
    y_rms = 0.0f;
    pred_rms = 0.0f;
    rel_err = 0.0f;
    real_signal_level = 0.0f;
    saturated_due = 0u;
    adapt_active = 0u;

    if (sample_counter > ((uint32_t)N_warm + (uint32_t)D_check + 5u))
    {
        valid_delay = 1u;

        for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
        {
            due_pred[phase] = h->pred_buf[phase][due_idx];
            due_kal[phase] = h->kal_buf[phase][due_idx];
            due_res[phase] = h->res_buf[phase][due_idx];
            eval_vec[phase] = e_abc[phase] - due_pred[phase];
        }

        sum_eval2 = 0.0f;
        sum_y2 = 0.0f;
        sum_p2 = 0.0f;

        for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
        {
            sum_eval2 += eval_vec[phase] * eval_vec[phase];
            sum_y2 += e_abc[phase] * e_abc[phase];
            sum_p2 += due_pred[phase] * due_pred[phase];
        }

        mean_eval2 = sum_eval2 / 3.0f;
        mean_y2 = sum_y2 / 3.0f;
        mean_p2 = sum_p2 / 3.0f;

        tau_perf = 3.0e-3f;
        alpha_perf = Ts_eff / (tau_perf + Ts_eff);
        if (alpha_perf > 0.05f)
        {
            alpha_perf = 0.05f;
        }

        h->env_eval2 += alpha_perf * (mean_eval2 - h->env_eval2);
        h->env_y2 += alpha_perf * (mean_y2 - h->env_y2);
        h->env_pred2 += alpha_perf * (mean_p2 - h->env_pred2);

        if (h->env_eval2 < 0.0f) { h->env_eval2 = 0.0f; }
        if (h->env_y2 < 0.0f) { h->env_y2 = 0.0f; }
        if (h->env_pred2 < 0.0f) { h->env_pred2 = 0.0f; }

        eval_rms = sqrtf(h->env_eval2);
        y_rms = sqrtf(h->env_y2);
        pred_rms = sqrtf(h->env_pred2);

        real_signal_rms = y_rms;
        real_signal_max = dv_maxf(dv_absf(e_abc[0]), dv_maxf(dv_absf(e_abc[1]), dv_absf(e_abc[2])));
        real_signal_level = dv_maxf(real_signal_rms, real_signal_max);
        norm_signal_rms = dv_maxf(y_rms, pred_rms);
        rel_err = eval_rms / (norm_signal_rms + 0.5f);

        signal_min = 1.0f;
        err_dead_abs = 0.50f;
        err_dead_rel = 0.030f;
        saturated_due = h->sat_buf[due_idx];

        if ((real_signal_level > signal_min) &&
            (saturated_due == 0u) &&
            (eval_rms > err_dead_abs) &&
            (rel_err > err_dead_rel))
        {
            adapt_active = 1u;
        }

        if (h->flicker_score < 0.15f)
        {
            adapt_active = 0u;
        }

        if (h->cfg.enable_adaptation == 0u)
        {
            adapt_active = 0u;
        }

        err_bad_abs = 5.0f;
        err_bad_rel = 0.15f;

        score_abs = (eval_rms - err_dead_abs) / (err_bad_abs - err_dead_abs);
        score_rel = (rel_err - err_dead_rel) / (err_bad_rel - err_dead_rel);
        score_abs = dv_clampf(score_abs, 0.0f, 1.0f);
        score_rel = dv_clampf(score_rel, 0.0f, 1.0f);

        target_gamma = dv_maxf(score_abs, score_rel);

        if ((real_signal_level <= signal_min) || (saturated_due != 0u) || (h->flicker_score < 0.15f))
        {
            target_gamma = 0.0f;
        }

        if (real_signal_level <= signal_min)
        {
            tau_meta = 0.5e-3f;
        }
        else
        {
            tau_meta = 2.0e-3f;
        }

        alpha_meta = Ts_eff / (tau_meta + Ts_eff);
        if (alpha_meta > 0.02f)
        {
            alpha_meta = 0.02f;
        }

        h->gamma_state += alpha_meta * (target_gamma - h->gamma_state);
        h->gamma_state = dv_clampf(h->gamma_state, 0.0f, 1.0f);

        if (adapt_active != 0u)
        {
            num_gain = 0.0f;
            den_gain = 0.0f;

            for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
            {
                num_gain += eval_vec[phase] * due_kal[phase];
                den_gain += due_kal[phase] * due_kal[phase];
            }

            if (den_gain > 1.0e-6f)
            {
                dKout = num_gain / (den_gain + 1.0e-9f);
                dKout = dv_clampf(dKout, -0.20f, 0.20f);
                h->kout_state += alpha_meta * 0.45f * dKout;
            }

            num_res = 0.0f;
            den_res = 0.0f;

            for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
            {
                num_res += eval_vec[phase] * due_res[phase];
                den_res += due_res[phase] * due_res[phase];
            }

            if (den_res > 1.0e-6f)
            {
                dKres = num_res / (den_res + 1.0e-9f);
                dKres = dv_clampf(dKres, -0.25f, 0.25f);
                h->kres_state += alpha_meta * 0.35f * dKres;
            }

            num_time = 0.0f;
            den_time = 0.0f;

            for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
            {
                num_time += eval_vec[phase] * h->de_filt[phase];
                den_time += h->de_filt[phase] * h->de_filt[phase];
            }

            if (den_time > 1.0e5f)
            {
                dT_est = num_time / (den_time + 1.0e-9f);
                dT_est = dv_clampf(dT_est, -80.0e-6f, 80.0e-6f);
                h->tcal_state += alpha_meta * 0.60f * dT_est;
            }
        }

        h->kout_state = dv_clampf(h->kout_state, 0.75f, 1.30f);
        h->kres_state = dv_clampf(h->kres_state, 0.0f, 0.55f);

        Tcal_bound = 0.50f * Td_eff;
        if (Tcal_bound < 2.0f * Ts_eff)
        {
            Tcal_bound = 2.0f * Ts_eff;
        }
        if (Tcal_bound > 250.0e-6f)
        {
            Tcal_bound = 250.0e-6f;
        }

        h->tcal_state = dv_clampf(h->tcal_state, -Tcal_bound, Tcal_bound);
    }
    else
    {
        /* Before delayed self-evaluation is valid, keep the initial schedule. */
        eval_rms = h->dbg.eval_rms;
        y_rms = h->dbg.signal_rms;
        pred_rms = h->dbg.pred_rms;
        rel_err = h->dbg.rel_err;
    }

    R_slow = 4.0f;
    R_fast = 1.0f;
    Qh_slow = 2.0e-4f;
    Qh_fast = 2.0e-3f;
    Qdc_slow = 1.0e-6f;
    Qdc_fast = 5.0e-6f;
    tau_res_slow = 600.0e-6f;
    tau_res_fast = 180.0e-6f;
    tau_der_slow = 900.0e-6f;
    tau_der_fast = 250.0e-6f;

    R_meas = R_slow + h->gamma_state * (R_fast - R_slow);
    Q_h = Qh_slow + h->gamma_state * (Qh_fast - Qh_slow);
    Q_dc = Qdc_slow + h->gamma_state * (Qdc_fast - Qdc_slow);
    tau_res = tau_res_slow + h->gamma_state * (tau_res_fast - tau_res_slow);
    tau_der = tau_der_slow + h->gamma_state * (tau_der_fast - tau_der_slow);

    Kres_eff = h->kres_state + 0.25f * h->gamma_state;
    Kres_eff = dv_clampf(Kres_eff, 0.0f, 0.75f);

    if (h->flicker_score < 0.10f)
    {
        Kres_eff = 0.0f;
    }
    else if (h->flicker_score < 0.30f)
    {
        Kres_eff *= (h->flicker_score / 0.30f);
    }

    Kout_eff = h->kout_state;

    Td_pred = Td_eff + h->tcal_state;
    if (Td_pred < 0.0f)
    {
        Td_pred = 0.0f;
    }

    h_now[0] = 1.0f;
    h_fut[0] = 1.0f;
    col = 1u;

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float f_m = h->freq_hz[m];
        float w_m = two_pi * f_m;
        float th_now = h->theta[m];
        float th_fut = th_now + w_m * Td_pred;

        h_now[col] = cosf(th_now);
        h_now[col + 1u] = sinf(th_now);
        h_fut[col] = cosf(th_fut);
        h_fut[col + 1u] = sinf(th_fut);
        col += 2u;
    }

    alpha_res = Ts_eff / (tau_res + Ts_eff);
    alpha_der = Ts_eff / (tau_der + Ts_eff);
    alpha_res = dv_clampf(alpha_res, 0.0f, 1.0f);
    alpha_der = dv_clampf(alpha_der, 0.0f, 1.0f);

    res_limit = (0.20f + 0.20f * h->gamma_state) * h->cfg.vmax_volts;

    for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
    {
        y = e_abc[phase];

        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            if (i == 0u)
            {
                P_temp = h->p[phase][i] + Q_dc;
            }
            else
            {
                P_temp = h->p[phase][i] + Q_h;
            }

            P_pred[i] = dv_clampf(P_temp, h->cfg.p_min, h->cfg.p_max);
        }

        e_hat_before = 0.0f;
        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            e_hat_before += h_now[i] * h->x[phase][i];
        }

        innovation = y - e_hat_before;
        e_hat_before_dbg[phase] = e_hat_before;
        innovation_dbg[phase] = innovation;

        S = R_meas;
        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            S += h_now[i] * h_now[i] * P_pred[i];
        }

        if (S < 1.0e-12f)
        {
            S = 1.0e-12f;
        }

        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            K = P_pred[i] * h_now[i] / S;
            h->x[phase][i] += K * innovation;

            P_new = (1.0f - K * h_now[i]) * P_pred[i];
            h->p[phase][i] = dv_clampf(P_new, h->cfg.p_min, h->cfg.p_max);
        }

        e_kalman_future = 0.0f;
        for (i = 0u; i < DYNVOLT_PRED_STATE_COUNT; i++)
        {
            e_kalman_future += h_fut[i] * h->x[phase][i];
        }

        residual = innovation;
        r_old = h->r_filt[phase];
        h->r_filt[phase] += alpha_res * (residual - h->r_filt[phase]);
        dr_raw = (h->r_filt[phase] - r_old) / Ts_eff;
        h->dr_filt[phase] += alpha_der * (dr_raw - h->dr_filt[phase]);

        r_future = h->r_filt[phase] + Td_pred * h->dr_filt[phase];
        r_future = dv_clampf(r_future, -res_limit, res_limit);

        kal_future_abc[phase] = e_kalman_future;
        res_future_abc[phase] = r_future;
        e_pred_abc[phase] = Kout_eff * e_kalman_future + Kres_eff * r_future;
    }

    p_rms_now = sqrtf((e_pred_abc[0] * e_pred_abc[0] + e_pred_abc[1] * e_pred_abc[1] + e_pred_abc[2] * e_pred_abc[2]) / 3.0f);
    p_max_now = dv_maxf(dv_absf(e_pred_abc[0]), dv_maxf(dv_absf(e_pred_abc[1]), dv_absf(e_pred_abc[2])));
    (void)p_rms_now;
    (void)p_max_now;

    alpha_target = h->flicker_score;
    if (alpha_target < 0.05f)
    {
        alpha_target = 0.0f;
    }
    if (alpha_target > 0.98f)
    {
        alpha_target = 1.0f;
    }

    if (alpha_target > h->alpha_pred_state)
    {
        tau_alpha = 1.0e-3f;
    }
    else
    {
        tau_alpha = 4.0e-3f;
    }

    alpha_pred = Ts_eff / (tau_alpha + Ts_eff);
    alpha_pred = dv_clampf(alpha_pred, 0.0f, 1.0f);

    h->alpha_pred_state += alpha_pred * (alpha_target - h->alpha_pred_state);
    h->alpha_pred_state = dv_clampf(h->alpha_pred_state, 0.0f, 1.0f);

    if ((no_flicker_candidate != 0u) && (h->flicker_score < 0.05f))
    {
        h->alpha_pred_state = 0.0f;
    }

    quiet_gate_global = h->alpha_pred_state;

    for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
    {
        e_pred_abc[phase] = e_abc[phase] + quiet_gate_global * (e_pred_abc[phase] - e_abc[phase]);
    }

    sat_flag = 0.0f;
    for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
    {
        if (e_pred_abc[phase] > h->cfg.vmax_volts)
        {
            e_pred_abc[phase] = h->cfg.vmax_volts;
            sat_flag = 1.0f;
        }
        else if (e_pred_abc[phase] < -h->cfg.vmax_volts)
        {
            e_pred_abc[phase] = -h->cfg.vmax_volts;
            sat_flag = 1.0f;
        }
    }

    if (sample_counter < (uint32_t)N_warm)
    {
        warm_gain = ((float)sample_counter) / ((float)N_warm);
    }
    else
    {
        warm_gain = 1.0f;
    }

    for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
    {
        e_pred_abc[phase] *= warm_gain;
    }

    for (phase = 0u; phase < DYNVOLT_PRED_PHASE_COUNT; phase++)
    {
        h->pred_buf[phase][h->buf_index] = e_pred_abc[phase];
        h->kal_buf[phase][h->buf_index] = kal_future_abc[phase];
        h->res_buf[phase][h->buf_index] = res_future_abc[phase];
    }

    h->sat_buf[h->buf_index] = (sat_flag > 0.5f) ? 1u : 0u;

    h->buf_index++;
    if (h->buf_index >= DYNVOLT_PRED_DELAY_BUF_LEN)
    {
        h->buf_index = 0u;
    }

    for (m = 0u; m < DYNVOLT_PRED_FREQ_COUNT; m++)
    {
        float w_m = two_pi * h->freq_hz[m];
        h->theta[m] += w_m * Ts_eff;

        if (h->theta[m] >= two_pi)
        {
            h->theta[m] -= two_pi * floorf(h->theta[m] / two_pi);
        }
    }

    if (ea_pred != NULL) { *ea_pred = e_pred_abc[0]; }
    if (eb_pred != NULL) { *eb_pred = e_pred_abc[1]; }
    if (ec_pred != NULL) { *ec_pred = e_pred_abc[2]; }

    h->dbg.sample_counter = sample_counter;
    h->dbg.delay_samples = D_check;
    h->dbg.warmup_samples = N_warm;

    h->dbg.eval_rms = eval_rms;
    h->dbg.signal_rms = y_rms;
    h->dbg.pred_rms = pred_rms;
    h->dbg.rel_err = rel_err;

    h->dbg.kout_state = h->kout_state;
    h->dbg.kout_eff = Kout_eff;
    h->dbg.kres_state = h->kres_state;
    h->dbg.kres_eff = Kres_eff;
    h->dbg.tcal_state = h->tcal_state;
    h->dbg.td_pred_sec = Td_pred;
    h->dbg.gamma_state = h->gamma_state;

    h->dbg.R_meas = R_meas;
    h->dbg.Q_h = Q_h;
    h->dbg.Q_dc = Q_dc;
    h->dbg.tau_res = tau_res;
    h->dbg.tau_der = tau_der;

    h->dbg.flicker_score = h->flicker_score;
    h->dbg.quiet_score = h->quiet_score;
    h->dbg.alpha_pred = h->alpha_pred_state;

    h->dbg.e_rms_env = h->e_rms_env;
    h->dbg.e_max_env = h->e_max_env;
    h->dbg.de_rms_env = h->de_rms_env;
    h->dbg.de_max_env = h->de_max_env;

    h->dbg.ea_pred = e_pred_abc[0];
    h->dbg.eb_pred = e_pred_abc[1];
    h->dbg.ec_pred = e_pred_abc[2];

    h->dbg.ea_kal_future = kal_future_abc[0];
    h->dbg.eb_kal_future = kal_future_abc[1];
    h->dbg.ec_kal_future = kal_future_abc[2];

    h->dbg.ea_res_future = res_future_abc[0];
    h->dbg.eb_res_future = res_future_abc[1];
    h->dbg.ec_res_future = res_future_abc[2];

    h->dbg.ea_innovation = innovation_dbg[0];
    h->dbg.eb_innovation = innovation_dbg[1];
    h->dbg.ec_innovation = innovation_dbg[2];

    h->dbg.ea_hat_before = e_hat_before_dbg[0];
    h->dbg.eb_hat_before = e_hat_before_dbg[1];
    h->dbg.ec_hat_before = e_hat_before_dbg[2];

    h->dbg.valid_delay = valid_delay;
    h->dbg.adapt_active = adapt_active;
    h->dbg.saturated = (sat_flag > 0.5f) ? 1u : 0u;
    h->dbg.active = (h->flicker_score > 0.15f) ? 1u : 0u;
}

const DynVoltPredictor3P_Debug *DynVoltPredictor3P_GetDebug(const DynVoltPredictor3P *h)
{
    if (h == NULL)
    {
        return NULL;
    }

    return &h->dbg;
}
