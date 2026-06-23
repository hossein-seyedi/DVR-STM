#ifndef DYNVOLT_PREDICTOR_3P_H_
#define DYNVOLT_PREDICTOR_3P_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYNVOLT_PRED_PHASE_COUNT      3u
#define DYNVOLT_PRED_FREQ_COUNT       51u
#define DYNVOLT_PRED_STATE_COUNT      ((2u * DYNVOLT_PRED_FREQ_COUNT) + 1u)
#define DYNVOLT_PRED_DELAY_BUF_LEN    128u

typedef struct
{
    float ts_sec;
    float td_sec;

    float f_start_hz;
    float df_hz;

    float vmax_volts;
    float warmup_sec;

    float p_init;
    float p_min;
    float p_max;

    uint8_t enable_predictor;
    uint8_t enable_adaptation;

} DynVoltPredictor3P_Config;

typedef struct
{
    uint32_t sample_counter;
    uint16_t delay_samples;
    uint16_t warmup_samples;

    float eval_rms;
    float signal_rms;
    float pred_rms;
    float rel_err;

    float kout_state;
    float kres_state;
    float tcal_state;
    float gamma_state;

    float flicker_score;
    float alpha_pred;

    float e_rms_env;
    float e_max_env;
    float de_rms_env;

    float ea_pred;
    float eb_pred;
    float ec_pred;

    uint8_t valid_delay;
    uint8_t adapt_active;
    uint8_t saturated;
    uint8_t active;

} DynVoltPredictor3P_Debug;

typedef struct
{
    DynVoltPredictor3P_Config cfg;

    float x[DYNVOLT_PRED_PHASE_COUNT][DYNVOLT_PRED_STATE_COUNT];
    float p[DYNVOLT_PRED_PHASE_COUNT][DYNVOLT_PRED_STATE_COUNT];

    float osc_cos[DYNVOLT_PRED_FREQ_COUNT];
    float osc_sin[DYNVOLT_PRED_FREQ_COUNT];

    float step_cos[DYNVOLT_PRED_FREQ_COUNT];
    float step_sin[DYNVOLT_PRED_FREQ_COUNT];

    float fut_cos[DYNVOLT_PRED_FREQ_COUNT];
    float fut_sin[DYNVOLT_PRED_FREQ_COUNT];

    float basis_now[DYNVOLT_PRED_STATE_COUNT];
    float basis_fut[DYNVOLT_PRED_STATE_COUNT];
    float p_work[DYNVOLT_PRED_STATE_COUNT];

    float r_filt[DYNVOLT_PRED_PHASE_COUNT];
    float dr_filt[DYNVOLT_PRED_PHASE_COUNT];

    float e_prev[DYNVOLT_PRED_PHASE_COUNT];
    float de_filt[DYNVOLT_PRED_PHASE_COUNT];

    float pred_buf[DYNVOLT_PRED_PHASE_COUNT][DYNVOLT_PRED_DELAY_BUF_LEN];
    float kal_buf[DYNVOLT_PRED_PHASE_COUNT][DYNVOLT_PRED_DELAY_BUF_LEN];
    float res_buf[DYNVOLT_PRED_PHASE_COUNT][DYNVOLT_PRED_DELAY_BUF_LEN];
    uint8_t sat_buf[DYNVOLT_PRED_DELAY_BUF_LEN];

    uint16_t buf_index;
    uint16_t delay_samples;
    uint16_t warmup_samples;

    float kout_state;
    float kres_state;
    float tcal_state;
    float gamma_state;

    float env_eval2;
    float env_y2;
    float env_pred2;

    float e_rms_env;
    float e_max_env;
    float de_rms_env;
    float de_max_env;

    float flicker_score;
    float alpha_pred_state;

    float last_future_delta_time_sec;

    uint32_t sample_counter;
    uint8_t initialized;

    DynVoltPredictor3P_Debug dbg;

} DynVoltPredictor3P;

void DynVoltPredictor3P_GetDefaultConfig(DynVoltPredictor3P_Config *cfg);

void DynVoltPredictor3P_Init(DynVoltPredictor3P *h,
                             const DynVoltPredictor3P_Config *cfg);

void DynVoltPredictor3P_Reset(DynVoltPredictor3P *h);

void DynVoltPredictor3P_Step(DynVoltPredictor3P *h,
                             float ea,
                             float eb,
                             float ec,
                             float *ea_pred,
                             float *eb_pred,
                             float *ec_pred);

const DynVoltPredictor3P_Debug *DynVoltPredictor3P_GetDebug(const DynVoltPredictor3P *h);

#ifdef __cplusplus
}
#endif

#endif
