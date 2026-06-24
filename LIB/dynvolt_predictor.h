#ifndef DYNVOLT_PREDICTOR_3P_H_
#define DYNVOLT_PREDICTOR_3P_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DynVolt 3-phase predictor - one-phase narrow-bank implementation.
 *
 * The public API is kept compatible with the previous DynVoltPredictor3P module.
 * Internally, only logical phase u is estimated by a narrow frequency-bank RLS/Kalman
 * predictor. The logical v and w predicted errors are reconstructed from the same
 * estimated harmonic coefficients using the configured three-phase sequence.
 *
 * Output convention:
 *   DynVoltPredictor3P_Step() returns predicted voltage errors, not injection
 *   voltages. The main program must keep using:
 *       g_pred_inj_x = -g_pred_err_x;
 */

#define DYNVOLT_PRED_PHASE_COUNT      3u

/* Narrow bank for the known grid-simulator flicker:
 *   half period = 0.0568 s
 *   flicker frequency = 1/(2*0.0568) = 8.8028169 Hz
 *
 * The bank is built around:
 *   50 Hz
 *   50 +/- flicker frequency, with fine offsets
 *   50 +/- 3*flicker frequency
 */
#define DYNVOLT_PRED_FREQ_COUNT       13u
#define DYNVOLT_PRED_STATE_COUNT      ((2u * DYNVOLT_PRED_FREQ_COUNT) + 1u)
#define DYNVOLT_PRED_DELAY_BUF_LEN    128u

typedef struct
{
    float ts_sec;
    float td_sec;

    /* Kept for backward compatibility with the older wide-bank version. */
    float f_start_hz;
    float df_hz;

    float vmax_volts;
    float warmup_sec;

    float p_init;
    float p_min;
    float p_max;

    uint8_t enable_predictor;
    uint8_t enable_adaptation;

    /* New fields for the one-phase narrow-bank version. */
    float grid_hz;
    float flicker_hz;
    float fine_df_hz;

    /* Phase shifts used for reconstructing V and W from U.
     * Default matches the centralized standard u-v-w sequence:
     *   V = U - 120 deg
     *   W = U + 120 deg
     */
    float phase_v_shift_rad;
    float phase_w_shift_rad;

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

    /* Single-phase diagonal RLS/Kalman state for phase U only. */
    float x[DYNVOLT_PRED_STATE_COUNT];
    float p[DYNVOLT_PRED_STATE_COUNT];

    float freq_hz[DYNVOLT_PRED_FREQ_COUNT];

    float osc_cos[DYNVOLT_PRED_FREQ_COUNT];
    float osc_sin[DYNVOLT_PRED_FREQ_COUNT];

    float step_cos[DYNVOLT_PRED_FREQ_COUNT];
    float step_sin[DYNVOLT_PRED_FREQ_COUNT];

    float fut_cos[DYNVOLT_PRED_FREQ_COUNT];
    float fut_sin[DYNVOLT_PRED_FREQ_COUNT];

    float phase_v_cos;
    float phase_v_sin;
    float phase_w_cos;
    float phase_w_sin;

    float e_prev_u;
    float de_filt_u;

    float pred_buf_u[DYNVOLT_PRED_DELAY_BUF_LEN];
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

#endif /* DYNVOLT_PREDICTOR_3P_H_ */
