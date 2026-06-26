#ifndef DYNVOLT_PREDICTOR_3P_H_
#define DYNVOLT_PREDICTOR_3P_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DynVolt 3-phase delay-compensated self-tuning error predictor.
 *
 * This is the wide-bank version translated from the MATLAB Function
 * DynVoltErrorPredictor3P().
 *
 * Frequency bank:
 *   f_start_hz = 25 Hz
 *   df_hz      = 1 Hz
 *   count      = 51 frequencies, i.e. 25:1:75 Hz
 *
 * Output convention:
 *   DynVoltPredictor3P_Step() returns predicted voltage errors.
 *   The main program must still use:
 *       g_pred_inj_x = -g_pred_err_x;
 */

#define DYNVOLT_PRED_PHASE_COUNT      3u
#define DYNVOLT_PRED_FREQ_COUNT       51u
#define DYNVOLT_PRED_STATE_COUNT      ((2u * DYNVOLT_PRED_FREQ_COUNT) + 1u)

/*
 * MATLAB version used MAX_BUF = 5000.
 * On STM32 this would consume too much SRAM:
 *   3 buffers * 3 phases * 5000 floats = 180 kB only for the buffers.
 *
 * For the current MCU implementation the predictor runs at 2 kHz.
 * With Td = 5 ms, the delayed self-evaluation needs only 10 samples.
 * Therefore 128 samples is enough for normal embedded tests.
 *
 * If you really want the exact MATLAB buffer length and have enough SRAM,
 * change this define to 5000u.
 */
#define DYNVOLT_PRED_DELAY_BUF_LEN    128u

#ifndef DYNVOLT_PRED_PI_F
#define DYNVOLT_PRED_PI_F             3.14159265358979323846f
#endif

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

    /* Kept for compatibility with the narrow-bank version. Not used here. */
    float grid_hz;
    float flicker_hz;
    float fine_df_hz;
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
    float kout_eff;
    float kres_state;
    float kres_eff;
    float tcal_state;
    float td_pred_sec;
    float gamma_state;

    float R_meas;
    float Q_h;
    float Q_dc;
    float tau_res;
    float tau_der;

    float flicker_score;
    float quiet_score;
    float alpha_pred;

    float e_rms_env;
    float e_max_env;
    float de_rms_env;
    float de_max_env;

    float ea_pred;
    float eb_pred;
    float ec_pred;

    float ea_kal_future;
    float eb_kal_future;
    float ec_kal_future;

    float ea_res_future;
    float eb_res_future;
    float ec_res_future;

    float ea_innovation;
    float eb_innovation;
    float ec_innovation;

    float ea_hat_before;
    float eb_hat_before;
    float ec_hat_before;

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

    float theta[DYNVOLT_PRED_FREQ_COUNT];
    float freq_hz[DYNVOLT_PRED_FREQ_COUNT];

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

    float quiet_score;
    float e_rms_env;
    float e_max_env;
    float de_rms_env;
    float de_max_env;
    float flicker_score;
    float alpha_pred_state;

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
