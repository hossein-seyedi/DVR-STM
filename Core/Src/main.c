/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sogi.h"
#include "pwm_3leg_sine.h"
#include "dynvolt_predictor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    float u;
    float v;
    float w;
} Phase3f;

typedef enum
{
    REF_MODE_SOGI_FILTERED = 0,
    REF_MODE_PLL_CLEAN     = 1
} ReferenceMode_t;

typedef enum
{
    PWM_MODE_MEASURED_DIRECT     = 0,
    PWM_MODE_REFERENCE_DIRECT    = 1,
    PWM_MODE_INJECTION_DIRECT    = 2,
    PWM_MODE_INJECTION_PREDICTOR = 3
} PwmMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Version                         "0.0.6"

#define PI_F                            3.1415926f
#define TWO_PI_F                        (2.0f * PI_F)
#define TWO_PI_OVER_3                   (2.0f * PI_F / 3.0f)
#define SQRT3_OVER_2                    0.8660254f

#define ADC_BUF_LEN                     3u
#define ADC_VREF                        3.3f
#define ADC_MAX_VALUE                   4095.0f
#define AMC_GAIN                        0.4f
#define RESISTOR_RATIO                  100.6f

#define CONTROL_RATE_HZ                 20000.0f
#define CONTROL_TS_SEC                  (1.0f / CONTROL_RATE_HZ)
#define CONTROL_TS_US                   (1000000.0f / CONTROL_RATE_HZ)
#define CONTROL_OVERRUN_LIMIT_US        45.0f

#define PREDICTOR_RATE_HZ               500.0f
#define PREDICTOR_TS_SEC                (1.0f / PREDICTOR_RATE_HZ)
#define PREDICTOR_OVERRUN_LIMIT_US      1500.0f

#define ERROR_DELAY_MS_DEFAULT          0.30f
#define ERROR_DELAY_BUFFER_LEN          2000u
#define ERROR_DELAY_SAMPLES_MAX         (ERROR_DELAY_BUFFER_LEN - 1u)
#define ERROR_DELAY_MS_MAX              (((float)ERROR_DELAY_SAMPLES_MAX * CONTROL_TS_US) / 1000.0f)

#define PREDICTOR_HORIZON_MS_DEFAULT    1.00f
#define PREDICTOR_HORIZON_MS_MIN        0.0f

#define TOTAL_DELAY_TARGET_MS_DEFAULT   1.00f
#define PREDICTOR_CALC_DELAY_MS_DEFAULT 0.70f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static PWM_3Leg_Handle g_pwm_3leg;
static PWM_3Leg_Config g_pwm_3leg_cfg;
static PWM_3Leg_Debug  g_pwm_3leg_dbg;

static DynVoltPredictor3P g_predictor;
static DynVoltPredictor3P_Config g_predictor_cfg;

volatile uint16_t adc_buf[ADC_BUF_LEN];

/* Mode variables for Watch:
   g_reference_mode: 0 = SOGI filtered reference, 1 = PLL clean sine reference
   g_output_mode:    0 = measured, 1 = reference, 2 = direct injection, 3 = predictor injection
   g_pwm_output_enabled: 0 = zero PWM command, 1 = selected output mode is applied
*/
volatile int g_reference_mode = REF_MODE_PLL_CLEAN;
volatile int g_output_mode = PWM_MODE_INJECTION_DIRECT;
volatile uint8_t g_pwm_output_enabled = 1u;

static int g_last_output_mode = -1;
volatile uint32_t g_invalid_output_mode_count = 0u;

/* User settings. These coefficients match the direct-injection program that works. */
volatile float g_clean_reference_phase_peak_volts = 325.0f;
volatile float g_pwm_reference_gain = 1.0f;
volatile float Transformer_Ratio = 9.58f;
volatile float g_direct_output_sign = 1.0f;
volatile float g_injection_output_sign = -1.0f;

/* Runtime delay setting.
   Change only g_error_delay_ms_request from Watch.
   The code converts milliseconds to an integer number of 20 kHz samples.
*/
volatile float g_error_delay_ms_request = ERROR_DELAY_MS_DEFAULT;
volatile float g_error_delay_ms_active = ERROR_DELAY_MS_DEFAULT;
volatile uint32_t g_error_delay_samples_active = 0u;
volatile uint8_t g_error_delay_limited = 0u;
volatile uint32_t g_error_delay_reconfigure_count = 0u;
volatile uint8_t g_error_delay_ready = 0u;
volatile uint32_t g_error_delay_sample_count = 0u;

static float g_error_delay_ms_request_last = -1.0f;
static float g_err_delay_buf_u[ERROR_DELAY_BUFFER_LEN];
static float g_err_delay_buf_v[ERROR_DELAY_BUFFER_LEN];
static float g_err_delay_buf_w[ERROR_DELAY_BUFFER_LEN];
static uint32_t g_err_delay_index = 0u;
static uint8_t g_error_delay_primed = 0u;

/* Optional timing helper. This does not automatically change the delay. */
volatile float g_total_delay_target_ms = TOTAL_DELAY_TARGET_MS_DEFAULT;
volatile float g_predictor_calc_delay_assumed_ms = PREDICTOR_CALC_DELAY_MS_DEFAULT;
volatile float g_error_delay_suggested_ms = ERROR_DELAY_MS_DEFAULT;
volatile uint32_t g_error_delay_suggested_samples = 0u;

/* Predictor horizon setting.
   Change g_predictor_horizon_ms_request from Watch if needed.
   No 1 ms upper clamp is used here.
*/
volatile float g_predictor_horizon_ms_request = PREDICTOR_HORIZON_MS_DEFAULT;
volatile float g_predictor_horizon_ms_active = PREDICTOR_HORIZON_MS_DEFAULT;
volatile uint32_t g_predictor_reconfigure_count = 0u;
static float g_predictor_horizon_ms_request_last = -1.0f;

/* Raw ADC */
volatile uint16_t g_adc_raw_u = 0u;
volatile uint16_t g_adc_raw_v = 0u;
volatile uint16_t g_adc_raw_w = 0u;

/* Measured voltages */
volatile float g_meas_u = 0.0f;
volatile float g_meas_v = 0.0f;
volatile float g_meas_w = 0.0f;

/* SOGI outputs */
volatile float g_sogi_u_alpha = 0.0f;
volatile float g_sogi_u_beta  = 0.0f;
volatile float g_sogi_v_alpha = 0.0f;
volatile float g_sogi_v_beta  = 0.0f;
volatile float g_sogi_w_alpha = 0.0f;
volatile float g_sogi_w_beta  = 0.0f;

/* PLL monitor variables */
volatile float g_pll_theta = 0.0f;
volatile float g_pll_freq_hz = 50.0f;
volatile float g_pll_error = 0.0f;

/* Active reference */
volatile float g_ref_u = 0.0f;
volatile float g_ref_v = 0.0f;
volatile float g_ref_w = 0.0f;

/* Error, delayed error, and direct injection */
volatile float g_err_u = 0.0f;
volatile float g_err_v = 0.0f;
volatile float g_err_w = 0.0f;

volatile float g_err_delay_u = 0.0f;
volatile float g_err_delay_v = 0.0f;
volatile float g_err_delay_w = 0.0f;

volatile float g_inj_u = 0.0f;
volatile float g_inj_v = 0.0f;
volatile float g_inj_w = 0.0f;

/* Predictor input, output, and monitor variables */
volatile float g_pred_input_err_u = 0.0f;
volatile float g_pred_input_err_v = 0.0f;
volatile float g_pred_input_err_w = 0.0f;

volatile float g_pred_input_err_filt_u = 0.0f;
volatile float g_pred_input_err_filt_v = 0.0f;
volatile float g_pred_input_err_filt_w = 0.0f;
volatile uint8_t g_predictor_error_filter_primed = 0u;

volatile float g_pred_err_u = 0.0f;
volatile float g_pred_err_v = 0.0f;
volatile float g_pred_err_w = 0.0f;

volatile float g_pred_inj_u = 0.0f;
volatile float g_pred_inj_v = 0.0f;
volatile float g_pred_inj_w = 0.0f;

volatile float g_predictor_flicker_score = 0.0f;
volatile float g_predictor_alpha = 0.0f;
volatile uint8_t g_predictor_active = 0u;
volatile uint8_t g_predictor_saturated = 0u;

/* Final PWM command */
volatile float g_pwm_u = 0.0f;
volatile float g_pwm_v = 0.0f;
volatile float g_pwm_w = 0.0f;

/* SOGI objects */
static SOGI_Config sogi_cfg;
static SOGI_State sogi_u_state;
static SOGI_State sogi_v_state;
static SOGI_State sogi_w_state;

/* PLL internal variables */
static float pll_ts = CONTROL_TS_SEC;
static float pll_kp = 80.0f;
static float pll_ki = 2000.0f;
static float pll_integrator = 0.0f;
static float pll_omega = TWO_PI_F * 50.0f;
static float pll_omega_nominal = TWO_PI_F * 50.0f;

/* Timing monitor */
volatile uint32_t g_control_cycles = 0u;
volatile uint32_t g_control_cycles_max = 0u;
volatile float g_control_time_us = 0.0f;
volatile uint8_t g_control_timing_overrun = 0u;

volatile uint8_t g_predictor_run_request = 0u;
volatile uint8_t g_predictor_is_running = 0u;
volatile uint32_t g_predictor_tick_count = 0u;
volatile uint32_t g_predictor_missed_count = 0u;

volatile uint32_t g_predictor_cycles = 0u;
volatile uint32_t g_predictor_cycles_max = 0u;
volatile float g_predictor_time_us = 0.0f;
volatile uint8_t g_predictor_timing_overrun = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void dvr_background_step(void);
static void dvr_control_step_20khz(void);
static void dvr_application_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static uint32_t clamp_u32(uint32_t x, uint32_t min_val, uint32_t max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static float cycles_to_us(uint32_t cycles)
{
    float cycles_per_us;

    cycles_per_us = ((float)SystemCoreClock) / 1000000.0f;

    if (cycles_per_us <= 0.0f)
    {
        return 0.0f;
    }

    return ((float)cycles) / cycles_per_us;
}

static float wrap_0_2pi(float x)
{
    while (x >= TWO_PI_F) x -= TWO_PI_F;
    while (x < 0.0f)      x += TWO_PI_F;
    return x;
}

static uint32_t delay_ms_to_samples(float delay_ms)
{
    float delay_us;
    uint32_t samples;

    if (delay_ms <= 0.0f)
    {
        return 0u;
    }

    delay_us = delay_ms * 1000.0f;
    samples = (uint32_t)((delay_us / CONTROL_TS_US) + 0.5f);
    samples = clamp_u32(samples, 0u, ERROR_DELAY_SAMPLES_MAX);

    return samples;
}

static float delay_samples_to_ms(uint32_t delay_samples)
{
    return (((float)delay_samples * CONTROL_TS_US) / 1000.0f);
}

static float adc_diff_to_volts(uint32_t adc_raw)
{
    float adc_diff_volts;
    float sensor_input_volts;
    float line_volts;

    adc_diff_volts = (((float)adc_raw / ADC_MAX_VALUE) * 2.0f * ADC_VREF) - ADC_VREF;
    sensor_input_volts = adc_diff_volts / AMC_GAIN;
    line_volts = sensor_input_volts * RESISTOR_RATIO;

    return line_volts;
}

static void read_adc_voltages(void)
{
    g_adc_raw_u = adc_buf[0];
    g_adc_raw_v = adc_buf[1];
    g_adc_raw_w = adc_buf[2];

    g_meas_u = adc_diff_to_volts(g_adc_raw_u);
    g_meas_v = adc_diff_to_volts(g_adc_raw_v);
    g_meas_w = adc_diff_to_volts(g_adc_raw_w);
}

static void update_sogi(void)
{
    SOGI_Output out_u;
    SOGI_Output out_v;
    SOGI_Output out_w;

    SOGI_Step(&sogi_u_state, g_meas_u, &out_u);
    SOGI_Step(&sogi_v_state, g_meas_v, &out_v);
    SOGI_Step(&sogi_w_state, g_meas_w, &out_w);

    g_sogi_u_alpha = out_u.v_alpha;
    g_sogi_u_beta  = out_u.v_beta;
    g_sogi_v_alpha = out_v.v_alpha;
    g_sogi_v_beta  = out_v.v_beta;
    g_sogi_w_alpha = out_w.v_alpha;
    g_sogi_w_beta  = out_w.v_beta;
}

static void update_pll(void)
{
    float alpha;
    float beta;
    float cos_th;
    float sin_th;
    float error_volts;
    float max_dev_int;
    float max_dev_out;

    /* Keep the same U-W-V sequence correction as the direct-injection code that works. */
    alpha = (2.0f / 3.0f) * (g_sogi_u_alpha - 0.5f * g_sogi_w_alpha - 0.5f * g_sogi_v_alpha);
    beta  = (2.0f / 3.0f) * (SQRT3_OVER_2 * (g_meas_w - g_meas_v));

    cos_th = cosf(g_pll_theta);
    sin_th = sinf(g_pll_theta);

    error_volts = (beta * cos_th) - (alpha * sin_th);
    g_pll_error = -(error_volts / g_clean_reference_phase_peak_volts);

    pll_integrator += pll_ki * pll_ts * g_pll_error;

    max_dev_int = TWO_PI_F * 10.0f;
    max_dev_out = TWO_PI_F * 20.0f;

    pll_integrator = clampf(pll_integrator, -max_dev_int, max_dev_int);

    pll_omega = pll_omega_nominal + (pll_kp * g_pll_error) + pll_integrator;
    pll_omega = clampf(pll_omega, pll_omega_nominal - max_dev_out, pll_omega_nominal + max_dev_out);

    g_pll_theta += pll_omega * pll_ts;
    g_pll_theta = wrap_0_2pi(g_pll_theta);

    g_pll_freq_hz = pll_omega / TWO_PI_F;
}

static void build_reference_from_sogi(void)
{
    g_ref_u = g_sogi_u_alpha;
    g_ref_v = g_sogi_v_alpha;
    g_ref_w = g_sogi_w_alpha;
}

static void build_reference_from_pll(void)
{
    float phase_u_angle;

    phase_u_angle = g_pll_theta - (PI_F * 0.5f);

    g_ref_u = g_clean_reference_phase_peak_volts * sinf(phase_u_angle);
    g_ref_v = g_clean_reference_phase_peak_volts * sinf(phase_u_angle + TWO_PI_OVER_3);
    g_ref_w = g_clean_reference_phase_peak_volts * sinf(phase_u_angle - TWO_PI_OVER_3);
}

static void update_selected_reference(void)
{
    /* Keep both algorithms alive all the time, exactly like the working direct-injection code. */
    update_sogi();
    update_pll();

    if (g_reference_mode == REF_MODE_PLL_CLEAN)
    {
        build_reference_from_pll();
    }
    else
    {
        build_reference_from_sogi();
    }
}

static void build_error(void)
{
    g_err_u = g_ref_u - g_meas_u;
    g_err_v = g_ref_v - g_meas_v;
    g_err_w = g_ref_w - g_meas_w;
}

static void reset_error_delay_buffer(void)
{
    uint32_t i;

    for (i = 0u; i < ERROR_DELAY_BUFFER_LEN; i++)
    {
        g_err_delay_buf_u[i] = g_err_u;
        g_err_delay_buf_v[i] = g_err_v;
        g_err_delay_buf_w[i] = g_err_w;
    }

    g_err_delay_u = g_err_u;
    g_err_delay_v = g_err_v;
    g_err_delay_w = g_err_w;

    g_err_delay_index = 0u;
    g_error_delay_sample_count = 0u;
    g_error_delay_ready = 0u;
    g_error_delay_primed = 1u;
}

static void update_error_delay(void)
{
    uint32_t delay_samples;

    delay_samples = g_error_delay_samples_active;

    if (g_error_delay_primed == 0u)
    {
        reset_error_delay_buffer();
        return;
    }

    if (delay_samples == 0u)
    {
        g_err_delay_u = g_err_u;
        g_err_delay_v = g_err_v;
        g_err_delay_w = g_err_w;
        g_err_delay_index = 0u;
        g_error_delay_ready = 1u;
    }
    else
    {
        g_err_delay_u = g_err_delay_buf_u[g_err_delay_index];
        g_err_delay_v = g_err_delay_buf_v[g_err_delay_index];
        g_err_delay_w = g_err_delay_buf_w[g_err_delay_index];

        g_err_delay_buf_u[g_err_delay_index] = g_err_u;
        g_err_delay_buf_v[g_err_delay_index] = g_err_v;
        g_err_delay_buf_w[g_err_delay_index] = g_err_w;

        g_err_delay_index++;

        if (g_err_delay_index >= delay_samples)
        {
            g_err_delay_index = 0u;
            g_error_delay_ready = 1u;
        }
    }

    g_error_delay_sample_count++;
}

static void build_direct_injection(void)
{
    /* This is the same injection rule as the working program, with only optional delay added. */
    g_inj_u = -g_err_delay_u;
    g_inj_v = -g_err_delay_v;
    g_inj_w = -g_err_delay_w;
}

static void filter_predictor_input_error(void)
{
    const float alpha = 0.6341f;

    if (g_predictor_error_filter_primed == 0u)
    {
        g_pred_input_err_filt_u = g_pred_input_err_u;
        g_pred_input_err_filt_v = g_pred_input_err_v;
        g_pred_input_err_filt_w = g_pred_input_err_w;

        g_predictor_error_filter_primed = 1u;
        return;
    }

    g_pred_input_err_filt_u += alpha * (g_pred_input_err_u - g_pred_input_err_filt_u);
    g_pred_input_err_filt_v += alpha * (g_pred_input_err_v - g_pred_input_err_filt_v);
    g_pred_input_err_filt_w += alpha * (g_pred_input_err_w - g_pred_input_err_filt_w);
}

static void build_predictor_injection(void)
{
    const DynVoltPredictor3P_Debug *pred_dbg;
    float pred_err_u;
    float pred_err_v;
    float pred_err_w;

    __disable_irq();
    g_pred_input_err_u = g_err_delay_u;
    g_pred_input_err_v = g_err_delay_v;
    g_pred_input_err_w = g_err_delay_w;
    __enable_irq();

    filter_predictor_input_error();

    DynVoltPredictor3P_Step(&g_predictor,
                            g_pred_input_err_filt_u,
                            g_pred_input_err_filt_v,
                            g_pred_input_err_filt_w,
                            &pred_err_u,
                            &pred_err_v,
                            &pred_err_w);

    g_pred_err_u = pred_err_u;
    g_pred_err_v = pred_err_v;
    g_pred_err_w = pred_err_w;

    g_pred_inj_u = -g_pred_err_u;
    g_pred_inj_v = -g_pred_err_v;
    g_pred_inj_w = -g_pred_err_w;

    pred_dbg = DynVoltPredictor3P_GetDebug(&g_predictor);

    if (pred_dbg != 0)
    {
        g_predictor_flicker_score = pred_dbg->flicker_score;
        g_predictor_alpha = pred_dbg->alpha_pred;
        g_predictor_active = pred_dbg->active;
        g_predictor_saturated = pred_dbg->saturated;
    }
}

static void select_pwm_command(Phase3f *cmd, float *sign)
{
    if (g_output_mode == PWM_MODE_MEASURED_DIRECT)
    {
        cmd->u = g_meas_u;
        cmd->v = g_meas_v;
        cmd->w = g_meas_w;
        *sign = g_direct_output_sign;
    }
    else if (g_output_mode == PWM_MODE_REFERENCE_DIRECT)
    {
        cmd->u = g_ref_u;
        cmd->v = g_ref_v;
        cmd->w = g_ref_w;
        *sign = g_direct_output_sign;
    }
    else if (g_output_mode == PWM_MODE_INJECTION_PREDICTOR)
    {
        cmd->u = g_pred_inj_u;
        cmd->v = g_pred_inj_v;
        cmd->w = g_pred_inj_w;
        *sign = g_injection_output_sign;
    }
    else
    {
        cmd->u = g_inj_u;
        cmd->v = g_inj_v;
        cmd->w = g_inj_w;
        *sign = g_injection_output_sign;
    }
}

static void apply_pwm_output(void)
{
    Phase3f cmd;
    float sign;

    select_pwm_command(&cmd, &sign);

    if (g_pwm_output_enabled == 0u)
    {
        g_pwm_u = 0.0f;
        g_pwm_v = 0.0f;
        g_pwm_w = 0.0f;
    }
    else
    {
        /* Keep the same coefficient chain as the working direct-injection code. */
        g_pwm_u = g_pwm_reference_gain * sign * cmd.u * Transformer_Ratio;
        g_pwm_v = g_pwm_reference_gain * sign * cmd.v * Transformer_Ratio;
        g_pwm_w = g_pwm_reference_gain * sign * cmd.w * Transformer_Ratio;
    }

    PWM_3Leg_ApplyPhaseReferences(&g_pwm_3leg,
                                  g_pwm_u,
                                  g_pwm_v,
                                  g_pwm_w,
                                  &g_pwm_3leg_dbg);
}

static void handle_mode_change(void)
{
    if ((g_output_mode < PWM_MODE_MEASURED_DIRECT) ||
        (g_output_mode > PWM_MODE_INJECTION_PREDICTOR))
    {
        g_output_mode = PWM_MODE_INJECTION_DIRECT;
        g_invalid_output_mode_count++;
    }

    if (g_output_mode != g_last_output_mode)
    {
        if (g_output_mode == PWM_MODE_INJECTION_PREDICTOR)
        {
            g_predictor_error_filter_primed = 0u;
        }

        g_last_output_mode = g_output_mode;
    }
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void predictor_init_from_runtime_settings(void)
{
    float horizon_ms;

    horizon_ms = g_predictor_horizon_ms_request;

    if (horizon_ms < PREDICTOR_HORIZON_MS_MIN)
    {
        horizon_ms = PREDICTOR_HORIZON_MS_MIN;
        g_predictor_horizon_ms_request = horizon_ms;
    }

    g_predictor_horizon_ms_active = horizon_ms;
    g_predictor_horizon_ms_request_last = horizon_ms;

    DynVoltPredictor3P_GetDefaultConfig(&g_predictor_cfg);
    g_predictor_cfg.ts_sec = PREDICTOR_TS_SEC;
    g_predictor_cfg.td_sec = horizon_ms * 1.0e-3f;
    g_predictor_cfg.vmax_volts = 80.0f;
    g_predictor_cfg.enable_predictor = 1u;
    g_predictor_cfg.enable_adaptation = 1u;
    DynVoltPredictor3P_Init(&g_predictor, &g_predictor_cfg);

    g_predictor_error_filter_primed = 0u;
    g_predictor_reconfigure_count++;
}

static void update_delay_suggestion(void)
{
    float suggested_ms;
    uint32_t suggested_samples;

    suggested_ms = g_total_delay_target_ms - g_predictor_calc_delay_assumed_ms;

    if (suggested_ms < 0.0f)
    {
        suggested_ms = 0.0f;
    }

    suggested_samples = delay_ms_to_samples(suggested_ms);
    g_error_delay_suggested_samples = suggested_samples;
    g_error_delay_suggested_ms = delay_samples_to_ms(suggested_samples);
}

static void service_error_delay_request(void)
{
    float request_ms;
    uint32_t samples;

    request_ms = g_error_delay_ms_request;

    if (fabsf(request_ms - g_error_delay_ms_request_last) <= 0.0001f)
    {
        return;
    }

    g_error_delay_limited = 0u;

    if (request_ms < 0.0f)
    {
        request_ms = 0.0f;
        g_error_delay_limited = 1u;
    }

    samples = delay_ms_to_samples(request_ms);

    if (request_ms > ERROR_DELAY_MS_MAX)
    {
        g_error_delay_limited = 1u;
    }

    __disable_irq();
    g_error_delay_samples_active = samples;
    g_error_delay_ms_active = delay_samples_to_ms(samples);
    g_error_delay_ms_request = g_error_delay_ms_active;
    g_error_delay_ms_request_last = g_error_delay_ms_active;
    reset_error_delay_buffer();
    g_error_delay_reconfigure_count++;
    __enable_irq();
}

static void service_predictor_horizon_request(void)
{
    float request_ms;

    request_ms = g_predictor_horizon_ms_request;

    if (fabsf(request_ms - g_predictor_horizon_ms_request_last) <= 0.0001f)
    {
        return;
    }

    predictor_init_from_runtime_settings();
}

static void dvr_predictor_step_500hz(void)
{
    uint32_t start_cycles;
    uint32_t end_cycles;
    uint32_t elapsed_cycles;

    g_predictor_is_running = 1u;

    start_cycles = DWT->CYCCNT;

    build_predictor_injection();

    end_cycles = DWT->CYCCNT;
    elapsed_cycles = end_cycles - start_cycles;

    g_predictor_cycles = elapsed_cycles;

    if (elapsed_cycles > g_predictor_cycles_max)
    {
        g_predictor_cycles_max = elapsed_cycles;
    }

    g_predictor_time_us = cycles_to_us(elapsed_cycles);

    if (g_predictor_time_us > PREDICTOR_OVERRUN_LIMIT_US)
    {
        g_predictor_timing_overrun = 1u;
    }

    g_predictor_is_running = 0u;
}

static void dvr_background_step(void)
{
    uint8_t run_predictor;

    update_delay_suggestion();
    service_error_delay_request();
    service_predictor_horizon_request();

    run_predictor = 0u;

    if (g_predictor_run_request != 0u)
    {
        __disable_irq();
        g_predictor_run_request = 0u;
        __enable_irq();

        if (g_output_mode == PWM_MODE_INJECTION_PREDICTOR)
        {
            run_predictor = 1u;
        }
    }

    if (run_predictor != 0u)
    {
        dvr_predictor_step_500hz();
    }
}

static void dvr_control_step_20khz(void)
{
    uint32_t start_cycles;
    uint32_t end_cycles;
    uint32_t elapsed_cycles;

    start_cycles = DWT->CYCCNT;

    read_adc_voltages();
    handle_mode_change();
    update_selected_reference();
    build_error();
    update_error_delay();
    build_direct_injection();
    apply_pwm_output();

    end_cycles = DWT->CYCCNT;
    elapsed_cycles = end_cycles - start_cycles;

    g_control_cycles = elapsed_cycles;

    if (elapsed_cycles > g_control_cycles_max)
    {
        g_control_cycles_max = elapsed_cycles;
    }

    g_control_time_us = cycles_to_us(elapsed_cycles);

    if (g_control_time_us > CONTROL_OVERRUN_LIMIT_US)
    {
        g_control_timing_overrun = 1u;
    }
}

static void dvr_application_init(void)
{
    HAL_ADC_Stop(&hadc4);

    if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    sogi_cfg.Ts = CONTROL_TS_SEC;
    sogi_cfg.k  = 0.8f;
    sogi_cfg.w  = TWO_PI_F * 50.0f;

    SOGI_Init(&sogi_u_state, &sogi_cfg);
    SOGI_Init(&sogi_v_state, &sogi_cfg);
    SOGI_Init(&sogi_w_state, &sogi_cfg);

    g_pll_theta = 0.0f;
    pll_integrator = 0.0f;
    pll_omega_nominal = TWO_PI_F * 50.0f;
    pll_omega = pll_omega_nominal;
    pll_ts = CONTROL_TS_SEC;

    /* Default mode matches the working direct-injection program. */
    g_reference_mode = REF_MODE_PLL_CLEAN;
    g_output_mode = PWM_MODE_INJECTION_DIRECT;
    g_last_output_mode = -1;
    g_pwm_output_enabled = 1u;

    /* Keep the direct-injection coefficients exactly as the working program. */
    g_pwm_reference_gain = 1.0f;
    Transformer_Ratio = 9.58f;
    g_direct_output_sign = 1.0f;
    g_injection_output_sign = -1.0f;

    g_error_delay_ms_request = ERROR_DELAY_MS_DEFAULT;
    g_error_delay_ms_request_last = -1.0f;
    service_error_delay_request();

    g_predictor_error_filter_primed = 0u;
    g_predictor_horizon_ms_request = PREDICTOR_HORIZON_MS_DEFAULT;
    g_predictor_horizon_ms_request_last = -1.0f;
    predictor_init_from_runtime_settings();

    if (HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK)
    {
        Error_Handler();
    }

    g_pwm_3leg_cfg.vdc_volts = 400.0f;
    g_pwm_3leg_cfg.output_freq_hz = 50.0f;
    g_pwm_3leg_cfg.pwm_update_hz = CONTROL_RATE_HZ;
    g_pwm_3leg_cfg.modulation_index = 1.0f;
    g_pwm_3leg_cfg.duty_min = 0.02f;
    g_pwm_3leg_cfg.duty_max = 0.98f;
    g_pwm_3leg_cfg.enable_zero_sequence = true;

    PWM_3Leg_Init(&g_pwm_3leg, &htim1, &g_pwm_3leg_cfg);
    PWM_3Leg_ApplyPhaseReferences(&g_pwm_3leg, 0.0f, 0.0f, 0.0f, &g_pwm_3leg_dbg);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start(&htim6);

    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
    {
        Error_Handler();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
dwt_init();
/* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC4_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
dvr_application_init();
g_error_delay_ms_request = 4.5f;
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    dvr_background_step();
/* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC4)
    {
        dvr_control_step_20khz();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        g_predictor_tick_count++;

        if (g_predictor_run_request == 0u)
        {
            g_predictor_run_request = 1u;
        }
        else
        {
            g_predictor_missed_count++;
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
