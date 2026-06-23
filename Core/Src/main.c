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
#include "dynvolt_lpf.h"
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

typedef struct
{
    float alpha;
    float beta;
} AlphaBeta2f;

typedef enum
{
    REF_MODE_SOGI_FILTERED = 0,
    REF_MODE_PLL_CLEAN     = 1
} ReferenceMode_t;

typedef enum
{
    PWM_MODE_MEASURED_DIRECT       = 0, /* output = measured voltage */
    PWM_MODE_REFERENCE_DIRECT      = 1, /* output = reference voltage */
    PWM_MODE_INJECTION_DIRECT      = 2, /* output = direct series injection */
    PWM_MODE_INJECTION_PREDICTOR   = 3  /* output = predictor-based series injection */
} PwmMode_t;

typedef struct
{
    float ts;
    float kp;
    float ki;

    float theta;          /* PLL vector angle */
    float omega;          /* rad/s */
    float omega_nominal;  /* rad/s */

    float integrator;
    float phase_error;
    float freq_hz;
} AB_PLL_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static PWM_3Leg_Handle g_pwm_3leg;
static PWM_3Leg_Config g_pwm_3leg_cfg;
static PWM_3Leg_Debug  g_pwm_3leg_dbg;

static DynVolt_LPF3P_F32 g_predictor_error_lpf;
static DynVoltPredictor3P g_predictor;
static DynVoltPredictor3P_Config g_predictor_cfg;

#define Version "0.0.3"
#define PREDICTOR_RATE_HZ        500.0f
#define PREDICTOR_TS_SEC         (1.0f / PREDICTOR_RATE_HZ)
#define PREDICTOR_PERIOD_US      2000.0f

#define ADC_BUF_LEN 3

volatile uint16_t adc_buf[ADC_BUF_LEN];

/* =========================
   MODE VARIABLES FOR WATCH
   =========================

   g_reference_mode:
   0 = use SOGI filtered voltage as reference
   1 = use PLL clean sine as reference

   g_output_mode:
   0 = PWM output = measured voltage
   1 = PWM output = reference voltage
   2 = PWM output = direct injection voltage
   3 = PWM output = predictor injection voltage
*/
volatile int g_reference_mode = 0;
volatile int g_output_mode    = 1;
static int g_last_output_mode = -1;

/* user settings */
volatile float g_clean_reference_phase_peak_volts = 325.0f;
volatile float g_pwm_reference_gain = 1.0f;

/* raw ADC */
volatile uint16_t g_adc_raw_u = 0;
volatile uint16_t g_adc_raw_v = 0;
volatile uint16_t g_adc_raw_w = 0;

/* measured voltages */
volatile float g_meas_u = 0.0f;
volatile float g_meas_v = 0.0f;
volatile float g_meas_w = 0.0f;

/* filtered error used only as predictor input */
volatile float g_pred_input_err_filt_u = 0.0f;
volatile float g_pred_input_err_filt_v = 0.0f;
volatile float g_pred_input_err_filt_w = 0.0f;
volatile uint8_t g_predictor_error_filter_primed = 0u;

/*the transformer ratio ----------------------------------*/
float Transformer_Ratio = 9.58;

/* SOGI outputs */
volatile float g_sogi_u_alpha = 0.0f;
volatile float g_sogi_u_beta  = 0.0f;
volatile float g_sogi_v_alpha = 0.0f;
volatile float g_sogi_v_beta  = 0.0f;
volatile float g_sogi_w_alpha = 0.0f;
volatile float g_sogi_w_beta  = 0.0f;

/* PLL variables */
volatile float g_pll_theta = 0.0f;
volatile float g_pll_freq_hz = 50.0f;
volatile float g_pll_error = 0.0f;

/* active reference */
volatile float g_ref_u = 0.0f;
volatile float g_ref_v = 0.0f;
volatile float g_ref_w = 0.0f;

/* error and injection */
volatile float g_err_u = 0.0f;
volatile float g_err_v = 0.0f;
volatile float g_err_w = 0.0f;

volatile float g_inj_u = 0.0f;
volatile float g_inj_v = 0.0f;
volatile float g_inj_w = 0.0f;

/* predictor input, prediction and injection */
volatile float g_pred_input_err_u = 0.0f;
volatile float g_pred_input_err_v = 0.0f;
volatile float g_pred_input_err_w = 0.0f;

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

/* final PWM command */
volatile float g_pwm_u = 0.0f;
volatile float g_pwm_v = 0.0f;
volatile float g_pwm_w = 0.0f;

/* SOGI objects */
static SOGI_Config sogi_cfg;
static SOGI_State  sogi_u_state;
static SOGI_State  sogi_v_state;
static SOGI_State  sogi_w_state;

/* simple PLL internal variables */
static float pll_ts = 0.00005f;         /* 10 kHz */
static float pll_kp = 80.0f;
static float pll_ki = 2000.0f;
static float pll_integrator = 0.0f;
static float pll_omega = 2.0f * 3.1415926f * 50.0f;
static float pll_omega_nominal = 2.0f * 3.1415926f * 50.0f;

volatile float g_direct_output_sign = 1.0f;
volatile float g_injection_output_sign = -1.0f;

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PI_F            3.1415926f
#define TWO_PI_F        (2.0f * PI_F)
#define TWO_PI_OVER_3   (2.0f * PI_F / 3.0f)
#define SQRT3_OVER_2    0.8660254f

#define ADC_VREF        3.3f
#define ADC_MAX_VALUE   4095.0f
#define AMC_GAIN        0.4f
#define RESISTOR_RATIO  100.6f




static float clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static float wrap_pm_pi(float x)
{
    while (x > PI_F)  x -= TWO_PI_F;
    while (x < -PI_F) x += TWO_PI_F;
    return x;
}

static float wrap_0_2pi(float x)
{
    while (x >= TWO_PI_F) x -= TWO_PI_F;
    while (x < 0.0f)      x += TWO_PI_F;
    return x;
}

static float adc_diff_to_volts(uint32_t adc_raw)
{
    float adc_diff_volts;
    float sensor_input_volts;
    float line_volts;

    adc_diff_volts     = (((float)adc_raw / ADC_MAX_VALUE) * 2.0f * ADC_VREF) - ADC_VREF;
    sensor_input_volts = adc_diff_volts / AMC_GAIN;
    line_volts         = sensor_input_volts * RESISTOR_RATIO;

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

/* simple alpha-beta PLL */
/* simple alpha-beta PLL */
/* simple alpha-beta PLL */
static void update_pll(void)
{
    float alpha, beta;
    float cos_th, sin_th;
    float error_volts;
    float max_dev_int, max_dev_out;

		/* Clarke transform with V and W swapped (Negative Sequence Fix) */
		alpha = (2.0f / 3.0f) * (g_sogi_u_alpha - 0.5f * g_sogi_w_alpha - 0.5f * g_sogi_v_alpha);
		beta  = (2.0f / 3.0f) * (SQRT3_OVER_2 * (g_meas_w - g_meas_v));

    /* 2. Calculate Phase Error using Park Transform equivalent
     * This completely avoids atan2f() and its dangerous phase-wrap jumps!
     * Mathematically, this equals: Vpeak * sin(measured_angle - g_pll_theta)
     */
    cos_th = cosf(g_pll_theta);
    sin_th = sinf(g_pll_theta);

    error_volts = (beta * cos_th) - (alpha * sin_th);

    /* 3. Normalize the error so your original Kp and Ki gains work perfectly */
    g_pll_error = -(error_volts / g_clean_reference_phase_peak_volts);

    /* 4. PI Controller with Anti-Windup */
    pll_integrator += pll_ki * pll_ts * g_pll_error;

    max_dev_int = TWO_PI_F * 10.0f;  /* Integrator limit: +/- 10 Hz */
    max_dev_out = TWO_PI_F * 20.0f; /* Total output limit: +/- 20 Hz */

    pll_integrator = clampf(pll_integrator, -max_dev_int, +max_dev_int);

    pll_omega = pll_omega_nominal + (pll_kp * g_pll_error) + pll_integrator;
    pll_omega = clampf(pll_omega, pll_omega_nominal - max_dev_out, pll_omega_nominal + max_dev_out);

    /* 5. Integrate frequency to get angle */
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

    /* * Since the PI controller locked at the opposite equilibrium (+90 deg),
     * subtracting 90 degrees (- PI/2) completely eliminates the 180-degree phase shift 
     * and perfectly aligns the PLL with the grid.
     */
    phase_u_angle = g_pll_theta - (PI_F * 0.5f);  /* Changed from + to - */

    g_ref_u = g_clean_reference_phase_peak_volts * sinf(phase_u_angle);
    
    /* * NEGATIVE SEQUENCE FIX FOR REFERENCE GENERATION *
     * Since the physical grid is U-W-V, we must generate references
     * with the same sequence. So V gets +120 and W gets -120.
     */
    g_ref_v = g_clean_reference_phase_peak_volts * sinf(phase_u_angle + TWO_PI_OVER_3); 
    g_ref_w = g_clean_reference_phase_peak_volts * sinf(phase_u_angle - TWO_PI_OVER_3); 
}

static void build_error(void)
{
    g_err_u = g_ref_u - g_meas_u;
    g_err_v = g_ref_v - g_meas_v;
    g_err_w = g_ref_w - g_meas_w;
}

static void build_injection(void)
{
    /* direct inverted error */
    g_inj_u = -g_err_u;   /* = measured - reference */
    g_inj_v = -g_err_v;
    g_inj_w = -g_err_w;
}

static void build_predictor_injection(void)
{
    const DynVoltPredictor3P_Debug *pred_dbg;
    float pred_err_u;
    float pred_err_v;
    float pred_err_w;

    /*
     * Important:
     * The predictor input is the voltage error, not the measured voltage.
     * First build the raw error: e = reference - measured.
     * Then filter this error and feed the filtered error to the predictor.
     * This avoids creating an artificial 50 Hz error caused by filtering only
     * the measured voltage while keeping the reference unfiltered.
     */
		__disable_irq();
		g_pred_input_err_u = g_err_u;
		g_pred_input_err_v = g_err_v;
		g_pred_input_err_w = g_err_w;
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

    /* Use the same injection polarity convention as direct injection. */
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

static void apply_pwm_output(void)
{
    float out_u;
    float out_v;
    float out_w;
    float sign;

    if (g_output_mode == PWM_MODE_MEASURED_DIRECT)
    {
        /* measured voltage */
        out_u = g_meas_u;
        out_v = g_meas_v;
        out_w = g_meas_w;
        sign = g_direct_output_sign;
    }
    else if (g_output_mode == PWM_MODE_REFERENCE_DIRECT)
    {
        /* reference voltage */
        out_u = g_ref_u;
        out_v = g_ref_v;
        out_w = g_ref_w;
        sign = g_direct_output_sign;
    }
    else if (g_output_mode == PWM_MODE_INJECTION_PREDICTOR)
    {
        /* predictor-based series injection */
        out_u = g_pred_inj_u;
        out_v = g_pred_inj_v;
        out_w = g_pred_inj_w;
        sign = g_injection_output_sign;
    }
    else
    {
        /* direct series injection */
        out_u = g_inj_u;
        out_v = g_inj_v;
        out_w = g_inj_w;
        sign = g_injection_output_sign;
    }

    g_pwm_u = g_pwm_reference_gain * sign * out_u * Transformer_Ratio;
    g_pwm_v = g_pwm_reference_gain * sign * out_v * Transformer_Ratio;
    g_pwm_w = g_pwm_reference_gain * sign * out_w * Transformer_Ratio;
		
		
		
    PWM_3Leg_ApplyPhaseReferences(&g_pwm_3leg,
                                  g_pwm_u,
                                  g_pwm_v,
                                  g_pwm_w,
                                  &g_pwm_3leg_dbg);
}

static void update_selected_reference(void)
{
    update_sogi();

    if (g_reference_mode == REF_MODE_PLL_CLEAN)
    {
        update_pll();
        build_reference_from_pll();
    }
    else
    {
        build_reference_from_sogi();
    }
}


static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void predictor_monitor_step(void)
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

    g_predictor_time_us = ((float)elapsed_cycles) / 170.0f;

    if (g_predictor_time_us > 1500.0f)
    {
        g_predictor_timing_overrun = 1u;
    }

    g_predictor_is_running = 0u;
}

static void control_step(void)
{
    read_adc_voltages();

    update_selected_reference();

    build_error();
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
	HAL_ADC_Stop(&hadc4);
	if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
	{
			Error_Handler();
	}

	/* SOGI init */
	sogi_cfg.Ts = 1.0f / 20000.0f;
	sogi_cfg.k  = 0.8f;
	sogi_cfg.w  = TWO_PI_F * 50.0f;

	SOGI_Init(&sogi_u_state, &sogi_cfg);
	SOGI_Init(&sogi_v_state, &sogi_cfg);
	SOGI_Init(&sogi_w_state, &sogi_cfg);

	/* LPF init for predictor input error */
	DynVolt_LPF3P_Init(&g_predictor_error_lpf, DYNVOLT_LPF_PROFILE_80HZ_20KHZ);
	g_predictor_error_filter_primed = 0u;

	/* Predictor init */
	DynVoltPredictor3P_GetDefaultConfig(&g_predictor_cfg);
	g_predictor_cfg.ts_sec = PREDICTOR_TS_SEC;
	g_predictor_cfg.td_sec = 1.0e-3f;
	g_predictor_cfg.vmax_volts = 80.0f;
	g_predictor_cfg.enable_predictor = 1u;
	g_predictor_cfg.enable_adaptation = 1u;
	DynVoltPredictor3P_Init(&g_predictor, &g_predictor_cfg);

	/* PLL init */
	g_pll_theta = 0.0f;
	pll_integrator = 0.0f;
	pll_omega_nominal = TWO_PI_F * 50.0f;
	pll_omega = pll_omega_nominal;

	/* default modes */
	g_reference_mode = REF_MODE_PLL_CLEAN;
	g_output_mode    = PWM_MODE_INJECTION_PREDICTOR;

	/* start ADC DMA */
	if (HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK)
	{
			Error_Handler();
	}

	/* PWM config */
	g_pwm_3leg_cfg.vdc_volts            = 400.0f;
	g_pwm_3leg_cfg.output_freq_hz       = 50.0f;
	g_pwm_3leg_cfg.pwm_update_hz        = 20000.0f;
	g_pwm_3leg_cfg.modulation_index     = 1.0f;
	g_pwm_3leg_cfg.duty_min             = 0.02f;
	g_pwm_3leg_cfg.duty_max             = 0.98f;
	g_pwm_3leg_cfg.enable_zero_sequence = true;

	PWM_3Leg_Init(&g_pwm_3leg, &htim1, &g_pwm_3leg_cfg);

//	/* start PWM */
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

//	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
//	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
//	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	/* TIM6 only triggers ADC */
	HAL_TIM_Base_Start(&htim6);
	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
			Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		    if (g_predictor_run_request != 0u)
    {
        __disable_irq();
        g_predictor_run_request = 0u;
        __enable_irq();

        predictor_monitor_step();
    }
    

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
        control_step();
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
