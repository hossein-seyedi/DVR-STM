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
#include "sogi.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


#define ADC_BUF_LEN  3

/* Differential ADC result should be treated as signed */
volatile int16_t adc_buf[ADC_BUF_LEN];

/* Raw (counts) per phase */
static volatile int16_t g_adc_raw_V = 0;
static volatile int16_t g_adc_raw_W = 0;
static volatile int16_t g_adc_raw_U = 0;

/* Instant (unfiltered) input to SOGI (in volts at ADC pins) */
static volatile float g_v_in_V = 0.0f;
static volatile float g_v_in_W = 0.0f;
static volatile float g_v_in_U = 0.0f;

/* Filtered outputs (v_alpha) per phase */
static volatile float g_v_alpha_V = 0.0f;
static volatile float g_v_alpha_W = 0.0f;
static volatile float g_v_alpha_U = 0.0f;

/* Optional: keep v_beta and e too (useful for debug) */
static volatile float g_v_beta_V  = 0.0f;
static volatile float g_v_beta_W  = 0.0f;
static volatile float g_v_beta_U  = 0.0f;

static volatile float g_e_V       = 0.0f;
static volatile float g_e_W       = 0.0f;
static volatile float g_e_U       = 0.0f;
/* SOGI instances (one per phase) */
static SOGI_Config sogi_cfg;
static SOGI_State  sogi_st_V;
static SOGI_State  sogi_st_W;
static SOGI_State  sogi_st_U;




float Vu_inst ;
float Vv_inst;
float Vw_inst ;

float Vu_filt;
float Vv_filt;
float Vw_filt ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
    float v_in;     /* unfiltered instantaneous */
    float v_alpha;  /* filtered in-phase */
    float v_beta;   /* 90deg component */
    float e;        /* v_in - v_alpha */
} SOGI_PhaseOut;

typedef struct
{
    SOGI_PhaseOut U;
    SOGI_PhaseOut V;
    SOGI_PhaseOut W;
} SOGI_3PhaseOut;



/* Get latest snapshot (safe copy) */
static SOGI_3PhaseOut SOGI_GetLast3(void);

/* ---- ADC differential counts -> volts at ADC pins ----
   For STM32G4 differential mode (12-bit):
   code range is typically -2048..+2047 (signed).
   LSB ˜ Vref / 2048.

   If your Vref is not 3.3V, change it here.
*/
#define ADC_VREF_VOLTS      (3.3f)
#define ADC_DIFF_FULLSCALE  (2048.0f)   /* 2^(12-1) */

static inline float adc_diff_to_volts(int16_t raw_diff)
{
    /* volts at ADC input pins (OUTP-OUTN) */
    return ((float)raw_diff) * (ADC_VREF_VOLTS / ADC_DIFF_FULLSCALE);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	//diff mode Calib

	HAL_ADC_Stop(&hadc4);
	if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
	{
			Error_Handler();
	}

	/* SOGI config (10 kHz sample, 50 Hz) */
	sogi_cfg.Ts = 1.0f / 10000.0f;               // TIM6 = 10kHz
	sogi_cfg.k  = 1.0f;
	sogi_cfg.w  = 2.0f * 3.1415926f * 50.0f;

	/* Init three SOGIs */
	SOGI_Init(&sogi_st_V, &sogi_cfg);
	SOGI_Init(&sogi_st_W, &sogi_cfg);
	SOGI_Init(&sogi_st_U, &sogi_cfg);

	HAL_TIM_Base_Start(&htim6);

	/* DMA length must match Number of Conversions = 3 */
	if (HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK)
	{
			Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    
		SOGI_3PhaseOut o = SOGI_GetLast3();

     Vu_inst = o.U.v_in;
     Vv_inst = o.V.v_in;
     Vw_inst = o.W.v_in;
		
     Vu_filt = o.U.v_alpha;
     Vv_filt = o.V.v_alpha;
     Vw_filt = o.W.v_alpha;

    (void)Vu_inst; (void)Vv_inst; (void)Vw_inst;
    (void)Vu_filt; (void)Vv_filt; (void)Vw_filt;
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
        /* DMA order = rank order */
        int16_t raw_V = adc_buf[0];   // Rank1 Channel5  -> V
        int16_t raw_W = adc_buf[1];   // Rank2 Channel1  -> W
        int16_t raw_U = adc_buf[2];   // Rank3 Channel3  -> U

        g_adc_raw_V = raw_V;
        g_adc_raw_W = raw_W;
        g_adc_raw_U = raw_U;

        /* Convert to volts (at ADC pins, differential) */
        float v_in_V = adc_diff_to_volts(raw_V);
        float v_in_W = adc_diff_to_volts(raw_W);
        float v_in_U = adc_diff_to_volts(raw_U);

        g_v_in_V = v_in_V;
        g_v_in_W = v_in_W;
        g_v_in_U = v_in_U;

        /* Run 3 independent SOGIs */
        SOGI_Output y;

        SOGI_Step(&sogi_st_V, &sogi_cfg, v_in_V, &y);
        g_v_alpha_V = y.v_alpha;  g_v_beta_V = y.v_beta;  g_e_V = y.e;

        SOGI_Step(&sogi_st_W, &sogi_cfg, v_in_W, &y);
        g_v_alpha_W = y.v_alpha;  g_v_beta_W = y.v_beta;  g_e_W = y.e;

        SOGI_Step(&sogi_st_U, &sogi_cfg, v_in_U, &y);
        g_v_alpha_U = y.v_alpha;  g_v_beta_U = y.v_beta;  g_e_U = y.e;
    }
}

static SOGI_3PhaseOut SOGI_GetLast3(void)
{
    SOGI_3PhaseOut out;

    __disable_irq();

    out.V.v_in    = g_v_in_V;
    out.V.v_alpha = g_v_alpha_V;
    out.V.v_beta  = g_v_beta_V;
    out.V.e       = g_e_V;

    out.W.v_in    = g_v_in_W;
    out.W.v_alpha = g_v_alpha_W;
    out.W.v_beta  = g_v_beta_W;
    out.W.e       = g_e_W;

    out.U.v_in    = g_v_in_U;
    out.U.v_alpha = g_v_alpha_U;
    out.U.v_beta  = g_v_beta_U;
    out.U.e       = g_e_U;

    __enable_irq();

    return out;
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
