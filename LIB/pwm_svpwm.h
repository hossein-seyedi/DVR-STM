#ifndef PWM_SVPWM_H
#define PWM_SVPWM_H

#include <stdint.h>
#include <stdbool.h>

/* Change this include if you use a different STM32 family */
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float vdc_volts;        /* DC-link voltage [V] (e.g., 800) */
    float k_gain;           /* u = k_gain * error_volts */

    float duty_min;         /* safety clamp (e.g., 0.02) */
    float duty_max;         /* safety clamp (e.g., 0.98) */

    bool  enable_scaling;   /* scale all commands if overmod occurs */
} PWM_SVPWM_Config;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t arr;           /* cached ARR */

    PWM_SVPWM_Config cfg;
} PWM_SVPWM_Handle;

typedef struct
{
    /* Debug / logging outputs */
    float ua, ub, uc;       /* phase commands before zero-seq [V] */
    float temp;             /* zero-seq offset [V] */

    float vaO, vbO, vcO, vnO; /* pole commands after zero-seq [V] */

    float dutyA, dutyB, dutyC, dutyN; /* 0..1 */
    uint32_t ccr1, ccr2, ccr3, ccr4;  /* compare values */
} PWM_SVPWM_Out;

/* Initialize modulator */
void PWM_SVPWM_Init(PWM_SVPWM_Handle *h,
                    TIM_HandleTypeDef *htim1,
                    const PWM_SVPWM_Config *cfg);

/* Update cached ARR (call if you change timer period at runtime) */
void PWM_SVPWM_RefreshARR(PWM_SVPWM_Handle *h);

/* Main function: error(phase-to-neutral) -> PWM on TIM1 CH1..CH4 */
void PWM_SVPWM_ApplyFromError(PWM_SVPWM_Handle *h,
                              float errA_volts,
                              float errB_volts,
                              float errC_volts,
                              PWM_SVPWM_Out *out_opt);

#ifdef __cplusplus
}
#endif

#endif /* PWM_SVPWM_H */