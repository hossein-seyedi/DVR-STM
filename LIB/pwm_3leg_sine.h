#ifndef PWM_3LEG_SINE_H
#define PWM_3LEG_SINE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float vdc_volts;             /* DC-link voltage */
    float output_freq_hz;        /* fundamental output frequency, e.g. 50 Hz */
    float pwm_update_hz;         /* update rate of duty cycle, e.g. 10 kHz */
    float modulation_index;      /* 0.0 .. 1.0 */
    float duty_min;              /* safety clamp */
    float duty_max;              /* safety clamp */
    bool  enable_zero_sequence;  /* true = zero-sequence injection */
} PWM_3Leg_Config;

typedef struct
{
    TIM_HandleTypeDef *htim;
    PWM_3Leg_Config cfg;

    uint32_t arr;
    float phase_rad;
    float phase_step_rad;
} PWM_3Leg_Handle;

typedef struct
{
    float theta_rad;

    float va_ref;
    float vb_ref;
    float vc_ref;

    float v_zero;

    float va_cmd;
    float vb_cmd;
    float vc_cmd;

    float duty_a;
    float duty_b;
    float duty_c;

    uint32_t ccr_a;
    uint32_t ccr_b;
    uint32_t ccr_c;
} PWM_3Leg_Debug;

void PWM_3Leg_Init(PWM_3Leg_Handle *h,
                   TIM_HandleTypeDef *htim,
                   const PWM_3Leg_Config *cfg);

void PWM_3Leg_RefreshARR(PWM_3Leg_Handle *h);

void PWM_3Leg_SetOutputFrequency(PWM_3Leg_Handle *h, float output_freq_hz);

void PWM_3Leg_SetModulationIndex(PWM_3Leg_Handle *h, float modulation_index);

void PWM_3Leg_Update(PWM_3Leg_Handle *h, PWM_3Leg_Debug *dbg);
void PWM_3Leg_ApplyPhaseReferences(PWM_3Leg_Handle *h,
                                   float va_ref,
                                   float vb_ref,
                                   float vc_ref,
                                   PWM_3Leg_Debug *dbg);

#endif /* PWM_3LEG_SINE_H */