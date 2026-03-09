#ifndef SOGI_H
#define SOGI_H

/* Structure to hold the state and coefficients of the SOGI filter */
typedef struct {
    /* Memory of previous inputs (n-1 and n-2) */
    float v_in_1;
    float v_in_2;

    /* Memory of previous alpha outputs */
    float v_alpha_1;
    float v_alpha_2;

    /* Memory of previous beta outputs */
    float v_beta_1;
    float v_beta_2;

    /* IIR Filter coefficients for V_alpha (Band-pass) */
    float b0_a;
    float b1_a;
    float b2_a;

    /* IIR Filter coefficients for V_beta (Low-pass) */
    float b0_b;
    float b1_b;
    float b2_b;

    /* Common denominator coefficients (Poles) */
    float a1;
    float a2;

    /* Current outputs */
    float v_alpha;
    float v_beta;
    float e;
} SOGI_State;

/* Structure for SOGI configuration parameters */
typedef struct {
    float Ts;  /* Sampling time in seconds (e.g., 0.0001 for 10kHz) */
    float w;   /* Grid angular frequency (e.g., 2.0f * PI * 50.0f) */
    float k;   /* Damping factor (typically sqrt(2) ~ 1.414) */
} SOGI_Config;

/* Structure for returning the outputs */
typedef struct {
    float v_alpha; /* In-phase component (filtered grid voltage) */
    float v_beta;  /* Quadrature component (90-degree shifted) */
    float e;       /* Error signal */
} SOGI_Output;

/* Function Prototypes */
void SOGI_Init(SOGI_State *st, const SOGI_Config *cfg);
void SOGI_Step(SOGI_State *st, float v_in, SOGI_Output *out);

#endif /* SOGI_H */