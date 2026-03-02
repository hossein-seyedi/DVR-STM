#ifndef SOGI_H
#define SOGI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float Ts;      /* sample time [s] */
    float k;       /* SOGI gain (typical 1.0 .. 2.0) */
    float w;       /* angular frequency [rad/s] */
} SOGI_Config;

typedef struct
{
    /* internal states */
    float x1;      /* in-phase state */
    float x2;      /* quadrature integrator state */

    /* last output cache (optional) */
    float v_alpha;
    float v_beta;
    float e;
} SOGI_State;

typedef struct
{
    float v_alpha; /* in-phase (filtered) */
    float v_beta;  /* quadrature (90 deg) */
    float e;       /* error: v_in - v_alpha */
} SOGI_Output;

/* Initialize state */
void SOGI_Init(SOGI_State *st, const SOGI_Config *cfg);

/* Update omega (rad/s) online (if you have w_grid from somewhere) */
static inline void SOGI_SetOmega(SOGI_Config *cfg, float w_rad_s)
{
    cfg->w = w_rad_s;
}

/* One sample step: call exactly once per Ts */
void SOGI_Step(SOGI_State *st,
               const SOGI_Config *cfg,
               float v_in,
               SOGI_Output *out);

#ifdef __cplusplus
}
#endif

#endif /* SOGI_H */