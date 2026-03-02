#include "sogi.h"

/*
  Standard continuous-time SOGI (common form):

    e = v_in - x1

    x1' = -k*w*x1 - w^2*x2 + k*w*v_in
    x2' = x1

    v_alpha = x1
    v_beta  = w*x2   (this makes the transfer exactly:
                      Q(s) = k*w^2 / (s^2 + k*w*s + w^2) )

  Discretization used here:
    Forward Euler (sample-by-sample, fast, common for high Fs).
    If you need exact match with a Simulink discretization method (Tustin/ZOH),
    we can swap this section to that exact method.
*/

void SOGI_Init(SOGI_State *st, const SOGI_Config *cfg)
{
    (void)cfg;
    if (!st) return;

    st->x1 = 0.0f;
    st->x2 = 0.0f;

    st->v_alpha = 0.0f;
    st->v_beta  = 0.0f;
    st->e       = 0.0f;
}

void SOGI_Step(SOGI_State *st,
               const SOGI_Config *cfg,
               float v_in,
               SOGI_Output *out)
{
    if (!st || !cfg || !out) return;

    const float Ts = cfg->Ts;
    const float w  = cfg->w;
    const float k  = cfg->k;

    /* error (standard SOGI internal signal) */
    const float e = v_in - st->x1;

    /* state derivatives (continuous model) */
    const float x1_dot = (-k * w * st->x1) - (w * w * st->x2) + (k * w * v_in);
    const float x2_dot = st->x1;

    /* discrete update (Euler) */
    st->x1 += Ts * x1_dot;
    st->x2 += Ts * x2_dot;

    /* outputs */
    const float v_alpha = st->x1;
    const float v_beta  = w * st->x2;

    st->v_alpha = v_alpha;
    st->v_beta  = v_beta;
    st->e       = e;

    out->v_alpha = v_alpha;
    out->v_beta  = v_beta;
    out->e       = e;
}