#include "sogi.h"

/**
  * @brief  Initializes the SOGI filter state and calculates Tustin coefficients.
  * This function should be called once before entering the control loop.
  * @param  st: Pointer to the SOGI state structure.
  * @param  cfg: Pointer to the SOGI configuration structure.
  */
void SOGI_Init(SOGI_State *st, const SOGI_Config *cfg)
{
    if (!st || !cfg) return;

    /* Clear memory buffers (shift registers) */
    st->v_in_1    = 0.0f; 
    st->v_in_2    = 0.0f;
    st->v_alpha_1 = 0.0f; 
    st->v_alpha_2 = 0.0f;
    st->v_beta_1  = 0.0f; 
    st->v_beta_2  = 0.0f;
    
    /* Clear initial outputs */
    st->v_alpha   = 0.0f; 
    st->v_beta    = 0.0f; 
    st->e         = 0.0f;

    /* * Calculate intermediate values for the Tustin (Bilinear) transform.
     * These pre-calculated coefficients prevent heavy math inside the ISR.
     */
    float x = 2.0f * cfg->k * cfg->w * cfg->Ts;
    float y = (cfg->w * cfg->Ts) * (cfg->w * cfg->Ts);
    float den = 4.0f + x + y;

    /* V_alpha (Band-pass) filter coefficients */
    st->b0_a = x / den;
    st->b1_a = 0.0f;
    st->b2_a = -x / den;

    /* V_beta (Low-pass) filter coefficients */
    st->b0_b = (cfg->k * y) / den;
    st->b1_b = (2.0f * cfg->k * y) / den;
    st->b2_b = (cfg->k * y) / den;

    /* Common denominator coefficients (System Poles) */
    st->a1 = (2.0f * y - 8.0f) / den;
    st->a2 = (4.0f - x + y) / den;
}

/**
  * @brief  Executes one calculation step of the SOGI filter.
  * This function should be called at the exact sampling frequency (Ts).
  * @param  st: Pointer to the SOGI state structure.
  * @param  v_in: The current sampled grid voltage.
  * @param  out: Pointer to the structure where outputs will be stored.
  */
void SOGI_Step(SOGI_State *st, float v_in, SOGI_Output *out)
{
    if (!st || !out) return;

    /* * Difference equation for V_alpha (In-phase component) 
     * Acts as a highly selective band-pass filter at frequency 'w'.
     */
    float v_alpha = (st->b0_a * v_in) + (st->b1_a * st->v_in_1) + (st->b2_a * st->v_in_2)
                  - (st->a1 * st->v_alpha_1) - (st->a2 * st->v_alpha_2);

    /* * Difference equation for V_beta (Quadrature component) 
     * Acts as a low-pass filter, resulting in a 90-degree phase shift.
     */
    float v_beta  = (st->b0_b * v_in) + (st->b1_b * st->v_in_1) + (st->b2_b * st->v_in_2)
                  - (st->a1 * st->v_beta_1) - (st->a2 * st->v_beta_2);

    /* Calculate SOGI extraction error */
    float e = v_in - v_alpha;

    /* * Update shift registers for the next execution cycle 
     * Note: Older values must be updated first!
     */
    st->v_in_2 = st->v_in_1;
    st->v_in_1 = v_in;

    st->v_alpha_2 = st->v_alpha_1;
    st->v_alpha_1 = v_alpha;

    st->v_beta_2 = st->v_beta_1;
    st->v_beta_1 = v_beta;

    /* Store current outputs in the state structure */
    st->v_alpha = v_alpha;
    st->v_beta  = v_beta;
    st->e       = e;

    /* Assign values to the output structure */
    out->v_alpha = v_alpha;
    out->v_beta  = v_beta;
    out->e       = e;
}