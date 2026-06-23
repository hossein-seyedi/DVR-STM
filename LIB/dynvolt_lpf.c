#include "dynvolt_lpf.h"

/*
 * DynVolt input lowpass filter
 *
 * Design:
 *   Type: Elliptic IIR lowpass
 *   Order: 6
 *   Fs: 20000 Hz
 *   Passband edge: 80 Hz
 *   Passband ripple: 0.05 dB
 *   Stopband attenuation: 80 dB
 *
 * Implementation:
 *   3 cascaded second-order sections
 *   Direct Form II Transposed
 *
 * Difference equation per section:
 *
 *   y[n]  = b0*x[n] + z1
 *   z1[n] = b1*x[n] - a1*y[n] + z2
 *   z2[n] = b2*x[n] - a2*y[n]
 *
 * Denominator format:
 *
 *   1 + a1*z^-1 + a2*z^-2
 */

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} DynVolt_BiquadCoeffF32;

typedef struct
{
    DynVolt_LPF_Profile profile;
    float sample_rate_hz;
    float passband_hz;
    const DynVolt_BiquadCoeffF32 *sos;
} DynVolt_LPF_ProfileDef;

static float DynVolt_LPF_AbsF32(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}

/*
 * SOS coefficients for:
 *   Elliptic lowpass, order 6
 *   Fs = 20000 Hz
 *   Fpass = 80 Hz
 *   Rp = 0.05 dB
 *   Rs = 80 dB
 */
static const DynVolt_BiquadCoeffF32 dynvolt_lpf_80hz_20khz_sos[DYNVOLT_LPF_STAGE_COUNT] =
{
    {
        9.883085774242810e-05f,
       -1.934893889644481e-04f,
        9.883085774242808e-05f,
       -1.974122214252051e+00f,
        9.743517395121338e-01f
    },
    {
        1.000000000000000e+00f,
       -1.994011004772570e+00f,
        9.999999999999996e-01f,
       -1.982424448104685e+00f,
        9.829244671928126e-01f
    },
    {
        1.000000000000000e+00f,
       -1.996634061451189e+00f,
        9.999999999999999e-01f,
       -1.993550504932647e+00f,
        9.942875963703487e-01f
    }
};

static const DynVolt_LPF_ProfileDef dynvolt_lpf_profile_table[DYNVOLT_LPF_PROFILE_COUNT] =
{
    {
        DYNVOLT_LPF_PROFILE_80HZ_20KHZ,
        20000.0f,
        80.0f,
        dynvolt_lpf_80hz_20khz_sos
    }
};

static void DynVolt_Biquad_Load(DynVolt_BiquadF32 *s,
                                const DynVolt_BiquadCoeffF32 *c)
{
    if ((s == 0) || (c == 0))
    {
        return;
    }

    s->b0 = c->b0;
    s->b1 = c->b1;
    s->b2 = c->b2;
    s->a1 = c->a1;
    s->a2 = c->a2;

    s->z1 = 0.0f;
    s->z2 = 0.0f;
}

static void DynVolt_Biquad_Reset(DynVolt_BiquadF32 *s)
{
    if (s == 0)
    {
        return;
    }

    s->z1 = 0.0f;
    s->z2 = 0.0f;
}

static float DynVolt_Biquad_Step(DynVolt_BiquadF32 *s, float x)
{
    float y;

    if (s == 0)
    {
        return x;
    }

    y = (s->b0 * x) + s->z1;

    s->z1 = (s->b1 * x) - (s->a1 * y) + s->z2;
    s->z2 = (s->b2 * x) - (s->a2 * y);

    return y;
}

static float DynVolt_Biquad_PrimeDC(DynVolt_BiquadF32 *s, float x)
{
    float num;
    float den;
    float gain_dc;
    float y;

    if (s == 0)
    {
        return x;
    }

    num = s->b0 + s->b1 + s->b2;
    den = 1.0f + s->a1 + s->a2;

    if (DynVolt_LPF_AbsF32(den) < 1.0e-12f)
    {
        gain_dc = 1.0f;
    }
    else
    {
        gain_dc = num / den;
    }

    y = gain_dc * x;

    /*
     * Internal-state initialization for constant input x.
     * This prevents a large zero-state startup transient.
     */
    s->z1 = y - (s->b0 * x);
    s->z2 = (s->b2 * x) - (s->a2 * y);

    return y;
}

static void DynVolt_LPF_Channel_Init(DynVolt_LPF_ChannelF32 *ch,
                                     const DynVolt_BiquadCoeffF32 *sos)
{
    uint32_t i;

    if ((ch == 0) || (sos == 0))
    {
        return;
    }

    for (i = 0u; i < DYNVOLT_LPF_STAGE_COUNT; i++)
    {
        DynVolt_Biquad_Load(&ch->stage[i], &sos[i]);
    }

    ch->last_input = 0.0f;
    ch->last_output = 0.0f;
}

static void DynVolt_LPF_Channel_Reset(DynVolt_LPF_ChannelF32 *ch)
{
    uint32_t i;

    if (ch == 0)
    {
        return;
    }

    for (i = 0u; i < DYNVOLT_LPF_STAGE_COUNT; i++)
    {
        DynVolt_Biquad_Reset(&ch->stage[i]);
    }

    ch->last_input = 0.0f;
    ch->last_output = 0.0f;
}

static float DynVolt_LPF_Channel_Prime(DynVolt_LPF_ChannelF32 *ch, float x0)
{
    uint32_t i;
    float y;

    if (ch == 0)
    {
        return x0;
    }

    y = x0;

    for (i = 0u; i < DYNVOLT_LPF_STAGE_COUNT; i++)
    {
        y = DynVolt_Biquad_PrimeDC(&ch->stage[i], y);
    }

    ch->last_input = x0;
    ch->last_output = y;

    return y;
}

static float DynVolt_LPF_Channel_Step(DynVolt_LPF_ChannelF32 *ch, float x)
{
    uint32_t i;
    float y;

    if (ch == 0)
    {
        return x;
    }

    y = x;

    for (i = 0u; i < DYNVOLT_LPF_STAGE_COUNT; i++)
    {
        y = DynVolt_Biquad_Step(&ch->stage[i], y);
    }

    ch->last_input = x;
    ch->last_output = y;

    return y;
}

void DynVolt_LPF3P_Init(DynVolt_LPF3P_F32 *f,
                        DynVolt_LPF_Profile profile)
{
    const DynVolt_LPF_ProfileDef *profile_def;

    if (f == 0)
    {
        return;
    }

    if ((uint32_t)profile >= (uint32_t)DYNVOLT_LPF_PROFILE_COUNT)
    {
        profile = DYNVOLT_LPF_PROFILE_80HZ_20KHZ;
    }

    profile_def = &dynvolt_lpf_profile_table[(uint32_t)profile];

    DynVolt_LPF_Channel_Init(&f->u, profile_def->sos);
    DynVolt_LPF_Channel_Init(&f->v, profile_def->sos);
    DynVolt_LPF_Channel_Init(&f->w, profile_def->sos);

    f->profile = profile_def->profile;
    f->sample_rate_hz = profile_def->sample_rate_hz;
    f->passband_hz = profile_def->passband_hz;
}

void DynVolt_LPF3P_Reset(DynVolt_LPF3P_F32 *f)
{
    if (f == 0)
    {
        return;
    }

    DynVolt_LPF_Channel_Reset(&f->u);
    DynVolt_LPF_Channel_Reset(&f->v);
    DynVolt_LPF_Channel_Reset(&f->w);
}

void DynVolt_LPF3P_Prime(DynVolt_LPF3P_F32 *f,
                         float u0,
                         float v0,
                         float w0)
{
    if (f == 0)
    {
        return;
    }

    (void)DynVolt_LPF_Channel_Prime(&f->u, u0);
    (void)DynVolt_LPF_Channel_Prime(&f->v, v0);
    (void)DynVolt_LPF_Channel_Prime(&f->w, w0);
}

void DynVolt_LPF3P_Step(DynVolt_LPF3P_F32 *f,
                        float u_in,
                        float v_in,
                        float w_in,
                        float *u_out,
                        float *v_out,
                        float *w_out)
{
    float u_y;
    float v_y;
    float w_y;

    if (f == 0)
    {
        if (u_out != 0)
        {
            *u_out = u_in;
        }

        if (v_out != 0)
        {
            *v_out = v_in;
        }

        if (w_out != 0)
        {
            *w_out = w_in;
        }

        return;
    }

    u_y = DynVolt_LPF_Channel_Step(&f->u, u_in);
    v_y = DynVolt_LPF_Channel_Step(&f->v, v_in);
    w_y = DynVolt_LPF_Channel_Step(&f->w, w_in);

    if (u_out != 0)
    {
        *u_out = u_y;
    }

    if (v_out != 0)
    {
        *v_out = v_y;
    }

    if (w_out != 0)
    {
        *w_out = w_y;
    }
}

float DynVolt_LPF3P_GetSampleRateHz(const DynVolt_LPF3P_F32 *f)
{
    if (f == 0)
    {
        return 0.0f;
    }

    return f->sample_rate_hz;
}

float DynVolt_LPF3P_GetPassbandHz(const DynVolt_LPF3P_F32 *f)
{
    if (f == 0)
    {
        return 0.0f;
    }

    return f->passband_hz;
}