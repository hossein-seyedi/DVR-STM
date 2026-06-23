#ifndef DYNVOLT_LPF_H
#define DYNVOLT_LPF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYNVOLT_LPF_STAGE_COUNT  3u

typedef enum
{
    DYNVOLT_LPF_PROFILE_80HZ_20KHZ = 0,
    DYNVOLT_LPF_PROFILE_COUNT
} DynVolt_LPF_Profile;

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    float z1;
    float z2;
} DynVolt_BiquadF32;

typedef struct
{
    DynVolt_BiquadF32 stage[DYNVOLT_LPF_STAGE_COUNT];

    float last_input;
    float last_output;
} DynVolt_LPF_ChannelF32;

typedef struct
{
    DynVolt_LPF_ChannelF32 u;
    DynVolt_LPF_ChannelF32 v;
    DynVolt_LPF_ChannelF32 w;

    DynVolt_LPF_Profile profile;

    float sample_rate_hz;
    float passband_hz;
} DynVolt_LPF3P_F32;

void DynVolt_LPF3P_Init(DynVolt_LPF3P_F32 *f,
                        DynVolt_LPF_Profile profile);

void DynVolt_LPF3P_Reset(DynVolt_LPF3P_F32 *f);

void DynVolt_LPF3P_Prime(DynVolt_LPF3P_F32 *f,
                         float u0,
                         float v0,
                         float w0);

void DynVolt_LPF3P_Step(DynVolt_LPF3P_F32 *f,
                        float u_in,
                        float v_in,
                        float w_in,
                        float *u_out,
                        float *v_out,
                        float *w_out);

float DynVolt_LPF3P_GetSampleRateHz(const DynVolt_LPF3P_F32 *f);
float DynVolt_LPF3P_GetPassbandHz(const DynVolt_LPF3P_F32 *f);

#ifdef __cplusplus
}
#endif

#endif