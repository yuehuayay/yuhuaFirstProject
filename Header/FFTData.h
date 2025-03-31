#ifndef FFTDATA_H
#define FFTDATA_H
#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include "senserconfig.h"
#include "win_api.h"

#define NUM_RANGE_CUBE         (128*32)
#define PI                     (3.1415926)
#define DF_TRACKTREE_MAX_NUM    (32)
//float rangefft_temp[RANGE_FFT_SIZE * 2];
// q15_t rangefft_temp[RANGE_FFT_SIZE * 2];

float rangefft[RANGE_FFT_SIZE  * DOPPLER_FFT_SIZE* NumchannelMimo * 2]; // rangfft结果为复数
float RD_Map[RANGE_FFT_SIZE  * DOPPLER_FFT_SIZE]; // ABS结果为实数
float RD_Map_ALL[RANGE_FFT_SIZE * DOPPLER_FFT_SIZE]; // ABS结果为实数
float RD_Map_Dopplerffttemp[RANGE_FFT_SIZE * DOPPLER_FFT_SIZE * 2];
Complexfloat gRadarCubeTemp[RANGE_FFT_SIZE];
Complexfloat gRadarCubeDTemp[DOPPLER_FFT_SIZE];
extern uint16_t gnumObjs;

typedef struct target_infov2_t
{
    uint32_t framenum;
    uint8_t  id;
    uint8_t  state; /* 0:无目标， 1：有目标 */
    float    speed;
    float    mag;
    float    noise;
    float    x;
    float    y;
} target_infov2_t;

typedef struct
{
    uint8_t state;
    uint8_t rangeIdx;
    uint8_t dopplerIdx;
    float    strength;
    float    noise1;
    float    noise2;
    float    range;
    float    velocity;
    float    Angle;
    float    channel_data[2*2];
} DPIF_PointCloud;

typedef enum en_fft_type
{
    FFT_IQ = 0u,         /*!< USE IQ */
    FFT_IOnly = 1u,         /*!< I ONLY */
    FFT_QOnly = 2u,         /*!< Q ONLY */
}en_fft_type_t;

typedef struct
{
    uint16_t fftLen;                   /**< length of the FFT. */
    const float * pTwiddle;         /**< points to the Twiddle factor table. */
    const uint16_t* pBitRevTable;      /**< points to the bit reversal table. */
    uint16_t bitRevLength;             /**< bit reversal table length. */
} arm_cfft_instance_f32;

extern const float f32WindowBlackmanharris128[];
extern const float f32WindowBlackmanharris256[];
extern const float f32WindowHanning64[];
extern const float f32WindowHanning32[];

#endif // FFTDATA_H