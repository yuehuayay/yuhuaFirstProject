#include "FFTData.h"
#include "stdint.h"
#include "stdio.h"

#define PI 3.1415926569
extern uint16_t gnumObjs;

#define PEAK_NUM              64
// #define PeakPara_PEAK_NUM     512
#define PEAK_NO_PEAK          0
#define PEAK_UNMATCHED        1
#define PEAK_MATCH_WELL       2
#define PEAK_RANGE_MIN              0.3f
#define PEAK_RANGE_MAX              180
#define PEAK_VELOCITY_MIN           -26
#define PEAK_VELOCITY_MAX           26
#define PEAK_ANGLE_MIN              -80
#define PEAK_ANGLE_MAX              80
#define PEAK_RCS_MAX                90


/* ############## Track_CenterPara ##################  */
#define    TRACK_MAX_NORMAL_TRACKS                   32
#define    TRACK_MEAS_NUM                            20


#define     TRACK_EMPTY             0
#define     TRACK_DETECTED          1
#define     TRACK_LOCKED            2

#define     TRACK_HIT                     1
#define     TRACK_MISS                    0
#define     TRACK_HIT_MISS_ARRAY_LENGTH   64

#define     TRACK_LENGTH        5//5
#define     TRACK_WIDTH         6
#define     TRACK_VELOCITY      3.5f //1.5

#define     TRACK_RANGE_GAIN                     0.5f
#define     TRACK_RATE_GAIN                      0.6f
#define     TRACK_ACCEL_GAIN                     0.1f
#define     TRACK_CROSS_RANGE_GAIN               0.02f
#define     TRACK_CROSS_RATE_GAIN                0

#define     TRACK_SORT_NUM               4
#define     TRACK_ANGLE_LENGTH           10

#define     TRACK_LOCK_NUM                6
#define     TRACK_LOCK_COUNTER            4
#define     TRACK_DROP_NUM                10

#define     TRACK_NEW_PEAK_NUM            2
#define     TRACK_MEAS_PEAK_NUM           1
#define     TRACK_COVMAT_NUM              3
#define     TRACK_COVMAT_PARA             2
#define     TRACK_SPREAD_NUM              3

#define     TRACK_DROP_RANGE                0.5f
#define     TRACK_DROP_VELOCITYE_MIN        -240
#define     TRACK_DROP_VELOCITYE_MAX        240

#define     LIFE_TIME_MIN                50


typedef struct
{
    uint8_t Status[PEAK_NUM];
    float Range[PEAK_NUM];
    float Velocity[PEAK_NUM];
    float Azimuth[PEAK_NUM];
    float Elev[PEAK_NUM];
    float DistX[PEAK_NUM];
    float DistY[PEAK_NUM];
    float DistZ[PEAK_NUM];
    float VrelX[PEAK_NUM];
    float VrelY[PEAK_NUM];
    float VrelZ[PEAK_NUM];
    float Snr[PEAK_NUM];
    float angle[PEAK_NUM];
    uint16_t  Num;
    float  CrossRange_Station[PEAK_NUM];

} PeakListS;



typedef struct TrackCenterListStruct
{
    uint8_t    Status[TRACK_MAX_NORMAL_TRACKS];
    uint32_t   ID[TRACK_MAX_NORMAL_TRACKS];

    float Range[TRACK_MAX_NORMAL_TRACKS];
    float Velocity[TRACK_MAX_NORMAL_TRACKS];
    float Azimuth[TRACK_MAX_NORMAL_TRACKS];
    float Elev[TRACK_MAX_NORMAL_TRACKS];
    float Accel[TRACK_MAX_NORMAL_TRACKS];
    float RCS[TRACK_MAX_NORMAL_TRACKS];

    float DistX[TRACK_MAX_NORMAL_TRACKS];
    float DistY[TRACK_MAX_NORMAL_TRACKS];
    float DistZ[TRACK_MAX_NORMAL_TRACKS];
    float VeloX[TRACK_MAX_NORMAL_TRACKS];
    float VeloY[TRACK_MAX_NORMAL_TRACKS];
    float VeloZ[TRACK_MAX_NORMAL_TRACKS];
    float AccX[TRACK_MAX_NORMAL_TRACKS];
    float AccY[TRACK_MAX_NORMAL_TRACKS];
    float AccZ[TRACK_MAX_NORMAL_TRACKS];

    uint8_t AngleBufLength[TRACK_MAX_NORMAL_TRACKS];
    float AngleBuf[TRACK_MAX_NORMAL_TRACKS][TRACK_ANGLE_LENGTH];

    uint8_t ElevAngleBufLength[TRACK_MAX_NORMAL_TRACKS];
    float EleAngleBuf[TRACK_MAX_NORMAL_TRACKS][TRACK_ANGLE_LENGTH];

    uint8_t DistYFilterLength[TRACK_MAX_NORMAL_TRACKS];
    float DistYFilterBuf[TRACK_MAX_NORMAL_TRACKS][TRACK_ANGLE_LENGTH];


    float Length[TRACK_MAX_NORMAL_TRACKS];
    float Width[TRACK_MAX_NORMAL_TRACKS];
    float VelocityCorr[TRACK_MAX_NORMAL_TRACKS];

    uint8_t CountNum[TRACK_MAX_NORMAL_TRACKS];
    uint32_t MeasIndex[TRACK_MAX_NORMAL_TRACKS][TRACK_MEAS_NUM];

    uint8_t HitMissLength[TRACK_MAX_NORMAL_TRACKS];
    uint8_t HitMiss[TRACK_MAX_NORMAL_TRACKS][TRACK_HIT_MISS_ARRAY_LENGTH];
    uint8_t MissNum[TRACK_MAX_NORMAL_TRACKS];
    uint8_t LifeTime[TRACK_MAX_NORMAL_TRACKS];
    uint8_t trackqualitylife[TRACK_MAX_NORMAL_TRACKS];
    uint8_t Type[TRACK_MAX_NORMAL_TRACKS];
    uint8_t Reserved[TRACK_MAX_NORMAL_TRACKS];

    float Est_Spread[TRACK_MAX_NORMAL_TRACKS][TRACK_SPREAD_NUM];
    float CovMat[TRACK_MAX_NORMAL_TRACKS][TRACK_COVMAT_NUM];

} TrackListS;


extern DPIF_PointCloud target[MAXNUM_OF_DETECTION];

void dataprocess();

void peak_get();

void Peak_Delete();

void track_proc();

void Track_Propagate();

void MatchMeas();

void UpdateTracks();

void Track_UpdateHitMiss(uint32_t TrackIndex, uint8_t Flag);

uint32_t Track_CountHitMiss(uint32_t TrackIndex, uint16_t N);

void Track_Gettrackqualitylife(uint32_t TrackIndex);

void FilterTrack(uint32_t TrackIndex);

void Track_AddNew();

void Spherical_To_Cartesian(float* z, float* x, float* y, float r, float elve, float azimuth);