#include "..\Header\data__process.h"
#include <stdint.h>
#include <stdbool.h>

//DPIF_PointCloud target[MAXNUM_OF_DETECTION];
PeakListS PeakList;
//
TrackListS TrackList;

#define RadarCycleTime 0.08f

#define RadarInstallAngle 0

#define workmode 2

void dataprocess()
{

    // UART_LOG("000");

    peak_get();
    Peak_Delete();//删除范围外的点
    track_proc();

    // for (uint16_t jj = 0; jj < TRACK_MAX_NORMAL_TRACKS; jj++)
    // {
    //     if (TrackList.Status[jj] == TRACK_LOCKED)
    //     {
    //         UART_LOG("framenum =%d,track_idx =%d, r=%.1f,s=%.1f,mr=%.0f\r\n",
    //                  framenum,
    //                  jj,
    //                  TrackList.DistX[jj],
    //                  TrackList.VeloX[jj],
    //                  TrackList.RCS[jj]); // distance单位：厘米
    //     }
    // }

    // UART_LOG("888");
}

uint32_t test1 = 0;
void     peak_get()
{

    uint16_t i = 0;

    memset(&PeakList, 0, sizeof(PeakListS));

    //-------------临时测试------------------
    //   gnumObjs = 3;

    //   target[0].range = 10 + 10 *RadarCycleTime * test1;
    //   target[0].velocity = 10;
    //   target[0].strength = 15;

    //   target[1].range = 11 + 9.8 * RadarCycleTime * test1;
    //   target[1].velocity = 9.8;
    //   target[1].strength = 10;

    //   target[2].range = 5 + 15 * RadarCycleTime * test1;
    //   target[2].velocity = 15;
    //   target[2].strength = 30;

    //   test1++;
    //   if (test1>=100)
    //   {
    //     test1 = 0;
    //   }
    //---------------------------------------

    PeakList.Num = gnumObjs; // 目标点数

    for (i = 0; i < PeakList.Num; i++)
    {
        PeakList.Status[i] = PEAK_UNMATCHED;
        PeakList.Range[i] = target[i].range;
        PeakList.Velocity[i] = target[i].velocity;
        PeakList.Snr[i] = target[i].strength;
        PeakList.Azimuth[i] = target[i].Angle;

        Spherical_To_Cartesian(&PeakList.DistZ[i],
            &PeakList.DistX[i],
            &PeakList.DistY[i],
            PeakList.Range[i],
            PeakList.Elev[i],
            PeakList.Azimuth[i]);
        Spherical_To_Cartesian(&PeakList.VrelZ[i],
            &PeakList.VrelX[i],
            &PeakList.VrelY[i],
            PeakList.Velocity[i],
            PeakList.Elev[i],
            PeakList.Azimuth[i]);
    }
}

void Peak_Delete()
{
    uint16_t i;
    uint16_t j;
    for (i = 0; i < PeakList.Num; i++)
    {

        if (workmode == 1)
        {

            if (PeakList.Status[i] == PEAK_UNMATCHED)
            {
                if ((PeakList.Range[i] < PEAK_RANGE_MIN) || (PeakList.Range[i] > PEAK_RANGE_MAX) ||
                    (PeakList.Velocity[i] < PEAK_VELOCITY_MIN) || (PeakList.Velocity[i] > PEAK_VELOCITY_MAX) ||
                    ((PeakList.Velocity[i] > -0.85f) && (PeakList.Velocity[i] < 0.85f)) // 删除速度为0的点lj20221009

                    )
                {
                    PeakList.Status[i] = PEAK_NO_PEAK;
                }
            }
        }

        else if (workmode == 2)
        {

            if (PeakList.Status[i] == PEAK_UNMATCHED)
            {
                if ((PeakList.Range[i] < PEAK_RANGE_MIN) || (PeakList.Range[i] > PEAK_RANGE_MAX) ||
                    (PeakList.Velocity[i] < PEAK_VELOCITY_MIN) || (PeakList.Velocity[i] > 0.1f) ||
                    ((PeakList.Velocity[i] > -1.0f) && (PeakList.Velocity[i] < 1.0f)) // 删除速度为0的点lj20221009

                    )
                {
                    PeakList.Status[i] = PEAK_NO_PEAK;
                }
            }
        }

        else
        {
        }
    }
    if (workmode == 2)
    {

        for (i = 0; i < PeakList.Num; i++)
        {
            {
                for (j = i + 1; j < PeakList.Num; j++)
                {
                    if (i != j)
                    {
                        if ((fabsf(PeakList.Range[j] - PeakList.Range[i]) < 2.0f) &&
                            (fabsf(fabsf(PeakList.Velocity[j] - PeakList.Velocity[i]) - 28.0f) < 1.2f))
                        {
                            PeakList.Status[i] = PEAK_NO_PEAK;
                            PeakList.Status[j] = PEAK_NO_PEAK;
                        }
                    }
                }
            }
        }
    }
}

void track_proc()
{
    Track_Propagate();
    MatchMeas(); //
    UpdateTracks();
    Track_AddNew();
}

void Track_Propagate()
{
    uint32_t i;

    for (i = 0; i < TRACK_MAX_NORMAL_TRACKS; i++)
    {
        if (TrackList.Status[i] != TRACK_EMPTY)
        {
            TrackList.CountNum[i] = 0;

            if (TrackList.CountNum[i] >= TRACK_MEAS_PEAK_NUM)
            {
                TrackList.Length[i] = TrackList.Length[i] * 2;
                TrackList.Width[i] = TrackList.Width[i] * 2;
                TrackList.VelocityCorr[i] = TrackList.VelocityCorr[i] * 1.2f;
            }
            else
            {
                TrackList.Length[i] = TrackList.Length[i] * 1.2f;
                TrackList.Width[i] = TrackList.Width[i] * 1.2f;
            }

            if (TrackList.Length[i] > TRACK_LENGTH)
            {
                TrackList.Length[i] = TRACK_LENGTH;
            }
            if (TrackList.Width[i] > TRACK_WIDTH)
            {
                TrackList.Width[i] = TRACK_WIDTH;
            }
            if (TrackList.VelocityCorr[i] > TRACK_VELOCITY)
            {
                TrackList.VelocityCorr[i] = TRACK_VELOCITY;
            }
            if (TrackList.Status[i] == TRACK_DETECTED)
            {
                TrackList.Range[i] = TrackList.Range[i] + TrackList.Velocity[i] * RadarCycleTime;
                Spherical_To_Cartesian(&TrackList.DistZ[i],
                    &TrackList.DistX[i],
                    &TrackList.DistY[i],
                    TrackList.Range[i],
                    TrackList.Elev[i],
                    TrackList.Azimuth[i]);
            }
            else if (TrackList.Status[i] == TRACK_LOCKED)
            {
                TrackList.Range[i] = TrackList.Range[i] + TrackList.Velocity[i] * RadarCycleTime;

                if (TrackList.MissNum[i] == 0)
                {
                    TrackList.Velocity[i] = TrackList.Velocity[i] + TrackList.Accel[i] * RadarCycleTime;
                }

                Spherical_To_Cartesian(&TrackList.DistZ[i],
                    &TrackList.DistX[i],
                    &TrackList.DistY[i],
                    TrackList.Range[i],
                    TrackList.Elev[i],
                    TrackList.Azimuth[i]);
            }
        }
    }
}

void MatchMeas(void)
{
    uint32_t i, j;
    float    Length_high, Length_low, Width_high, Width_low;
    bool     flag;

    for (i = 0; i < TRACK_MAX_NORMAL_TRACKS; i++)
    {
        if (TrackList.Status[i] == TRACK_LOCKED)
        {
            Length_high = TrackList.DistX[i] + TrackList.Length[i];
            Length_low = TrackList.DistX[i] - TrackList.Length[i];
            Width_high = TrackList.DistY[i] + TrackList.Width[i];
            Width_low = TrackList.DistY[i] - TrackList.Width[i];

            for (j = 0; j < PeakList.Num; j++)
            {
                if (PeakList.Status[j] == PEAK_UNMATCHED)
                {
                    flag = (PeakList.DistX[j] > Length_low) && (PeakList.DistX[j] < Length_high);
                    flag = flag && (PeakList.DistY[j] > Width_low) && (PeakList.DistY[j] < Width_high);
                    flag = flag && (fabsf(PeakList.Velocity[j] - TrackList.Velocity[i]) < TrackList.VelocityCorr[i]);

                    if (flag)
                    {
                        PeakList.Status[j] = PEAK_MATCH_WELL;
                        TrackList.MeasIndex[i][TrackList.CountNum[i]] = j;

                        TrackList.CountNum[i]++;

                        if (TrackList.CountNum[i] >= TRACK_MEAS_NUM)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    for (i = 0; i < TRACK_MAX_NORMAL_TRACKS; i++)
    {
        if (TrackList.Status[i] == TRACK_DETECTED)
        {
            Length_high = TrackList.DistX[i] + TrackList.Length[i];
            Length_low = TrackList.DistX[i] - TrackList.Length[i];
            Width_high = TrackList.DistY[i] + TrackList.Width[i];
            Width_low = TrackList.DistY[i] - TrackList.Width[i];

            for (j = 0; j < PeakList.Num; j++)
            {
                if (PeakList.Status[j] == PEAK_UNMATCHED)
                {
                    flag = (PeakList.DistX[j] > Length_low) && (PeakList.DistX[j] < Length_high);
                    flag = flag && (PeakList.DistY[j] > Width_low) && (PeakList.DistY[j] < Width_high);
                    flag = flag && (fabsf(PeakList.Velocity[j] - TrackList.Velocity[i]) < TrackList.VelocityCorr[i]);

                    if (flag)
                    {
                        PeakList.Status[j] = PEAK_MATCH_WELL;
                        TrackList.MeasIndex[i][TrackList.CountNum[i]] = j;

                        TrackList.CountNum[i]++;

                        if (TrackList.CountNum[i] >= TRACK_MEAS_NUM)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }
}

void UpdateTracks(void)
{
    uint32_t i;
    // uint16_t MEAS_PEAK_NUM;

    for (i = 0; i < TRACK_MAX_NORMAL_TRACKS; i++)
    {
        if (TrackList.Status[i] != TRACK_EMPTY)
        {
            if (TrackList.CountNum[i] >= TRACK_MEAS_PEAK_NUM)
            {
                Track_UpdateHitMiss(i, TRACK_HIT);
                TrackList.MissNum[i] = 0;
                FilterTrack(i);
            }
            // else if (TrackList.CountNum[i] == (TRACK_MEAS_PEAK_NUM - 1))
            // {
            //     FilterTrack(i);
            // }
            else
            {
                Track_UpdateHitMiss(i, TRACK_MISS);
                TrackList.MissNum[i] = TrackList.MissNum[i] + 1;
            }

            if ((TrackList.Range[i] < TRACK_DROP_RANGE) || (TrackList.Range[i] > 180) ||
                ((TrackList.Velocity[i] * 3.6f) < TRACK_DROP_VELOCITYE_MIN) ||
                ((TrackList.Velocity[i] * 3.6f) > TRACK_DROP_VELOCITYE_MAX) || (fabsf(TrackList.Velocity[i]) < 0.01f))
            {
                TrackList.Status[i] = TRACK_EMPTY;
            }
        }
    }

    for (i = 0; i < TRACK_MAX_NORMAL_TRACKS; i++)
    {
        if (TrackList.Status[i] == TRACK_DETECTED)
        {

            if (TrackList.HitMissLength[i] == 2)
            {
                if (Track_CountHitMiss(i, 2) < 2)
                {
                    TrackList.Status[i] = TRACK_EMPTY;
                }
            }
            else if (TrackList.HitMissLength[i] >= TRACK_LOCK_NUM)
            {

                // if (fabs(TrackList.Velocity[i]) < 17.0f)
                {
                    if (Track_CountHitMiss(i, TRACK_LOCK_NUM) < TRACK_LOCK_COUNTER)
                    {
                        TrackList.Status[i] = TRACK_EMPTY;
                    }
                    else
                    {
                        TrackList.MissNum[i] = 0;
                        TrackList.Status[i] = TRACK_LOCKED;
                        TrackList.LifeTime[i] = 0;
                    }
                }

                // else if (fabs(TrackList.Velocity[i]) < 21.6f)
                // {
                //     if (Track_CountHitMiss(i, 10) < 9)
                //     {
                //         TrackList.Status[i] = TRACK_EMPTY;
                //     }
                //     else
                //     {
                //         TrackList.MissNum[i] = 0;
                //         TrackList.Status[i] = TRACK_LOCKED;
                //         TrackList.LifeTime[i] = 0;
                //     }
                // }
                // else
                // {
                // if (TrackList.HitMissLength[i] >= 15)
                // {
                //     if (Track_CountHitMiss(i, 15) < 14)
                //     {
                //         TrackList.Status[i] = TRACK_EMPTY;
                //     }
                //     else
                //     {
                //         TrackList.MissNum[i] = 0;
                //         TrackList.Status[i] = TRACK_LOCKED;
                //         TrackList.LifeTime[i] = 0;
                //     }
                // }
                // }
            }
        }
        else if (TrackList.Status[i] == TRACK_LOCKED)
        {
            TrackList.LifeTime[i]++;
            Track_Gettrackqualitylife(i);
            if (TrackList.MissNum[i] > TRACK_DROP_NUM)
            {
                TrackList.Status[i] = TRACK_EMPTY;
            }
        }
    }
}

void Track_UpdateHitMiss(uint32_t TrackIndex, uint8_t Flag)
{
    uint16_t i;
    for (i = (TRACK_HIT_MISS_ARRAY_LENGTH - 1); i > 0; i--)
    {
        TrackList.HitMiss[TrackIndex][i] = TrackList.HitMiss[TrackIndex][i - 1];
    }

    TrackList.HitMissLength[TrackIndex]++;
    if (TrackList.HitMissLength[TrackIndex] > TRACK_HIT_MISS_ARRAY_LENGTH)
    {
        TrackList.HitMissLength[TrackIndex] = TRACK_HIT_MISS_ARRAY_LENGTH;
    }

    TrackList.HitMiss[TrackIndex][0] = Flag;
}

uint32_t Track_CountHitMiss(uint32_t TrackIndex, uint16_t N)
{
    uint16_t i;
    uint16_t Count;

    Count = 0;
    for (i = 0; i < N; i++)
    {
        if (TrackList.HitMiss[TrackIndex][i] == 1)
        {
            Count = Count + 1;
        }
    }
    return Count;
}

void Track_Gettrackqualitylife(uint32_t TrackIndex)
{
    uint8_t HitNum;

    if (TrackList.HitMissLength[TrackIndex] > (TRACK_HIT_MISS_ARRAY_LENGTH / 2))
    {
        HitNum = Track_CountHitMiss(TrackIndex, TrackList.HitMissLength[TrackIndex]);
        TrackList.trackqualitylife[TrackIndex] = HitNum / TrackList.HitMissLength[TrackIndex] * 100;
    }
    else
    {
        TrackList.trackqualitylife[TrackIndex] = LIFE_TIME_MIN;
    }
}

void FilterTrack(uint32_t TrackIndex)
{
    uint32_t i;
    float    Sum_Range, Sum_Velocity, Sum_Azimuth, Sum_Elevation, Sum_RCStotal;
    float Current_Range, Current_Zpos, Current_Xpos, Current_Ypos, Current_Velocity, Current_Azimuth, Current_Elevation,
        Current_RCS;
    float Length_Max, Length_Min, Width_Max, Width_Min, Velocity_Max, Velocity_Min;

    Sum_Range = 0;
    Sum_Velocity = 0;
    Sum_Azimuth = 0;

    Sum_Elevation = 0;
    Sum_RCStotal = 0;

    Length_Max = 0;
    Length_Min = 1000;
    Width_Max = 0;
    Width_Min = 1000;
    Velocity_Max = -1000;
    Velocity_Min = 1000;

    for (i = 0; i < TrackList.CountNum[TrackIndex]; i++)
    {
        Sum_Range = Sum_Range + PeakList.Range[TrackList.MeasIndex[TrackIndex][i]];
        Sum_Velocity = Sum_Velocity + PeakList.Velocity[TrackList.MeasIndex[TrackIndex][i]];
        Sum_Azimuth = Sum_Azimuth + PeakList.Azimuth[TrackList.MeasIndex[TrackIndex][i]];
        Sum_Elevation = Sum_Elevation + PeakList.Elev[TrackList.MeasIndex[TrackIndex][i]];
        Sum_RCStotal = Sum_RCStotal + PeakList.Snr[TrackList.MeasIndex[TrackIndex][i]];
    }

    Current_Range = Sum_Range / TrackList.CountNum[TrackIndex];
    Current_Velocity = Sum_Velocity / TrackList.CountNum[TrackIndex];
    Current_Azimuth = Sum_Azimuth / TrackList.CountNum[TrackIndex];
    Current_Elevation = Sum_Elevation / TrackList.CountNum[TrackIndex];
    Current_RCS = Sum_RCStotal;
    Spherical_To_Cartesian(
        &Current_Zpos, &Current_Xpos, &Current_Ypos, Current_Range, Current_Elevation, Current_Azimuth);

    if (TrackList.CountNum[TrackIndex] >= TRACK_COVMAT_NUM)
    {
        for (i = 0; i < TrackList.CountNum[TrackIndex]; i++)
        {
            if ((PeakList.DistX[TrackList.MeasIndex[TrackIndex][i]] - Current_Xpos) > Length_Max)
            {
                Length_Max = PeakList.DistX[TrackList.MeasIndex[TrackIndex][i]] - Current_Xpos;
            }
            if ((PeakList.DistX[TrackList.MeasIndex[TrackIndex][i]] - Current_Xpos) < Length_Min)
            {
                Length_Min = PeakList.DistX[TrackList.MeasIndex[TrackIndex][i]] - Current_Xpos;
            }
            if ((PeakList.DistY[TrackList.MeasIndex[TrackIndex][i]] - Current_Ypos) > Width_Max)
            {
                Width_Max = PeakList.DistY[TrackList.MeasIndex[TrackIndex][i]] - Current_Ypos;
            }
            if ((PeakList.DistY[TrackList.MeasIndex[TrackIndex][i]] - Current_Ypos) < Width_Min)
            {
                Width_Min = PeakList.DistY[TrackList.MeasIndex[TrackIndex][i]] - Current_Ypos;
            }
            if ((PeakList.Velocity[TrackList.MeasIndex[TrackIndex][i]] - Current_Velocity) > Velocity_Max)
            {
                Velocity_Max = PeakList.Velocity[TrackList.MeasIndex[TrackIndex][i]] - Current_Velocity;
            }
            if ((PeakList.Velocity[TrackList.MeasIndex[TrackIndex][i]] - Current_Velocity) < Velocity_Min)
            {
                Velocity_Min = PeakList.Velocity[TrackList.MeasIndex[TrackIndex][i]] - Current_Velocity;
            }
        }
        if (TrackList.Est_Spread[TrackIndex][0] < (Length_Max - Length_Min))
        {
            TrackList.Est_Spread[TrackIndex][0] = Length_Max - Length_Min;
            if (TrackList.Est_Spread[TrackIndex][0] > TRACK_LENGTH)
            {
                TrackList.Est_Spread[TrackIndex][0] = TRACK_LENGTH;
            }
        }
        if (TrackList.Est_Spread[TrackIndex][1] < (Width_Max - Width_Min))
        {
            TrackList.Est_Spread[TrackIndex][1] = Width_Max - Width_Min;
            if (TrackList.Est_Spread[TrackIndex][1] > TRACK_WIDTH)
                TrackList.Est_Spread[TrackIndex][1] = TRACK_WIDTH;
        }
        if (TrackList.Est_Spread[TrackIndex][2] < (Velocity_Max - Velocity_Min))
        {
            TrackList.Est_Spread[TrackIndex][2] = Velocity_Max - Velocity_Min;
            if (TrackList.Est_Spread[TrackIndex][2] > TRACK_VELOCITY)
                TrackList.Est_Spread[TrackIndex][2] = TRACK_VELOCITY;
        }
    }

    TrackList.Range[TrackIndex] =
        TrackList.Range[TrackIndex] + TRACK_RANGE_GAIN * (Current_Range - TrackList.Range[TrackIndex]);
    TrackList.Velocity[TrackIndex] =
        TrackList.Velocity[TrackIndex] + TRACK_RATE_GAIN * (Current_Velocity - TrackList.Velocity[TrackIndex]);
    TrackList.Accel[TrackIndex] =
        TrackList.Accel[TrackIndex] +
        TRACK_ACCEL_GAIN * (Current_Velocity - TrackList.Velocity[TrackIndex]) / RadarCycleTime;
    TrackList.RCS[TrackIndex] = Current_RCS;

    // if (TrackList.Range[TrackIndex]>30)
    // {
    // TrackList.Azimuth[TrackIndex] = Track_AngleFilter(TrackIndex, Current_Azimuth);
    // TrackList.Elev[TrackIndex] = Track_ElevAngleFilter(TrackIndex, Current_Elevation);
    // Spherical_To_Cartesian(&TrackList.DistZ[TrackIndex], &TrackList.DistX[TrackIndex], &TrackList.DistY[TrackIndex],
    // TrackList.Range[TrackIndex], TrackList.Elev[TrackIndex], TrackList.Azimuth[TrackIndex]);
    // }
    // else
    {
        TrackList.Azimuth[TrackIndex] = Current_Azimuth;
        TrackList.Elev[TrackIndex] = Current_Elevation;
        Spherical_To_Cartesian(&TrackList.DistZ[TrackIndex],
            &TrackList.DistX[TrackIndex],
            &TrackList.DistY[TrackIndex],
            TrackList.Range[TrackIndex],
            TrackList.Elev[TrackIndex],
            TrackList.Azimuth[TrackIndex]);
        //  TrackList.DistY[TrackIndex] = Track_DistYFilter(TrackIndex, TrackList.DistY[TrackIndex]);
    }
    // TrackList.VeloX[TrackIndex] = TrackList.Velocity[TrackIndex] * cos((RadarPara.InstallAngle +
    // TrackList.Azimuth[TrackIndex]) * pi / 180); TrackList.VeloY[TrackIndex] = TrackList.Velocity[TrackIndex] *
    // sin((RadarPara.InstallAngle + TrackList.Azimuth[TrackIndex]) * pi / 180);

    // if (TrackList.Azimuth[TrackIndex] <= 10)
    // {
    //     TrackList.VeloY[TrackIndex] = TrackList.Velocity[TrackIndex] * sin((RadarPara.InstallAngle +
    //     TrackList.Azimuth[TrackIndex]) * pi / 180); TrackList.VeloX[TrackIndex] = TrackList.Velocity[TrackIndex] /
    //     cos((RadarPara.InstallAngle + TrackList.Azimuth[TrackIndex]) * pi / 180);
    // }
    // else
    {
        TrackList.VeloY[TrackIndex] =
            TrackList.Velocity[TrackIndex] / cosf((90 - RadarInstallAngle - TrackList.Azimuth[TrackIndex]) * PI / 180);
        TrackList.VeloX[TrackIndex] =
            TrackList.Velocity[TrackIndex] * sinf((90 - RadarInstallAngle - TrackList.Azimuth[TrackIndex]) * PI / 180);
    }
}

void Track_AddNew(void)
{
    uint32_t i, j, m, n;
    uint32_t MeasIndex[TRACK_MEAS_NUM];
    uint8_t  CountNum;
    uint8_t  CountNum_Peak;
    float    Sum_Range, Sum_Velocity, Sum_Azimuth, Sum_Elevation, Sum_RCStotal;
    float Current_Range, Current_Zpos, Current_Xpos, Current_Ypos, Current_Velocity, Current_Azimuth, Current_Elevation,
        Current_RCS;
    bool flag;

    if (PeakList.Num == 0)
    {
        return;
    }

    for (i = 0; i < (PeakList.Num); i++)
    {
        if (PeakList.Status[i] == PEAK_UNMATCHED)
        {
            if (fabsf(PeakList.Velocity[i]) < 0.35f)
            {
                PeakList.Status[i] = PEAK_MATCH_WELL;
                continue;
            }

            CountNum = 0;
            MeasIndex[CountNum] = i;
            Sum_Range = PeakList.Range[i];
            Sum_Velocity = PeakList.Velocity[i];
            Sum_Azimuth = PeakList.Azimuth[i];
            Sum_Elevation = PeakList.Elev[i];
            Sum_RCStotal = PeakList.Snr[i];
            CountNum++;

            Current_Range = PeakList.Range[i];
            Current_Velocity = PeakList.Velocity[i];
            Current_Azimuth = PeakList.Azimuth[i];
            Current_Elevation = PeakList.Elev[i];
            Current_Xpos = PeakList.DistX[i];
            Current_Ypos = PeakList.DistY[i];
            Current_RCS = PeakList.Snr[i];

            for (j = i + 1; j <= PeakList.Num; j++)

            {

                if (j == PeakList.Num)
                {
                    break;
                }
                if (PeakList.Status[j] == PEAK_UNMATCHED)
                {
                    // if (fabs(PeakList.Velocity[j]) < 0.35)
                    // {
                    //     PeakList.Status[j] = PEAK_MATCH_WELL;
                    //     continue;
                    // }
                    flag = (fabsf(Current_Xpos - PeakList.DistX[j]) < TRACK_LENGTH);
                    flag = flag && (fabsf(Current_Ypos - PeakList.DistY[j]) < TRACK_WIDTH);
                    flag = flag && (fabsf(Current_Velocity - PeakList.Velocity[j]) < TRACK_VELOCITY);

                    if (flag)
                    {
                        PeakList.Status[j] = PEAK_MATCH_WELL;
                        Sum_Range = Sum_Range + PeakList.Range[j];
                        Sum_Velocity = Sum_Velocity + PeakList.Velocity[j];
                        Sum_Azimuth = Sum_Azimuth + PeakList.Azimuth[j];
                        Sum_Elevation = Sum_Elevation + PeakList.Elev[j];
                        Sum_RCStotal = Sum_RCStotal + PeakList.Snr[j];
                        MeasIndex[CountNum] = j;

                        CountNum = CountNum + 1;
                        Current_Range = Sum_Range / CountNum;
                        Current_Velocity = Sum_Velocity / CountNum;
                        Current_Azimuth = Sum_Azimuth / CountNum;
                        Current_Elevation = Sum_Elevation / CountNum;
                        Current_RCS = Sum_RCStotal;
                        Spherical_To_Cartesian(&Current_Zpos,
                            &Current_Xpos,
                            &Current_Ypos,
                            Current_Range,
                            Current_Elevation,
                            Current_Azimuth);

                        if (CountNum >= TRACK_MEAS_NUM)
                        {
                            break;
                        }
                    }
                }
            }

            Spherical_To_Cartesian(
                &Current_Zpos, &Current_Xpos, &Current_Ypos, Current_Range, Current_Elevation, Current_Azimuth);

            if (Current_Range < 30)
            {
                CountNum_Peak = TRACK_NEW_PEAK_NUM + 3;
            }
            else if ((Current_Range >= 30) && (Current_Range < 60))
            {
                CountNum_Peak = TRACK_NEW_PEAK_NUM + 1;
            }
            else if ((Current_Range >= 60) && (Current_Range < 100))
            {
                CountNum_Peak = TRACK_NEW_PEAK_NUM;
            }
            else
            {
                CountNum_Peak = TRACK_NEW_PEAK_NUM - 1;
            }
            // if (CountNum >= CountNum_Peak)

            //  if (CountNum >= TRACK_NEW_PEAK_NUM)
            {
                for (m = 0; m < TRACK_MAX_NORMAL_TRACKS; m++)
                {
                    if (TrackList.Status[m] == TRACK_EMPTY)
                    {
                        TrackList.Status[m] = TRACK_DETECTED;
                        TrackList.Range[m] = Current_Range;
                        TrackList.Velocity[m] = Current_Velocity;
                        TrackList.Azimuth[m] = Current_Azimuth;
                        TrackList.Elev[m] = Current_Elevation;
                        TrackList.RCS[m] = Current_RCS;

                        TrackList.DistX[m] = Current_Xpos;
                        TrackList.DistY[m] = Current_Ypos;

                        TrackList.Length[m] = TRACK_LENGTH;
                        TrackList.Width[m] = TRACK_WIDTH;
                        TrackList.VelocityCorr[m] = TRACK_VELOCITY;

                        TrackList.CountNum[m] = CountNum;
                        for (n = 0; n < TrackList.CountNum[m]; n++)
                        {
                            TrackList.MeasIndex[m][n] = MeasIndex[n];
                        }

                        TrackList.HitMissLength[m] = 1;
                        TrackList.HitMiss[m][0] = TRACK_HIT;
                        TrackList.AngleBufLength[m] = 1;
                        TrackList.AngleBuf[m][0] = Current_Azimuth;
                        TrackList.ElevAngleBufLength[m] = 1;
                        TrackList.EleAngleBuf[m][0] = Current_Elevation;

                        // TrackList.DistYFilterLength[m] = 1;
                        // TrackList.DistYFilterBuf[m][0]  = Current_Ypos;

                        break;
                    }
                }
            }
        }
    }
}

void Spherical_To_Cartesian(float* z, float* x, float* y, float r, float elve, float azimuth)
{
    // RadarPara.InstallAngle = 0;

    // *z = r * sin(elve * PI / 180);
    // *x = r * cos(elve * PI / 180) * cos((azimuth + RadarPara.InstallAngle) * PI / 180);
    // *y = r * cos(elve * PI / 180) * sin((azimuth + RadarPara.InstallAngle) * PI / 180);

    // *z = r * sinf(elve * PI / 180);
    // *x = r * cosf(elve * PI / 180) * cosf((azimuth)*PI / 180);
    // *y = r * cosf(elve * PI / 180) * sinf((azimuth)*PI / 180);

    *x = r * cosf(elve * PI / 180) * sinf((azimuth)*PI / 180);
    *y = r * cosf(elve * PI / 180) * cosf((azimuth)*PI / 180);
}