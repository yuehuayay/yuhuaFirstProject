#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include "../ProjectFY001/Header/senserconfig.h"
#include "../ProjectFY001/source/RangePro.c"
#include "../ProjectFY001/Header/AssisedDriing.h"
#include"../ProjectFY001/Header/ABMatch.h"

#define datalengthA  (RANGE_FFT_SIZE * NumchirpA * NumchannelMimo*2)
#define datalengthB  (RANGE_FFT_SIZE * NumchirpB * NumchannelMimo*2)
target_infov2_t info[MAXNUM_OF_DETECTION];
float AdcDataA[datalengthA];
float AdcDataB[datalengthB];
AppHandle_t  app_Handle;



int main()
{
	uint16_t num_chirps;
	//read rawdata
	errno_t err;
	FILE* file = NULL;
	extern void RangeFFT(uint8_t channleine,uint8_t dopplerLine, uint16_t num_chirps,float* adcbuffer);  //源代码里是uint16
	const char* filename = "C:\\Users\\zhangpeng\\Desktop\\March\\ProjectFY001\\data\\destataA.bin"; // bin文件名
	memset(AdcDataA, 0, sizeof(AdcDataA));
	err = fopen_s(&file, filename, "rb");
	if (err != 0) {
		fprintf(stderr, "Error opening file: %d\n", err);
		return -1; 
	}
	fread(AdcDataA, sizeof(float), datalengthA, file);
	if (frametype)
	{
		num_chirps = NumchirpB;

	}
	else
	{
		num_chirps = NumchirpA;
	}
	
	uint16_t RDatalengthAB = RANGE_FFT_SIZE * num_chirps;
	//OutputDatainitial();

	for (uint8_t channelidx = 0; channelidx < NumchannelMimo;channelidx++)
	{
		for (uint8_t chirpidx = 0; chirpidx < num_chirps; chirpidx++)
		{
			RangeFFT(channelidx, chirpidx, num_chirps,&AdcDataA[channelidx * RDatalengthAB *2 + chirpidx * RANGE_FFT_SIZE * 2]);
		}
	}

    DopplerProcess (num_chirps);
	CFARprocess();
	//CM_phase_Angle();
	Complexfloat spdcom_complex = Velocity_Compensation();
	Estimate_Azim(spdcom_complex);



	for (int i = 0; i < gnumObjs; i++)
	{
		info[i].state = target[i].state;

		info[i].id = i;
		info[i].x = target[i].range * sinf(target[i].Angle * PI / 180); // 单位m
		info[i].y = target[i].range * cosf(target[i].Angle * PI / 180); // 单位m
		info[i].speed = target[i].velocity;  // 单位m/s
		info[i].mag = target[i].strength - target[i].noise1;
		info[i].noise = target[i].strength - target[i].noise2;
	}
	 //printf("numobj = %d\n", gnumObjs);

	 Functional_alarm(&app_Handle,info, gnumObjs);
	return 0;


}

