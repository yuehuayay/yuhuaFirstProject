#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include "../ProjectFY001/Header/senserconfig.h"
#include "../ProjectFY001/source/RangePro.c"
#include "../ProjectFY001/Header/AssisedDriing.h"

#define datalength  (RANGE_FFT_SIZE * DOPPLER_FFT_SIZE * NumchannelMimo*2)
target_infov2_t info[MAXNUM_OF_DETECTION];
float AdcData[datalength];
AppHandle_t  app_Handle;



int main()
{
	uint16_t num_chirps;
	//read rawdata
	errno_t err;
	FILE* file = NULL;
	extern void RangeFFT(uint8_t channleine,uint8_t dopplerLine, float* adcbuffer);  //源代码里是uint16
	const char* filename = "C:\\Users\\zhangpeng\\Desktop\\March\\ProjectFY001\\data\\destata.bin"; // bin文件名
	memset(AdcData, 0, sizeof(AdcData));
	err = fopen_s(&file, filename, "rb");
	if (err != 0) {
		fprintf(stderr, "Error opening file: %d\n", err);
		return -1; 
	}
	fread(AdcData, sizeof(float), datalength, file);
	num_chirps = DOPPLER_FFT_SIZE;

	//OutputDatainitial();

	for (uint8_t channelidx = 0; channelidx < NumchannelMimo;channelidx++)
	{
		for (uint8_t chirpidx = 0; chirpidx < num_chirps; chirpidx++)
		{
			RangeFFT(channelidx, chirpidx,&AdcData[channelidx * RDatalength*2 + chirpidx * RANGE_FFT_SIZE * 2]);
		}
	}

  DopplerProcess ();
	CFARprocess();
	CM_phase_Angle();

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

