#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include "../Project1215YD/Header/senserconfig.h"
#include "../Project1215YD/source/RangePro.c"
#include "../Project1215YD/Header/AssisedDriing.h"

#define datalength  (128 * 32 * 2 * 2)
target_infov2_t info[MAXNUM_OF_DETECTION];
float radarcubeodd[datalength];
AppHandle_t  app_Handle;



int main()
{
	uint16_t num_chirps;
	//read rawdata
	errno_t err;
	FILE* file = NULL;
	extern void RangeFFT(uint8_t dopplerLine, float* adcbuffer);  //源代码里是uint16
	const char* filename = "C:\\Users\\zhangpeng\\Desktop\\January\\Project1215YD\\SIL1215YD\\destodddata.bin"; // bin文件名
	memset(radarcubeodd, 0, sizeof(radarcubeodd));
	err = fopen_s(&file, filename, "rb");
	if (err != 0) {
		fprintf(stderr, "Error opening file: %d\n", err);
		return -1; 
	}
	fread(radarcubeodd, sizeof(float), datalength, file);
	num_chirps = 32;


	for (uint8_t i = 0; i < num_chirps; i++)
	{
		RangeFFT(i,&radarcubeodd[i*128*2]);
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
	 printf("numobj = %d\n", gnumObjs);

	 Functional_alarm(&app_Handle,info, gnumObjs);
	return 0;


}

