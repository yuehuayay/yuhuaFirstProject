#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include "../Project1215YD/Header/senserconfig.h"
#include "../Project1215YD/source/RangePro.c"

#define datalength  (128 * 32 * 2 * 2)
float radarcubeodd[datalength];





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
	return 0;


}

