#ifndef XTLOG_H
#define XTLOG_H
#include<stdint.h>
#include<stdio.h>
#include <string.h>


errno_t err;

//rngfftdatafile
FILE* file_Rngfft = NULL;
const char* filename_Rngfft = "D:\\outputdata\\output.log";

#ifdef __cplusplus
extern "C" {
#endif

	void OutputDatainitial();

#ifdef __cplusplus
}
#endif

#endif // XTLOG_H
