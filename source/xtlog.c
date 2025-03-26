#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include "../Header/xtlog.h"

void OutputDatainitial()
{
    err = fopen_s(&file_Rngfft, filename_Rngfft, "a"); // 以追加模式打开文件

    if (err != 0) {
        // 处理错误
        printf("无法打开文件\n");
        return;
    }
}



