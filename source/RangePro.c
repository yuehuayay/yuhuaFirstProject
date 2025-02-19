#include<stdint.h>
#include<stdio.h>
#include <string.h>
#include <stdbool.h>
#include<math.h>
#include "../Header/senserconfig.h"
#include "../Header/FFTData.h"
#include "../Header/win_api.h"

arm_cfft_instance_f32 czt_fft_256;
typedef float float32_t;

//  
DPIF_PointCloud target[MAXNUM_OF_DETECTION];
uint32_t gcfarDopplerDetOutBitMask[RANGE_FFT_SIZE / 2 * DOPPLER_FFT_SIZE / 32];
uint16_t gnumObjs;

void CM_phase_Angle()
{
    for (uint16_t objidx = 0; objidx< gnumObjs;objidx++)
    {
        float channe0_real = target[objidx].channel_data[0];
        float channe0_imag = target[objidx].channel_data[1];
        float channe1_real = target[objidx].channel_data[2];
        float channe1_imag = target[objidx].channel_data[3];

        float deltaPhase_real = (channe0_real * channe1_real + channe0_imag * channe1_imag) / (channe0_real* channe0_real+ channe0_imag* channe0_imag);//((c+di)/(a+bi))
        float deltaPhase_imag = (channe0_real * channe1_imag - channe0_imag * channe1_real) / (channe0_real * channe0_real + channe0_imag * channe0_imag);//((c+di)/(a+bi))
        float deltphase = atan2f(deltaPhase_imag, deltaPhase_real);
        float Angle_Degree = asinf(deltphase / PI)*(180/PI);
        printf("angle = %f", Angle_Degree);
    }



}

void Dsp_Quadratic_Filter(float* x, float* y, float* xv, float* yv)
{
    float a, b, c, denom;
    float x0 = x[0];
    float x1 = x[1];
    float x2 = x[2];
    float y0 = y[0];
    float y1 = y[1];
    float y2 = y[2];

    denom = (x0 - x1) * (x0 - x2) * (x1 - x2);                      //<0
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom; // <0
    b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / denom;
    c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / denom;

    *xv = -b / (2 * a);
    *yv = c - b * b / (4 * a);
}

uint16_t cfarPeakPruning(uint16_t* grpPeakIdx,
    DPIF_PointCloud* detObj,
    uint16_t         numDetectedObjects,
    float* detMatrix,
    uint16_t         numDopplerBins,
    uint16_t         maxRangeIdx,
    uint16_t         minRangeIdx,
    uint16_t         groupInDopplerDirection,
    uint16_t         groupInRangeDirection,
    uint16_t         maxNumDetObjs)
{
    uint16_t numObjOut = 0;          // 检测后检测到的目标数
    uint16_t rangeIdx, dopplerIdx;   // 存储目标的距离和多普勒索引
    float    peakVal;                // 当前检测的目标的峰值
    uint16_t rowIdx, i, numRows;     // 行索引、循环变量和要处理的行数
    int16_t  columnIdx, columnCount; // 列索引和列计数
    uint16_t detectedObjFlag;        // 标识是否检测到目标
    uint16_t tempPtr;                // 临时指针用于遍历检测矩阵

    // 同时在距离和多普勒方向进行检测
    if ((groupInDopplerDirection == 1) && (groupInRangeDirection == 1))
    {
        // 遍历所有已检测到的目标，进行峰值检测
        for (i = 0; i < numDetectedObjects; i++)
        {
            detectedObjFlag = 0;                       // 初始化标识
            rangeIdx = detObj[i].rangeIdx;             // 获取目标的距离索引
            dopplerIdx = detObj[i].dopplerIdx;         // 获取目标的多普勒索引
            tempPtr = rangeIdx * numDopplerBins;       // 获取检测矩阵中的索引位置
            peakVal = detMatrix[tempPtr + dopplerIdx]; // 获取该位置的峰值


            // 判断目标在检测矩阵中的相邻行数
            numRows = 0;
            if (rangeIdx == minRangeIdx)
            {
                detectedObjFlag = 1;
                numRows = 2; // 只有两行（当前和下一行）
                tempPtr = rangeIdx * numDopplerBins;
            }
            else if (rangeIdx == maxRangeIdx)
            {
                detectedObjFlag = 1;
                numRows = 2; // 只有两行（当前和上一行）
                tempPtr = (rangeIdx - 1) * numDopplerBins;
            }
            else if ((rangeIdx < maxRangeIdx) && (rangeIdx > minRangeIdx))
            {
                detectedObjFlag = 1;
                numRows = 3; // 三行：上一行、当前行、下一行
                tempPtr = (rangeIdx - 1) * numDopplerBins;
            }

            // 在各行中检测目标峰值
            for (rowIdx = 0; rowIdx < numRows; rowIdx++)
            {
                for (columnCount = 0; columnCount < 3; columnCount++)
                {
                    // 计算列索引，确保多普勒方向上进行循环
                    columnIdx = dopplerIdx + columnCount - 1;
                    if (columnIdx < 0)
                    {
                        columnIdx += numDopplerBins;
                    }
                    else if (columnIdx >= numDopplerBins)
                    {
                        columnIdx -= numDopplerBins;
                    }
                    // 如果当前峰值小于邻近的峰值，认为不是检测到的目标
                    if (peakVal < detMatrix[tempPtr + columnIdx])
                    {
                        detectedObjFlag = 0;
                    }
                }
                tempPtr += numDopplerBins; // 移动到下一行
            }
            // 如果目标在所有行中都保持最大峰值，将其添加到检测目标列表
            if (detectedObjFlag == 1)
            {
                grpPeakIdx[numObjOut] = i;

                detObj[i].channel_data[0] = RD_Map_Dopplerffttemp[2 * (rangeIdx* numDopplerBins + dopplerIdx)];
                detObj[i].channel_data[1] = RD_Map_Dopplerffttemp[2 * (rangeIdx * numDopplerBins + dopplerIdx) + 1];
                detObj[i].channel_data[2] = RD_Map_Dopplerffttemp1[2 * (rangeIdx * numDopplerBins + dopplerIdx)];
                detObj[i].channel_data[3] = RD_Map_Dopplerffttemp1[2 * (rangeIdx * numDopplerBins + dopplerIdx) + 1];   //保留峰值点通道复数

                numObjOut++;
            }
            // 若检测到的目标数已达到最大数量，则停止检测
            if (numObjOut >= maxNumDetObjs)
            {
                break;
            }
        }
    }
    // 仅在多普勒方向上进行检测
    else if ((groupInDopplerDirection == 1) && (groupInRangeDirection == 0))
    {
        // 遍历所有检测到的目标，进行多普勒方向上的检测
        for (i = 0; i < numDetectedObjects; i++)
        {
            detectedObjFlag = 0;
            rangeIdx = detObj[i].rangeIdx;
            dopplerIdx = detObj[i].dopplerIdx;
            tempPtr = rangeIdx * numDopplerBins;
            peakVal = detMatrix[tempPtr + dopplerIdx];

            // 检查目标峰值在多普勒方向上是否保持最高
            if ((rangeIdx <= maxRangeIdx) && (rangeIdx >= minRangeIdx))
            {
                detectedObjFlag = 1;
                for (columnCount = 0; columnCount < 2; columnCount++)
                {
                    columnIdx = dopplerIdx + 2 * columnCount - 1;   //计算相邻目标的idx-1和idx+1
                    if (columnIdx < 0)
                    {
                        columnIdx += numDopplerBins;
                    }
                    else if (columnIdx >= numDopplerBins)
                    {
                        columnIdx -= numDopplerBins;
                    }
                    if (peakVal < detMatrix[tempPtr + columnIdx])
                    {
                        detectedObjFlag = 0;
                    }
                }
                // 如果目标在多普勒方向上保持最大峰值，将其添加到检测目标列表
                if (detectedObjFlag == 1)
                {
                    grpPeakIdx[numObjOut] = i;
                    numObjOut++;
                }
                // 若检测到的目标数已达到最大数量，则停止检测
                if (numObjOut >= maxNumDetObjs)
                {
                    break;
                }
            }
        }
    }
    // 仅在距离方向上进行检测
    else if ((groupInDopplerDirection == 0) && (groupInRangeDirection == 1))
    {
        // 遍历所有检测到的目标，进行距离方向上的检测
        for (i = 0; i < numDetectedObjects; i++)
        {
            rangeIdx = detObj[i].rangeIdx;
            dopplerIdx = detObj[i].dopplerIdx;
            tempPtr = rangeIdx * numDopplerBins;
            peakVal = detMatrix[tempPtr + dopplerIdx];

            numRows = 1;
            detectedObjFlag = 1;
            if (rangeIdx == minRangeIdx)
            {
                tempPtr += numDopplerBins; // 只向上检查一行
            }
            else if (rangeIdx == maxRangeIdx)
            {
                tempPtr -= numDopplerBins; // 只向下检查一行
            }
            else if ((rangeIdx < maxRangeIdx) && (rangeIdx > minRangeIdx))
            {
                numRows = 2; // 检查上下两行
                tempPtr -= numDopplerBins;
            }
            else
            {
                numRows = 0;
                detectedObjFlag = 0;
            }

            for (rowIdx = 0; rowIdx < numRows; rowIdx++)
            {
                if (peakVal < detMatrix[tempPtr + dopplerIdx])
                {
                    detectedObjFlag = 0;
                }
                tempPtr += (numDopplerBins * 2); // 下一行
            }
            // 如果目标在距离方向上保持最大峰值，将其添加到检测目标列表
            if (detectedObjFlag == 1)
            {
                grpPeakIdx[numObjOut] = i;
                numObjOut++;
            }
            // 若检测到的目标数已达到最大数量，则停止检测
            if (numObjOut >= maxNumDetObjs)
            {
                break;
            }
        }
    }

    return numObjOut;
}

uint16_t CFARCA_peakGrouping(uint16_t numDetectedObjects)
{

    uint16_t grpPeakIdx[MAXNUM_OF_DETECTION]; // 检测到的目标索引
    uint16_t numPeakGrp = 0;                  // 检测后分组的峰值数量
    uint16_t index;                           // 遍历的索引变量

    // 参数声明
    uint16_t numDopplerBins = DOPPLER_FFT_SIZE;    // 多普勒维度的总数
    uint16_t maxRangeIdx = RANGE_FFT_SIZE / 2 - 1; // 最大距离索引
    uint16_t minRangeIdx = 0;                      // 最小距离索引
    uint16_t groupInDopplerDirection = 1;          // 是否在多普勒方向进行检测
    uint16_t groupInRangeDirection = 1;            // 是否在距离方向进行检测
    uint16_t maxNumDetObjs = MAXNUM_OF_DETECTION;  // 检测后保留的最大目标数量

    // 调用峰值检测函数以获取检测后的峰值目标索引
    numPeakGrp = cfarPeakPruning(grpPeakIdx,
        target,
        numDetectedObjects,
        RD_Map,
        numDopplerBins,
        maxRangeIdx,
        minRangeIdx,
        groupInDopplerDirection,
        groupInRangeDirection,
        maxNumDetObjs);

    // 遍历检测后的目标，将目标的属性复制到 `target` 数组的前 `numPeakGrp` 个位置
    for (index = 0; index < numPeakGrp; index++)
    {
        target[index].rangeIdx = target[grpPeakIdx[index]].rangeIdx;
        target[index].dopplerIdx = target[grpPeakIdx[index]].dopplerIdx;
        target[index].noise1 = 20 * log10f(target[grpPeakIdx[index]].noise1);
        target[index].noise2 = 20 * log10f(target[grpPeakIdx[index]].noise2);

    }

    // 返回检测后的目标数量
    return numPeakGrp;
}

bool isObjectDetected(uint16_t rangeIdx, uint16_t dopplerIdx, uint16_t numDopplerBins, uint32_t* cfarDetOutBitMask)
{
    uint32_t bit, word;
    uint32_t bitIndex;
    bool     detected = false;

    // 计算一维比特索引：rangeIdx 和 dopplerIdx 的组合
    bitIndex = rangeIdx * numDopplerBins + dopplerIdx;

    // 计算该比特索引在掩码数组中的元素索引和比特位置
    word = bitIndex >> 5; // 相当于除以 32，计算数组的元素索引
    bit = bitIndex & 31;  // 计算在数组元素内的比特位置 (0-31)

    // 检查比特位置是否被置位，如果是，则检测到对象
    if (cfarDetOutBitMask[word] & (1 << bit))
    {
        detected = true;
    }

    return detected;
}


uint16_t cfarCadB_SOGO(float* inp,
    uint16_t* out,
    uint16_t  cfartype,
    uint16_t  len,
    uint16_t  guardLen,
    uint16_t  noiseLen,
    float     Pfa,
    float* outnoise)
{
    // 定义和初始化索引和变量
    uint16_t idx, idxLeftNext, idxLeftPrev, idxRightNext, idxRightPrev, outIdx, idxCUT;
    float    sum, sumLeft, sumRight;
    float    Nnum = 2.0f * noiseLen; // 噪声窗的总长度
    // float    alpha = Nnum * (pow(Pfa, -1.0f / Nnum) - 1); // 计算门限因子alpha

// float alpha = 7; // 调整门限值

    float alpha = 10; // 调整门限值

    // float alpha = 10; // 调整门限值

    float threshold = 0.0f;

    // 初始化索引和累加变量
    outIdx = 0;
    sumLeft = 0;
    sumRight = 0;

    // 初始化右侧噪声窗的总和
    for (idx = 0; idx < noiseLen; idx++)
    {
        sumRight += inp[idx + guardLen + 1U];
    }

    /*********************************************************************************************/
    // 前段部分：针对第一个保护区和噪声窗的检测
    idxCUT = 0;

    //
    // 计算初始门限值
    threshold = sumRight / (0.5f * Nnum) * alpha;

    // 如果第一个检测单元 (CUT) 的值超过门限值，将其标记为目标
    if (inp[idxCUT] > threshold)
    {
        out[outIdx] = idxCUT;
        outnoise[outIdx] = sumRight / (0.5f * Nnum);
        outIdx++;
    }
    idxCUT++;

    // 初始化左右噪声窗的边界索引
    idxLeftNext = 0;
    idxRightPrev = idxCUT + guardLen;
    idxRightNext = idxRightPrev + noiseLen;

    // 循环处理保护区和噪声窗的前段部分
    for (idx = 0; idx < (noiseLen + guardLen - 1U); idx++)
    {
        // 更新右侧噪声窗的总和
        sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev];
        idxRightNext++;
        idxRightPrev++;

        // 更新左侧噪声窗的总和
        if (idx < noiseLen)
        {
            sumLeft += inp[idxLeftNext];
            idxLeftNext++;
        }

        // 计算门限值
        threshold = sumRight / (0.5f * Nnum) * alpha;

        // 如果检测单元 (CUT) 的值超过门限值，将其标记为目标
        if (inp[idxCUT] > threshold)
        {
            out[outIdx] = idxCUT;
            outnoise[outIdx] = sumRight / (0.5f * Nnum);
            outIdx++;
        }
        idxCUT++;
    }

    /*********************************************************************************************/
    // 中段部分：根据指定的 CFAR 类型进行双侧检测
    sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev];
    idxRightNext++;
    idxRightPrev++;

    // 判断 CFAR 类型，选择不同的检测策略
    if (cfartype == 0) // 常规 CFAR_CA 算法
    {
        // 左右两侧噪声窗的和
        sum = sumRight + sumLeft;

        // 计算门限值
        threshold = sum / Nnum * alpha;

        // 如果当前 CUT 超过门限值，将其标记为目标
        if (inp[idxCUT] > threshold)
        {
            out[outIdx] = idxCUT;
            outnoise[outIdx] = sum / Nnum;
            outIdx++;
        }
        idxCUT++;

        // 初始化左侧噪声窗的边界索引
        idxLeftPrev = 0;

        // 循环处理中段部分的所有检测单元 (CUT)
        for (idx = 0; idx < (len - 2U * (noiseLen + guardLen) - 1U); idx++)
        {
            // 更新左右噪声窗的和
            sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev];
            sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev];
            idxLeftNext++;
            idxLeftPrev++;
            idxRightNext++;
            idxRightPrev++;

            // 计算新的门限值
            sum = sumLeft + sumRight;
            threshold = sum / Nnum * alpha;

            // 如果当前 CUT 超过门限值，将其标记为目标
            if (inp[idxCUT] > threshold)
            {
                out[outIdx] = idxCUT;
                outnoise[outIdx] = sum / Nnum;
                outIdx++;
            }
            idxCUT++;
        }
    }
    else if (cfartype == 1) // CFAR_CASO 算法
    {
        // 取左侧和右侧噪声窗中较小的值
        sum = (sumLeft < sumRight) ? sumLeft : sumRight;

        // 计算门限值
        threshold = sum / (0.5f * Nnum) * alpha;

        // 如果当前 CUT 超过门限值，将其标记为目标
        if (inp[idxCUT] > threshold)
        {
            out[outIdx] = idxCUT;
            outnoise[outIdx] = sum / (0.5f * Nnum);
            outIdx++;
        }
        idxCUT++;

        // 初始化左侧噪声窗的边界索引
        idxLeftPrev = 0;

        // 循环处理中段部分的所有检测单元 (CUT)
        for (idx = 0; idx < (len - 2U * (noiseLen + guardLen) - 1U); idx++)
        {
            // 更新左右噪声窗的和
            sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev];
            sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev];
            idxLeftNext++;
            idxLeftPrev++;
            idxRightNext++;
            idxRightPrev++;

            // 取左侧和右侧噪声窗中较小的值
            sum = (sumLeft < sumRight) ? sumLeft : sumRight;

            // 计算新的门限值
            threshold = sum / (0.5f * Nnum) * alpha;

            // 如果当前 CUT 超过门限值，将其标记为目标
            if (inp[idxCUT] > threshold)
            {
                out[outIdx] = idxCUT;
                outnoise[outIdx] = sum / (0.5f * Nnum);
                outIdx++;
            }
            idxCUT++;
        }
    }
    else // CFAR_CAGO 算法
    {
        // 取左侧和右侧噪声窗中较大的值
        sum = (sumLeft > sumRight) ? sumLeft : sumRight;
        threshold = sum / (0.5f * Nnum) * alpha;

        // 如果当前 CUT 超过门限值，将其标记为目标
        if (inp[idxCUT] > threshold)
        {
            out[outIdx] = idxCUT;
            outnoise[outIdx] = sum / (0.5f * Nnum);
            outIdx++;
        }
        idxCUT++;

        // 初始化左侧噪声窗的边界索引
        idxLeftPrev = 0;
        // sum = sumLeft + sumRight;

        // 循环处理中段部分的所有检测单元 (CUT)
        for (idx = 0; idx < (len - 2U * (noiseLen + guardLen) - 1U); idx++)
        {
            // 更新左右噪声窗的和
            sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev];
            sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev];
            idxLeftNext++;
            idxLeftPrev++;
            idxRightNext++;
            idxRightPrev++;

            // 取左侧和右侧噪声窗中较大的值
            sum = (sumLeft > sumRight) ? sumLeft : sumRight;

            // 计算新的门限值
            threshold = sum / (0.5f * Nnum) * alpha;

            // 如果当前 CUT 超过门限值，将其标记为目标
            if (inp[idxCUT] > threshold)
            {
                out[outIdx] = idxCUT;
                outnoise[outIdx] = sum / (0.5f * Nnum);
                outIdx++;
            }
            idxCUT++;
        }
    }

    /*********************************************************************************************/
    // 后段部分：针对最后一个保护区和噪声窗的检测
    for (idx = 0; idx < (noiseLen + guardLen); idx++)
    {
        // 更新左侧噪声窗的和
        sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev];
        idxLeftNext++;
        idxLeftPrev++;

        // 计算门限值
        threshold = sumLeft / (0.5f * Nnum) * alpha;

        // 如果当前 CUT 超过门限值，将其标记为目标
        if (inp[idxCUT] > threshold)
        {
            out[outIdx] = idxCUT;
            outnoise[outIdx] = sumLeft / (0.5f * Nnum);
            outIdx++;
        }
        idxCUT++;
    }

    /*********************************************************************************************/

    // 返回检测到的目标数量
    return outIdx;
}

bool isObjectDetectedOnDopplerLine(uint16_t  dopplerIdx,
    uint16_t  numDopplerBins,
    uint16_t  numRangeBins,
    uint32_t* cfarDetOutBitMask)
{
    uint32_t bit, word;
    uint32_t bitIndex;
    uint32_t rangeIdx;
    bool     detected = false;

    // 遍历所有距离索引
    for (rangeIdx = 0; rangeIdx < numRangeBins; rangeIdx++)
    {
        // 计算一维比特索引：rangeIdx 和 dopplerIdx 的组合
        bitIndex = rangeIdx * numDopplerBins + dopplerIdx;

        // 计算该比特索引在掩码数组中的元素索引和比特位置
        word = bitIndex >> 5; // 相当于除以 32，计算数组的元素索引
        bit = bitIndex & 31;  // 计算在数组元素内的比特位置 (0-31)

        // 检查比特位置是否被置位，如果是，则检测到对象
        if (cfarDetOutBitMask[word] & (1 << bit))
        {
            detected = true;
            break; // 检测到目标后立即停止循环
        }
    }

    return detected;
}

void setCfarDetMaskLine(uint16_t rangeInd, uint16_t dopplerInd, uint16_t numDopplerBins, uint32_t* cfarDetOutBitMask)
{
    uint32_t word;
    uint32_t bit;
    uint32_t bitIndex;

    bitIndex = rangeInd * numDopplerBins + dopplerInd;
    word = bitIndex >> 5;
    bit = bitIndex & 31;

    cfarDetOutBitMask[word] |= (0x1 << bit);
}
uint16_t CfarWrap_1D(float* inp, uint16_t* out, uint16_t len, uint16_t guardLen, uint16_t noiseLen, float Pfa, float* noiseout)
{
    uint16_t idx, idxLeftNext, idxLeftPrev, idxRightNext, idxRightPrev, outIdx;
    uint16_t idxCUT;
    float    sum, sumLeft, sumRight;
    float    Nnum = 2.0f * noiseLen; // 噪声窗的总长度
    // float    alpha = Nnum * (pow(Pfa, -1.0f / Nnum) - 1); // 计算门限因子alpha,需要确认
    // float alpha = 10; // 计算门限因子alpha,需要确认

    // float alpha = g_SensorCfgDefault.algo_config.th0; // 计算门限因子alpha,需要确认
    float alpha = Pfa; // 计算门限因子alpha,需要确认
    float threshold = 0.0f;

    alpha = 10.0f;
    /* 初始化变量 */
    outIdx = 0U;
    sumLeft = 0U;
    sumRight = 0U;
    for (idx = 1U; idx <= noiseLen; idx++)
    {
        sumLeft += inp[len - guardLen - idx]; // 计算初始左侧噪声窗的和
    }

    for (idx = 1U; idx <= noiseLen; idx++)
    {
        sumRight += inp[idx + guardLen]; // 计算初始右侧噪声窗的和
    }

    /* 计算初始检测单元(CUT)的门限值 */
    sum = sumLeft + sumRight;
    threshold = sum / Nnum * alpha; // 确定门限值

    if (inp[0] > threshold)
    {
        out[outIdx] = 0U;
        noiseout[outIdx] = sum / Nnum;
        outIdx++;
    }

    /* 处理从索引1到guardLen的CUT */
    idxLeftPrev = len - guardLen - noiseLen; // e.g., 32-4-8 = 20
    idxLeftNext = idxLeftPrev + noiseLen;    // e.g., 28
    idxRightPrev = 1U + guardLen;            // e.g., 1+4=5
    idxRightNext = idxRightPrev + noiseLen;  // e.g., 13
    for (idxCUT = 1U; idxCUT <= guardLen; idxCUT++)
    {
        sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev]; // 更新左侧噪声窗的和
        idxLeftNext++;
        idxLeftPrev++;
        sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev]; // 更新右侧噪声窗的和
        idxRightNext++;
        idxRightPrev++;
        sum = sumLeft + sumRight;

        threshold = sum / Nnum * alpha; // 计算新的门限值

        if ((inp[idxCUT]) > threshold)
        {
            out[outIdx] = idxCUT;
            noiseout[outIdx] = sum / Nnum;
            outIdx++;
        }
    }

    /* 处理从guardLen+1到guardLen+noiseLen的CUT */
    idxLeftNext = 0U;
    for (idxCUT = (guardLen + 1U); idxCUT <= (guardLen + noiseLen); idxCUT++)
    {
        sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev]; // 更新左侧噪声窗的和
        idxLeftNext++;
        idxLeftPrev++;
        sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev]; // 更新右侧噪声窗的和
        idxRightNext++;
        idxRightPrev++;
        sum = sumLeft + sumRight;
        threshold = sum / Nnum * alpha; // 计算新的门限值

        if ((inp[idxCUT]) > threshold)
        {
            out[outIdx] = idxCUT;
            noiseout[outIdx] = sum / Nnum;
            outIdx++;
        }
    }

    /* 处理中间的CUT */
    idxLeftPrev = 0U;
    for (idxCUT = (guardLen + noiseLen + 1U); idxCUT < (len - (noiseLen + guardLen)); idxCUT++)
    {
        sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev]; // 更新左侧噪声窗的和
        idxLeftNext++;
        idxLeftPrev++;
        sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev]; // 更新右侧噪声窗的和
        idxRightNext++;
        idxRightPrev++;
        sum = sumLeft + sumRight; // 计算CFAR

        threshold = sum / Nnum * alpha; // 计算新的门限值

        if ((inp[idxCUT]) > threshold)
        {
            out[outIdx] = idxCUT;
            noiseout[outIdx] = sum / Nnum;
            outIdx++;
        }
    }

    /*********************************************************************************************/
    /* 处理从guardLen的最后一个检测单元到len - guardLen的CUT */
    idxRightNext = 0U;
    for (idxCUT = (len - (noiseLen + guardLen)); idxCUT < (len - guardLen); idxCUT++)
    {
        sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev]; // 更新左侧噪声窗的和
        idxLeftNext++;
        idxLeftPrev++;
        sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev]; // 更新右侧噪声窗的和
        idxRightNext++;
        idxRightPrev++;
        sum = sumLeft + sumRight;
        threshold = sum / Nnum * alpha; // 计算新的门限值

        if ((inp[idxCUT]) > threshold)
        {
            out[outIdx] = idxCUT;
            noiseout[outIdx] = sum / Nnum;
            outIdx++;
        }
    }

    /* 处理最后的guardLen个检测单元 */
    idxRightPrev = 0U;
    for (idxCUT = (len - guardLen); idxCUT < len; idxCUT++)
    {
        sumLeft = (sumLeft + inp[idxLeftNext]) - inp[idxLeftPrev]; // 更新左侧噪声窗的和
        idxLeftNext++;
        idxLeftPrev++;
        sumRight = (sumRight + inp[idxRightNext]) - inp[idxRightPrev]; // 更新右侧噪声窗的和
        idxRightNext++;
        idxRightPrev++;
        sum = sumLeft + sumRight;
        threshold = sum / Nnum * alpha; // 计算新的门限值

        if ((inp[idxCUT]) > threshold)
        {
            out[outIdx] = idxCUT;
            noiseout[outIdx] = sum / Nnum;
            outIdx++;
        }
    }

    return (outIdx); // 返回检测到的目标数量
}

uint16_t CFARCA_processRangeDomain()
{
    uint16_t dopplerLine;
    uint16_t i;

    float RangeCfarDetdata[RANGE_FFT_SIZE / 2];

    uint16_t rangelen = (RANGE_FFT_SIZE / 2);
    uint16_t guardLen = 4;
    uint16_t noiseLen = 8;
    float    Pfa = 0.1f; // 虚警率
    uint16_t cfarDetOutBuffer[(RANGE_FFT_SIZE / 2)] = {0};

    uint16_t numDetObjRange = 0;

    uint16_t numDetObjPerRangeCfar = 0;
    uint16_t detIdx;
    uint16_t rangeIdx;
    uint16_t cfartype = 0;//CA-CFAR

    float noise[RANGE_FFT_SIZE / 2];

    memset(noise, 0, sizeof(float) * RANGE_FFT_SIZE / 2);

    for (dopplerLine = 0; dopplerLine < DOPPLER_FFT_SIZE; dopplerLine++)
    {
        // 确认该条doppler线上存在目标
        if (isObjectDetectedOnDopplerLine(dopplerLine, DOPPLER_FFT_SIZE, rangelen, gcfarDopplerDetOutBitMask) == false)
        {
            continue;
        }

        // 取一条doppler线上的二维fft结果进行rangecfar
        for (i = 0; i < RANGE_FFT_SIZE / 2; i++)
        {
            RangeCfarDetdata[i] = RD_Map[i * DOPPLER_FFT_SIZE + dopplerLine];
        }

        numDetObjPerRangeCfar =
            cfarCadB_SOGO(RangeCfarDetdata, cfarDetOutBuffer, cfartype, rangelen, guardLen, noiseLen, Pfa, &noise[0]);

        for (detIdx = 0; detIdx < numDetObjPerRangeCfar; detIdx++)

        {
            rangeIdx = cfarDetOutBuffer[detIdx];

            /* 判断该目标是否在dopplercfar中已经检测出来*/
            if (isObjectDetected(rangeIdx, dopplerLine, DOPPLER_FFT_SIZE, gcfarDopplerDetOutBitMask) == false)
            {
                /* Object is not detected in Doppler domain, continue for next detected objects */
                continue;
            }

            target[numDetObjRange].rangeIdx = cfarDetOutBuffer[detIdx];
            target[numDetObjRange].dopplerIdx = dopplerLine;
            target[numDetObjRange].noise2 = noise[detIdx];
            numDetObjRange++;
        }
    }

    return numDetObjRange;
}

uint16_t CFARCA_processDopplerDomain()
{
    uint16_t DetRangeIdx;
    uint16_t index;
    uint16_t len = DOPPLER_FFT_SIZE; // 检测长度
    uint16_t guardLen = 4;           // 保护半窗长度
    uint16_t noiseLen = 8;           // 噪声半窗长度
    float    Pfa = 1.0f;             // 虚警率  ,需确认
    uint16_t cfarDetOutBuffer[DOPPLER_FFT_SIZE];

    float noise[DOPPLER_FFT_SIZE];

    uint16_t numDetObjDoppler = 0;

    uint16_t numDetObjPerDopplerCfar = 0;

    memset(gcfarDopplerDetOutBitMask, 0, RANGE_FFT_SIZE / 2 * DOPPLER_FFT_SIZE / 32 * sizeof(uint32_t));
    memset(cfarDetOutBuffer, 0, DOPPLER_FFT_SIZE * sizeof(uint16_t));
    memset(noise, 0, DOPPLER_FFT_SIZE * sizeof(float));

    for (DetRangeIdx = 0; DetRangeIdx < RANGE_FFT_SIZE / 2; DetRangeIdx++)
    {
        //if ((DetRangeIdx >= 0) && (DetRangeIdx < 20))
        //{
        //    Pfa = g_SensorCfgDefault.algo_config.th0;
        //}
        //else
        //{
        //    Pfa = g_SensorCfgDefault.algo_config.th0;
        //}

        numDetObjPerDopplerCfar =
            CfarWrap_1D(&RD_Map[DetRangeIdx * DOPPLER_FFT_SIZE], cfarDetOutBuffer, len, guardLen, noiseLen, Pfa, &noise[0]);

        for (index = 0; index < numDetObjPerDopplerCfar; index++)
        {

            setCfarDetMaskLine(
                DetRangeIdx, cfarDetOutBuffer[index], DOPPLER_FFT_SIZE, gcfarDopplerDetOutBitMask); // uint16？uint32?

            target[numDetObjDoppler].rangeIdx = DetRangeIdx;
            target[numDetObjDoppler].dopplerIdx = cfarDetOutBuffer[index];
            target[numDetObjDoppler].noise1 = noise[index];
            numDetObjDoppler++;

            // isObjectDetectedOnDopplerLine[cfarDetOutBuffer[index]] = 1;

            if (numDetObjDoppler > MAXNUM_OF_DETECTION - 1)

            {
                break;
            }
        }

        if (numDetObjDoppler > MAXNUM_OF_DETECTION - 1)
        {
            break;
        }
    }

    return numDetObjDoppler;
}

void CFARprocess()
{

    uint16_t numObjs = 0;

    uint16_t index = 0;

    uint16_t dopplerIdx = 0;

    uint16_t rangeIdx = 0;

    float peak_idx = 0.0f;

    float mapvalue = 0.0f;

    float  peakIdxOffset = 0.0f;

    float x[3], y[3];

    // float distance_per_bin = PEED_OF_LIGHT_IN_METERS_PER_SEC * g_adcCfg.f32SampleRateHz *
    //                          g_SensorCfgDefault.rf_config.chirpSweepTimeUs * 1e-6 /
    //                          (2 * g_SensorCfgDefault.rf_config.rfBandwidthMHz * 1e6 * RANGE_FFT_SIZE);

    float distance_per_bin = 1.2500f;

    float velocity_per_bin = 1.1808f;
        

    gnumObjs = 0;


    memset(target, 0, sizeof(DPIF_PointCloud) * MAXNUM_OF_DETECTION);

    // doppler维cfar
    numObjs = CFARCA_processDopplerDomain();

    //  UART_Printf("numObjs1=%d\n ", numObjs);

    // // range维cfar
    numObjs = CFARCA_processRangeDomain();

    // UART_Printf("numObjs2=%d\n ", numObjs);

    // // 峰值搜索
    gnumObjs = CFARCA_peakGrouping(numObjs);

    // UART_Printf("numObjs3=%d\n ", gnumObjs);

    for (index = 0; index < gnumObjs; index++)
    {
        rangeIdx = target[index].rangeIdx;
        dopplerIdx = target[index].dopplerIdx;

        peak_idx = dopplerIdx;


        if (dopplerIdx > 0 && dopplerIdx < (DOPPLER_FFT_SIZE - 1))
        {
            x[0] = dopplerIdx - 1;
            x[1] = dopplerIdx;
            x[2] = dopplerIdx + 1;
            y[0] = RD_Map[rangeIdx * DOPPLER_FFT_SIZE + dopplerIdx - 1];
            y[1] = RD_Map[rangeIdx * DOPPLER_FFT_SIZE + dopplerIdx];
            y[2] = RD_Map[rangeIdx * DOPPLER_FFT_SIZE + dopplerIdx + 1];

            // 对3个输入点进行抛物线/二次拟合的效用函数并返回峰值的坐标。
            Dsp_Quadratic_Filter(x, y, &peak_idx, &mapvalue);
        }

        // peakIdxOffset =  DPC_ObjDet_quadInterpAroundPeak( y);   

        // peak_idx = dopplerIdx + peakIdxOffset;   //如果目标位于0频怎么处理

        if (peak_idx < DOPPLER_FFT_SIZE / 2)
        {
            target[index].velocity = peak_idx * velocity_per_bin;
        }
        else
        {
            target[index].velocity = -1.0f * (DOPPLER_FFT_SIZE - peak_idx) * velocity_per_bin;
        }


        if ((rangeIdx > 0) && (rangeIdx < RANGE_FFT_SIZE - 1))
        {

            x[0] = rangeIdx - 1;
            x[1] = rangeIdx;
            x[2] = rangeIdx + 1;
            y[0] = RD_Map[(rangeIdx - 1) * DOPPLER_FFT_SIZE + dopplerIdx];
            y[1] = RD_Map[rangeIdx * DOPPLER_FFT_SIZE + dopplerIdx];
            y[2] = RD_Map[(rangeIdx + 1) * DOPPLER_FFT_SIZE + dopplerIdx];

            Dsp_Quadratic_Filter(x, y, &peak_idx, &mapvalue);

            // peakIdxOffset =  DPC_ObjDet_quadInterpAroundPeak( y);   

            // peak_idx = rangeIdx + peakIdxOffset;   //如果目标位于0频怎么处理



            target[index].range = peak_idx * distance_per_bin; // 目标距离 m
        }
        else
        {
            target[index].range = rangeIdx * distance_per_bin; // 目标距离 m        
        }

        target[index].strength = 20 * log10f(RD_Map[rangeIdx * DOPPLER_FFT_SIZE + dopplerIdx]);

        target[index].state = 1;

    }

}




void FFT_Abs_f32(const float* pSrc, float* pDst, uint32_t blockSize)
{
    float real, imag, min, max, absvalue;
    uint32_t blkCnt = blockSize;                               /* Loop counter */

    while (blkCnt > 0U)
    {
        /* C = |A| */
        real = fabsf(*pSrc++);
        imag = fabsf(*pSrc++);

        max = imag;
        min = real;

        if (real > imag)
        {
            max = real;
            min = imag;
        }

        /* Approximate absolute function.
             * abs (x[i] + ix[i+1]) \approx  (max + min*3/8) */
        absvalue = max + min * 0.375f;

        *pDst++ = absvalue;
        /* Decrement loop counter */
        blkCnt--;
    }
}
void arm_bitreversal_32(
    uint32_t* pSrc,
    const uint16_t bitRevLen,
    const uint16_t* pBitRevTab)
{
    uint32_t a, b, i, tmp;

    for (i = 0; i < bitRevLen; )
    {
        a = pBitRevTab[i] >> 2;
        b = pBitRevTab[i + 1] >> 2;

        //real
        tmp = pSrc[a];
        pSrc[a] = pSrc[b];
        pSrc[b] = tmp;

        //complex
        tmp = pSrc[a + 1];
        pSrc[a + 1] = pSrc[b + 1];
        pSrc[b + 1] = tmp;

        i += 2;
    }
}

void arm_radix8_butterfly_f32(
    float32_t* pSrc,
    uint16_t fftLen,
    const float32_t* pCoef,
    uint16_t twidCoefModifier)
{
    uint32_t ia1, ia2, ia3, ia4, ia5, ia6, ia7;
    uint32_t i1, i2, i3, i4, i5, i6, i7, i8;
    uint32_t id;
    uint32_t n1, n2, j;

    float32_t r1, r2, r3, r4, r5, r6, r7, r8;
    float32_t t1, t2;
    float32_t s1, s2, s3, s4, s5, s6, s7, s8;
    float32_t p1, p2, p3, p4;
    float32_t co2, co3, co4, co5, co6, co7, co8;
    float32_t si2, si3, si4, si5, si6, si7, si8;
    const float32_t C81 = 0.70710678118f;

    n2 = fftLen;

    do
    {
        n1 = n2;
        n2 = n2 >> 3;
        i1 = 0;

        do
        {
            i2 = i1 + n2;
            i3 = i2 + n2;
            i4 = i3 + n2;
            i5 = i4 + n2;
            i6 = i5 + n2;
            i7 = i6 + n2;
            i8 = i7 + n2;
            r1 = pSrc[2 * i1] + pSrc[2 * i5];
            r5 = pSrc[2 * i1] - pSrc[2 * i5];
            r2 = pSrc[2 * i2] + pSrc[2 * i6];
            r6 = pSrc[2 * i2] - pSrc[2 * i6];
            r3 = pSrc[2 * i3] + pSrc[2 * i7];
            r7 = pSrc[2 * i3] - pSrc[2 * i7];
            r4 = pSrc[2 * i4] + pSrc[2 * i8];
            r8 = pSrc[2 * i4] - pSrc[2 * i8];
            t1 = r1 - r3;
            r1 = r1 + r3;
            r3 = r2 - r4;
            r2 = r2 + r4;
            pSrc[2 * i1] = r1 + r2;
            pSrc[2 * i5] = r1 - r2;
            r1 = pSrc[2 * i1 + 1] + pSrc[2 * i5 + 1];
            s5 = pSrc[2 * i1 + 1] - pSrc[2 * i5 + 1];
            r2 = pSrc[2 * i2 + 1] + pSrc[2 * i6 + 1];
            s6 = pSrc[2 * i2 + 1] - pSrc[2 * i6 + 1];
            s3 = pSrc[2 * i3 + 1] + pSrc[2 * i7 + 1];
            s7 = pSrc[2 * i3 + 1] - pSrc[2 * i7 + 1];
            r4 = pSrc[2 * i4 + 1] + pSrc[2 * i8 + 1];
            s8 = pSrc[2 * i4 + 1] - pSrc[2 * i8 + 1];
            t2 = r1 - s3;
            r1 = r1 + s3;
            s3 = r2 - r4;
            r2 = r2 + r4;
            pSrc[2 * i1 + 1] = r1 + r2;
            pSrc[2 * i5 + 1] = r1 - r2;
            pSrc[2 * i3] = t1 + s3;
            pSrc[2 * i7] = t1 - s3;
            pSrc[2 * i3 + 1] = t2 - r3;
            pSrc[2 * i7 + 1] = t2 + r3;
            r1 = (r6 - r8) * C81;
            r6 = (r6 + r8) * C81;
            r2 = (s6 - s8) * C81;
            s6 = (s6 + s8) * C81;
            t1 = r5 - r1;
            r5 = r5 + r1;
            r8 = r7 - r6;
            r7 = r7 + r6;
            t2 = s5 - r2;
            s5 = s5 + r2;
            s8 = s7 - s6;
            s7 = s7 + s6;
            pSrc[2 * i2] = r5 + s7;
            pSrc[2 * i8] = r5 - s7;
            pSrc[2 * i6] = t1 + s8;
            pSrc[2 * i4] = t1 - s8;
            pSrc[2 * i2 + 1] = s5 - r7;
            pSrc[2 * i8 + 1] = s5 + r7;
            pSrc[2 * i6 + 1] = t2 - r8;
            pSrc[2 * i4 + 1] = t2 + r8;

            i1 += n1;
        } while (i1 < fftLen);

        if (n2 < 8)
            break;

        ia1 = 0;
        j = 1;

        do
        {
            /*  index calculation for the coefficients */
            id = ia1 + twidCoefModifier;
            ia1 = id;
            ia2 = ia1 + id;
            ia3 = ia2 + id;
            ia4 = ia3 + id;
            ia5 = ia4 + id;
            ia6 = ia5 + id;
            ia7 = ia6 + id;

            co2 = pCoef[2 * ia1];
            co3 = pCoef[2 * ia2];
            co4 = pCoef[2 * ia3];
            co5 = pCoef[2 * ia4];
            co6 = pCoef[2 * ia5];
            co7 = pCoef[2 * ia6];
            co8 = pCoef[2 * ia7];
            si2 = pCoef[2 * ia1 + 1];
            si3 = pCoef[2 * ia2 + 1];
            si4 = pCoef[2 * ia3 + 1];
            si5 = pCoef[2 * ia4 + 1];
            si6 = pCoef[2 * ia5 + 1];
            si7 = pCoef[2 * ia6 + 1];
            si8 = pCoef[2 * ia7 + 1];

            i1 = j;

            do
            {
                /*  index calculation for the input */
                i2 = i1 + n2;
                i3 = i2 + n2;
                i4 = i3 + n2;
                i5 = i4 + n2;
                i6 = i5 + n2;
                i7 = i6 + n2;
                i8 = i7 + n2;
                r1 = pSrc[2 * i1] + pSrc[2 * i5];
                r5 = pSrc[2 * i1] - pSrc[2 * i5];
                r2 = pSrc[2 * i2] + pSrc[2 * i6];
                r6 = pSrc[2 * i2] - pSrc[2 * i6];
                r3 = pSrc[2 * i3] + pSrc[2 * i7];
                r7 = pSrc[2 * i3] - pSrc[2 * i7];
                r4 = pSrc[2 * i4] + pSrc[2 * i8];
                r8 = pSrc[2 * i4] - pSrc[2 * i8];
                t1 = r1 - r3;
                r1 = r1 + r3;
                r3 = r2 - r4;
                r2 = r2 + r4;
                pSrc[2 * i1] = r1 + r2;
                r2 = r1 - r2;
                s1 = pSrc[2 * i1 + 1] + pSrc[2 * i5 + 1];
                s5 = pSrc[2 * i1 + 1] - pSrc[2 * i5 + 1];
                s2 = pSrc[2 * i2 + 1] + pSrc[2 * i6 + 1];
                s6 = pSrc[2 * i2 + 1] - pSrc[2 * i6 + 1];
                s3 = pSrc[2 * i3 + 1] + pSrc[2 * i7 + 1];
                s7 = pSrc[2 * i3 + 1] - pSrc[2 * i7 + 1];
                s4 = pSrc[2 * i4 + 1] + pSrc[2 * i8 + 1];
                s8 = pSrc[2 * i4 + 1] - pSrc[2 * i8 + 1];
                t2 = s1 - s3;
                s1 = s1 + s3;
                s3 = s2 - s4;
                s2 = s2 + s4;
                r1 = t1 + s3;
                t1 = t1 - s3;
                pSrc[2 * i1 + 1] = s1 + s2;
                s2 = s1 - s2;
                s1 = t2 - r3;
                t2 = t2 + r3;
                p1 = co5 * r2;
                p2 = si5 * s2;
                p3 = co5 * s2;
                p4 = si5 * r2;
                pSrc[2 * i5] = p1 + p2;
                pSrc[2 * i5 + 1] = p3 - p4;
                p1 = co3 * r1;
                p2 = si3 * s1;
                p3 = co3 * s1;
                p4 = si3 * r1;
                pSrc[2 * i3] = p1 + p2;
                pSrc[2 * i3 + 1] = p3 - p4;
                p1 = co7 * t1;
                p2 = si7 * t2;
                p3 = co7 * t2;
                p4 = si7 * t1;
                pSrc[2 * i7] = p1 + p2;
                pSrc[2 * i7 + 1] = p3 - p4;
                r1 = (r6 - r8) * C81;
                r6 = (r6 + r8) * C81;
                s1 = (s6 - s8) * C81;
                s6 = (s6 + s8) * C81;
                t1 = r5 - r1;
                r5 = r5 + r1;
                r8 = r7 - r6;
                r7 = r7 + r6;
                t2 = s5 - s1;
                s5 = s5 + s1;
                s8 = s7 - s6;
                s7 = s7 + s6;
                r1 = r5 + s7;
                r5 = r5 - s7;
                r6 = t1 + s8;
                t1 = t1 - s8;
                s1 = s5 - r7;
                s5 = s5 + r7;
                s6 = t2 - r8;
                t2 = t2 + r8;
                p1 = co2 * r1;
                p2 = si2 * s1;
                p3 = co2 * s1;
                p4 = si2 * r1;
                pSrc[2 * i2] = p1 + p2;
                pSrc[2 * i2 + 1] = p3 - p4;
                p1 = co8 * r5;
                p2 = si8 * s5;
                p3 = co8 * s5;
                p4 = si8 * r5;
                pSrc[2 * i8] = p1 + p2;
                pSrc[2 * i8 + 1] = p3 - p4;
                p1 = co6 * r6;
                p2 = si6 * s6;
                p3 = co6 * s6;
                p4 = si6 * r6;
                pSrc[2 * i6] = p1 + p2;
                pSrc[2 * i6 + 1] = p3 - p4;
                p1 = co4 * t1;
                p2 = si4 * t2;
                p3 = co4 * t2;
                p4 = si4 * t1;
                pSrc[2 * i4] = p1 + p2;
                pSrc[2 * i4 + 1] = p3 - p4;

                i1 += n1;
            } while (i1 < fftLen);

            j++;
        } while (j < n2);

        twidCoefModifier <<= 3;
    } while (n2 > 7);
}



void arm_cfft_radix8by4_f32(arm_cfft_instance_f32* S, float32_t* p1)
{
    uint32_t    L = S->fftLen >> 1;
    float32_t* pCol1, * pCol2, * pCol3, * pCol4, * pEnd1, * pEnd2, * pEnd3, * pEnd4;
    const float32_t* tw2, * tw3, * tw4;
    float32_t* p2 = p1 + L;
    float32_t* p3 = p2 + L;
    float32_t* p4 = p3 + L;
    float32_t t2[4], t3[4], t4[4], twR, twI;
    float32_t p1ap3_0, p1sp3_0, p1ap3_1, p1sp3_1;
    float32_t m0, m1, m2, m3;
    uint32_t l, twMod2, twMod3, twMod4;

    pCol1 = p1;         /* points to real values by default */
    pCol2 = p2;
    pCol3 = p3;
    pCol4 = p4;
    pEnd1 = p2 - 1;     /* points to imaginary values by default */
    pEnd2 = p3 - 1;
    pEnd3 = p4 - 1;
    pEnd4 = pEnd3 + L;

    tw2 = tw3 = tw4 = (float32_t*)S->pTwiddle;

    L >>= 1;

    /* do four dot Fourier transform */

    twMod2 = 2;
    twMod3 = 4;
    twMod4 = 6;

    /* TOP */
    p1ap3_0 = p1[0] + p3[0];
    p1sp3_0 = p1[0] - p3[0];
    p1ap3_1 = p1[1] + p3[1];
    p1sp3_1 = p1[1] - p3[1];

    /* col 2 */
    t2[0] = p1sp3_0 + p2[1] - p4[1];
    t2[1] = p1sp3_1 - p2[0] + p4[0];
    /* col 3 */
    t3[0] = p1ap3_0 - p2[0] - p4[0];
    t3[1] = p1ap3_1 - p2[1] - p4[1];
    /* col 4 */
    t4[0] = p1sp3_0 - p2[1] + p4[1];
    t4[1] = p1sp3_1 + p2[0] - p4[0];
    /* col 1 */
    *p1++ = p1ap3_0 + p2[0] + p4[0];
    *p1++ = p1ap3_1 + p2[1] + p4[1];

    /* Twiddle factors are ones */
    *p2++ = t2[0];
    *p2++ = t2[1];
    *p3++ = t3[0];
    *p3++ = t3[1];
    *p4++ = t4[0];
    *p4++ = t4[1];

    tw2 += twMod2;
    tw3 += twMod3;
    tw4 += twMod4;

    for (l = (L - 2) >> 1; l > 0; l--)
    {
        /* TOP */
        p1ap3_0 = p1[0] + p3[0];
        p1sp3_0 = p1[0] - p3[0];
        p1ap3_1 = p1[1] + p3[1];
        p1sp3_1 = p1[1] - p3[1];
        /* col 2 */
        t2[0] = p1sp3_0 + p2[1] - p4[1];
        t2[1] = p1sp3_1 - p2[0] + p4[0];
        /* col 3 */
        t3[0] = p1ap3_0 - p2[0] - p4[0];
        t3[1] = p1ap3_1 - p2[1] - p4[1];
        /* col 4 */
        t4[0] = p1sp3_0 - p2[1] + p4[1];
        t4[1] = p1sp3_1 + p2[0] - p4[0];
        /* col 1 - top */
        *p1++ = p1ap3_0 + p2[0] + p4[0];
        *p1++ = p1ap3_1 + p2[1] + p4[1];

        /* BOTTOM */
        p1ap3_1 = pEnd1[-1] + pEnd3[-1];
        p1sp3_1 = pEnd1[-1] - pEnd3[-1];
        p1ap3_0 = pEnd1[0] + pEnd3[0];
        p1sp3_0 = pEnd1[0] - pEnd3[0];
        /* col 2 */
        t2[2] = pEnd2[0] - pEnd4[0] + p1sp3_1;
        t2[3] = pEnd1[0] - pEnd3[0] - pEnd2[-1] + pEnd4[-1];
        /* col 3 */
        t3[2] = p1ap3_1 - pEnd2[-1] - pEnd4[-1];
        t3[3] = p1ap3_0 - pEnd2[0] - pEnd4[0];
        /* col 4 */
        t4[2] = pEnd2[0] - pEnd4[0] - p1sp3_1;
        t4[3] = pEnd4[-1] - pEnd2[-1] - p1sp3_0;
        /* col 1 - Bottom */
        *pEnd1-- = p1ap3_0 + pEnd2[0] + pEnd4[0];
        *pEnd1-- = p1ap3_1 + pEnd2[-1] + pEnd4[-1];

        /* COL 2 */
        /* read twiddle factors */
        twR = *tw2++;
        twI = *tw2++;
        /* multiply by twiddle factors */
        /*  let    Z1 = a + i(b),   Z2 = c + i(d) */
        /*   =>  Z1 * Z2  =  (a*c - b*d) + i(b*c + a*d) */

        /* Top */
        m0 = t2[0] * twR;
        m1 = t2[1] * twI;
        m2 = t2[1] * twR;
        m3 = t2[0] * twI;

        *p2++ = m0 + m1;
        *p2++ = m2 - m3;
        /* use vertical symmetry col 2 */
        /* 0.9997 - 0.0245i  <==>  0.0245 - 0.9997i */
        /* Bottom */
        m0 = t2[3] * twI;
        m1 = t2[2] * twR;
        m2 = t2[2] * twI;
        m3 = t2[3] * twR;

        *pEnd2-- = m0 - m1;
        *pEnd2-- = m2 + m3;

        /* COL 3 */
        twR = tw3[0];
        twI = tw3[1];
        tw3 += twMod3;
        /* Top */
        m0 = t3[0] * twR;
        m1 = t3[1] * twI;
        m2 = t3[1] * twR;
        m3 = t3[0] * twI;

        *p3++ = m0 + m1;
        *p3++ = m2 - m3;
        /* use vertical symmetry col 3 */
        /* 0.9988 - 0.0491i  <==>  -0.9988 - 0.0491i */
        /* Bottom */
        m0 = -t3[3] * twR;
        m1 = t3[2] * twI;
        m2 = t3[2] * twR;
        m3 = t3[3] * twI;

        *pEnd3-- = m0 - m1;
        *pEnd3-- = m3 - m2;

        /* COL 4 */
        twR = tw4[0];
        twI = tw4[1];
        tw4 += twMod4;
        /* Top */
        m0 = t4[0] * twR;
        m1 = t4[1] * twI;
        m2 = t4[1] * twR;
        m3 = t4[0] * twI;

        *p4++ = m0 + m1;
        *p4++ = m2 - m3;
        /* use vertical symmetry col 4 */
        /* 0.9973 - 0.0736i  <==>  -0.0736 + 0.9973i */
        /* Bottom */
        m0 = t4[3] * twI;
        m1 = t4[2] * twR;
        m2 = t4[2] * twI;
        m3 = t4[3] * twR;

        *pEnd4-- = m0 - m1;
        *pEnd4-- = m2 + m3;
    }

    /* MIDDLE */
    /* Twiddle factors are */
    /*  1.0000  0.7071-0.7071i  -1.0000i  -0.7071-0.7071i */
    p1ap3_0 = p1[0] + p3[0];
    p1sp3_0 = p1[0] - p3[0];
    p1ap3_1 = p1[1] + p3[1];
    p1sp3_1 = p1[1] - p3[1];

    /* col 2 */
    t2[0] = p1sp3_0 + p2[1] - p4[1];
    t2[1] = p1sp3_1 - p2[0] + p4[0];
    /* col 3 */
    t3[0] = p1ap3_0 - p2[0] - p4[0];
    t3[1] = p1ap3_1 - p2[1] - p4[1];
    /* col 4 */
    t4[0] = p1sp3_0 - p2[1] + p4[1];
    t4[1] = p1sp3_1 + p2[0] - p4[0];
    /* col 1 - Top */
    *p1++ = p1ap3_0 + p2[0] + p4[0];
    *p1++ = p1ap3_1 + p2[1] + p4[1];

    /* COL 2 */
    twR = tw2[0];
    twI = tw2[1];

    m0 = t2[0] * twR;
    m1 = t2[1] * twI;
    m2 = t2[1] * twR;
    m3 = t2[0] * twI;

    *p2++ = m0 + m1;
    *p2++ = m2 - m3;
    /* COL 3 */
    twR = tw3[0];
    twI = tw3[1];

    m0 = t3[0] * twR;
    m1 = t3[1] * twI;
    m2 = t3[1] * twR;
    m3 = t3[0] * twI;

    *p3++ = m0 + m1;
    *p3++ = m2 - m3;
    /* COL 4 */
    twR = tw4[0];
    twI = tw4[1];

    m0 = t4[0] * twR;
    m1 = t4[1] * twI;
    m2 = t4[1] * twR;
    m3 = t4[0] * twI;

    *p4++ = m0 + m1;
    *p4++ = m2 - m3;

    /* first col */
    arm_radix8_butterfly_f32(pCol1, L, (float32_t*)S->pTwiddle, 4U);

    /* second col */
    arm_radix8_butterfly_f32(pCol2, L, (float32_t*)S->pTwiddle, 4U);

    /* third col */
    arm_radix8_butterfly_f32(pCol3, L, (float32_t*)S->pTwiddle, 4U);

    /* fourth col */
    arm_radix8_butterfly_f32(pCol4, L, (float32_t*)S->pTwiddle, 4U);
}



void arm_cfft_radix8by2_f32(arm_cfft_instance_f32* S, float32_t* p1)
{
    uint32_t    L = S->fftLen;
    float32_t* pCol1, * pCol2, * pMid1, * pMid2;
    float32_t* p2 = p1 + L;
    const float32_t* tw = (float32_t*)S->pTwiddle;
    float32_t t1[4], t2[4], t3[4], t4[4], twR, twI;
    float32_t m0, m1, m2, m3;
    uint32_t l;

    pCol1 = p1;
    pCol2 = p2;

    /* Define new length */
    L >>= 1;

    /* Initialize mid pointers */
    pMid1 = p1 + L;
    pMid2 = p2 + L;

    /* do two dot Fourier transform */
    for (l = L >> 2; l > 0; l--)
    {
        t1[0] = p1[0];
        t1[1] = p1[1];
        t1[2] = p1[2];
        t1[3] = p1[3];

        t2[0] = p2[0];
        t2[1] = p2[1];
        t2[2] = p2[2];
        t2[3] = p2[3];

        t3[0] = pMid1[0];
        t3[1] = pMid1[1];
        t3[2] = pMid1[2];
        t3[3] = pMid1[3];

        t4[0] = pMid2[0];
        t4[1] = pMid2[1];
        t4[2] = pMid2[2];
        t4[3] = pMid2[3];

        *p1++ = t1[0] + t2[0];
        *p1++ = t1[1] + t2[1];
        *p1++ = t1[2] + t2[2];
        *p1++ = t1[3] + t2[3];    /* col 1 */

        t2[0] = t1[0] - t2[0];
        t2[1] = t1[1] - t2[1];
        t2[2] = t1[2] - t2[2];
        t2[3] = t1[3] - t2[3];    /* for col 2 */

        *pMid1++ = t3[0] + t4[0];
        *pMid1++ = t3[1] + t4[1];
        *pMid1++ = t3[2] + t4[2];
        *pMid1++ = t3[3] + t4[3]; /* col 1 */

        t4[0] = t4[0] - t3[0];
        t4[1] = t4[1] - t3[1];
        t4[2] = t4[2] - t3[2];
        t4[3] = t4[3] - t3[3];    /* for col 2 */

        twR = *tw++;
        twI = *tw++;

        /* multiply by twiddle factors */
        m0 = t2[0] * twR;
        m1 = t2[1] * twI;
        m2 = t2[1] * twR;
        m3 = t2[0] * twI;

        /* R  =  R  *  Tr - I * Ti */
        *p2++ = m0 + m1;
        /* I  =  I  *  Tr + R * Ti */
        *p2++ = m2 - m3;

        /* use vertical symmetry */
        /*  0.9988 - 0.0491i <==> -0.0491 - 0.9988i */
        m0 = t4[0] * twI;
        m1 = t4[1] * twR;
        m2 = t4[1] * twI;
        m3 = t4[0] * twR;

        *pMid2++ = m0 - m1;
        *pMid2++ = m2 + m3;

        twR = *tw++;
        twI = *tw++;

        m0 = t2[2] * twR;
        m1 = t2[3] * twI;
        m2 = t2[3] * twR;
        m3 = t2[2] * twI;

        *p2++ = m0 + m1;
        *p2++ = m2 - m3;

        m0 = t4[2] * twI;
        m1 = t4[3] * twR;
        m2 = t4[3] * twI;
        m3 = t4[2] * twR;

        *pMid2++ = m0 - m1;
        *pMid2++ = m2 + m3;
    }

    /* first col */
    arm_radix8_butterfly_f32(pCol1, L, (float32_t*)S->pTwiddle, 2U);

    /* second col */
    arm_radix8_butterfly_f32(pCol2, L, (float32_t*)S->pTwiddle, 2U);
}


void arm_cfft_f32(
    const arm_cfft_instance_f32* S,
    float32_t* p1,
    uint8_t ifftFlag,
    uint8_t bitReverseFlag)
{
    uint32_t  L = S->fftLen, l;
    float32_t invL, * pSrc;

    if (ifftFlag == 1U)
    {
        /* Conjugate input data */
        pSrc = p1 + 1;
        for (l = 0; l < L; l++)
        {
            *pSrc = -*pSrc;
            pSrc += 2;
        }
    }

    switch (L)
    {
    case 16:
    case 128:
    case 1024:
        arm_cfft_radix8by2_f32((arm_cfft_instance_f32*)S, p1);
        break;
    case 32:
    case 256:
    case 2048:
        arm_cfft_radix8by4_f32((arm_cfft_instance_f32*)S, p1);
        break;
    case 64:
    case 512:
    case 4096:
        arm_radix8_butterfly_f32(p1, L, (float32_t*)S->pTwiddle, 1);
        break;
    }

    if (bitReverseFlag)
        arm_bitreversal_32((uint32_t*)p1, S->bitRevLength, S->pBitRevTable);

    if (ifftFlag == 1U)
    {
        invL = 1.0f / (float32_t)L;

        /* Conjugate and scale output data */
        pSrc = p1;
        for (l = 0; l < L; l++)
        {
            *pSrc++ *= invL;
            *pSrc = -(*pSrc) * invL;
            pSrc++;
        }
    }
}

/**
 ******************************************************************************
 ** \brief  复数FFT计算
 ** @param  pf32FftSignal, complex形式， 信号输入：[I,Q,I,Q...]  FFT输出 [Re,Im,Re,Im...]
 ** @param  u16SignalLen, 输入数据长度
 ** @param  pf32WindowBuffer  窗函数buffer.若窗函数指针为NULL，则不加窗
 ** @param  enFftType  0-IQ  1-仅I  2-仅Q
 ** @param  S, ARM_FFT计算结构体

 ** \return 无
 ******************************************************************************/
static void FFT_Compute_Signal_f32(float* pf32FftSignal,
    uint16_t                     u16SignalLen,
    float* pf32WindowBuffer,
    en_fft_type_t                enFftType,
    bool                         bRemoveMean,
    const arm_cfft_instance_f32* f32S)
{
    if (f32S == NULL)
        return;

    uint16_t u16FftSize = f32S->fftLen;

    /*计算平均值*/
    float f32ISum = 0;
    float f32QSum = 0;
    float f32IMean = 0, f32QMean = 0;
    if (bRemoveMean)
    {
        for (uint16_t i = 0; i < u16SignalLen; i++)
        {
            f32ISum += pf32FftSignal[2 * i];
            f32QSum += pf32FftSignal[2 * i + 1];
        }
        f32IMean = f32ISum / (float)u16SignalLen;
        f32QMean = f32QSum / (float)u16SignalLen;
    }
    for (uint16_t i = 0; i < u16SignalLen; i++)
    {

        if (enFftType == FFT_IQ)
        {
            /* C = A - B */
            /* Subtract and store result in destination buffer. */
            pf32FftSignal[2 * i] = pf32FftSignal[2 * i] - f32IMean;
            pf32FftSignal[2 * i + 1] = pf32FftSignal[2 * i + 1] - f32QMean;
            if (pf32WindowBuffer != NULL)
            {
                /* C = A * B */
                /* Multiply inputs and store result in destination buffer. */
                pf32FftSignal[2 * i] = pf32FftSignal[2 * i] * pf32WindowBuffer[i];
                pf32FftSignal[2 * i + 1] = pf32FftSignal[2 * i + 1] * pf32WindowBuffer[i];
            }
        }
        else if (enFftType == FFT_IOnly)
        {
            /* C = A - B */
            /* Subtract and store result in destination buffer. */
            pf32FftSignal[2 * i] = pf32FftSignal[2 * i] - f32IMean;
            pf32FftSignal[2 * i + 1] = 0;
            if (pf32WindowBuffer != NULL)
            {
                /* C = A * B */
                /* Multiply inputs and store result in destination buffer. */
                pf32FftSignal[2 * i] = pf32FftSignal[2 * i] * pf32WindowBuffer[i];
            }
        }
        else // FFT_QOnly
        {
            pf32FftSignal[2 * i] = 0;
            /* C = A - B */
            /* Subtract and store result in destination buffer. */
            pf32FftSignal[2 * i + 1] = pf32FftSignal[2 * i + 1] - f32QMean;
            if (pf32WindowBuffer != NULL)
            {
                /* C = A * B */
                /* Multiply inputs and store result in destination buffer. */
                pf32FftSignal[2 * i + 1] = pf32FftSignal[2 * i + 1] * pf32WindowBuffer[i];
            }
        }
    }

    /* Zero Padding */
    for (uint16_t i = u16SignalLen; i < u16FftSize; i++)
    {
        pf32FftSignal[2 * i] = 0;
        pf32FftSignal[2 * i + 1] = 0;
    }
    arm_cfft_f32(f32S, pf32FftSignal, 0, 1);
}


void cfft_f32(arm_cfft_instance_f32* instance,
    float* pf32FftSignal,
    uint16_t           u16SignalLen,
    float* pf32WindowBuffer,
    en_fft_type_t      enFftType,
    bool               bRemoveMean)
{
    FFT_Compute_Signal_f32(pf32FftSignal, u16SignalLen, pf32WindowBuffer, enFftType, bRemoveMean, instance);
}

void RangeFFT(uint8_t dopplerLine, float* adcbuffer)
{
    uint16_t jj = 0;

    float processtime2 = 0.0f;

    for (jj = 0; jj < MTI_Size; jj++) // 需要确认MTI_Size的大小
    {
        //rangefft_temp[2 * jj] = (float)adcbuffer[2*jj];
        //rangefft_temp[2 * jj + 1] = (float)adcbuffer[2*jj+1];
        gRadarCubeTemp[jj].real = (float)adcbuffer[2 * jj];
        gRadarCubeTemp[jj].image = (float)adcbuffer[2 * jj+1]; //for complex fft

        gRadarCubeTemp1[jj].real = (float)adcbuffer[2* RANGE_FFT_SIZE * DOPPLER_FFT_SIZE + 2 * jj];
        gRadarCubeTemp1[jj].image =(float)adcbuffer[2 * RANGE_FFT_SIZE * DOPPLER_FFT_SIZE + 2 * jj + 1]; //for complex fft
        //printf("%f", rangefft_temp[2 * jj]);
        //printf(",");
        //printf("%f", rangefft_temp[2 * jj + 1]);
        //printf(",");
    }

    //cfft_f32(&czt_fft_256, rangefft_temp, MTI_Size, (float*)f32WindowBlackmanharris128, FFT_IQ, 0);
    win_fft( 128, gRadarCubeTemp,  0, 32768.0);
    win_fft(128, gRadarCubeTemp1, 0, 32768.0);   //rangefft for RX2


    for (jj = 0; jj < RANGE_FFT_SIZE ; jj++)
    {
        rangefft[jj * DOPPLER_FFT_SIZE * 2 + dopplerLine * 2] = gRadarCubeTemp[jj].real; // I
        rangefft[jj * DOPPLER_FFT_SIZE * 2 + dopplerLine * 2 + 1] = gRadarCubeTemp[jj].image; // Q

        rangefft1[jj * DOPPLER_FFT_SIZE * 2 + dopplerLine * 2] = gRadarCubeTemp1[jj].real; // I
        rangefft1[jj * DOPPLER_FFT_SIZE * 2 + dopplerLine * 2 + 1] = gRadarCubeTemp1[jj].image; // Q result for R1

        //printf("%f", rangefft[jj * DOPPLER_FFT_SIZE * 2 + dopplerLine * 2]);
        //printf(",");
        //printf("%f", rangefft[jj * DOPPLER_FFT_SIZE * 2 + dopplerLine * 2 + 1]);
        //printf(",");                                                                   result is ok
    }
}


void DopplerProcess()
{

    for (uint16_t ii = 0; ii < RANGE_FFT_SIZE; ii++)
    {
        for (uint16_t kk = 0; kk < DOPPLER_FFT_SIZE; kk++)
        {
            gRadarCubeDTemp[kk].real = rangefft[ii* DOPPLER_FFT_SIZE * 2+kk*2];
            gRadarCubeDTemp[kk].image = rangefft[ii * DOPPLER_FFT_SIZE * 2+ kk * 2+1];

            gRadarCubeDTemp1[kk].real = rangefft1[ii * DOPPLER_FFT_SIZE * 2 + kk * 2];
            gRadarCubeDTemp1[kk].image = rangefft1[ii * DOPPLER_FFT_SIZE * 2 + kk * 2 + 1];

        }


        //cfft_f32(&czt_fft_256,
        //    &rangefft[ii * DOPPLER_FFT_SIZE * 2],
        //    DOPPLER_FFT_SIZE,
        //    (float*)f32WindowHanning32,
        //    FFT_IQ,
        //    1);
        win_fft(32, gRadarCubeDTemp, 0, 32768.0);
        win_fft(32, gRadarCubeDTemp1, 0, 32768.0);

 /*       for (uint16_t kkk =0;kkk<32;kkk++)
        {
            printf("%f", gRadarCubeDTemp[kkk].real);
            printf(",");
            printf("%f", gRadarCubeDTemp[kkk].image);
            printf(","); 
        }  */                                          //result is ok
             
           for (uint16_t kkk =0;kkk<32;kkk++)
        {
               RD_Map_Dopplerffttemp[ii * DOPPLER_FFT_SIZE*2 +2*kkk] = gRadarCubeDTemp[kkk].real;
               RD_Map_Dopplerffttemp[ii * DOPPLER_FFT_SIZE*2 +2*kkk+1] = gRadarCubeDTemp[kkk].image;

               RD_Map_Dopplerffttemp1[ii * DOPPLER_FFT_SIZE * 2 + 2 * kkk] = gRadarCubeDTemp1[kkk].real;
               RD_Map_Dopplerffttemp1[ii * DOPPLER_FFT_SIZE * 2 + 2 * kkk + 1] = gRadarCubeDTemp1[kkk].image;

                 //printf("%f", gRadarCubeDTemp[kkk].real);
                 //printf(",");
                 //printf("%f", gRadarCubeDTemp[kkk].image);
                 //printf(",");       //result is ok
                                                        
        }                                          

        FFT_Abs_f32(&RD_Map_Dopplerffttemp[ii * DOPPLER_FFT_SIZE*2], &RD_Map[ii * DOPPLER_FFT_SIZE], DOPPLER_FFT_SIZE);
        FFT_Abs_f32(&RD_Map_Dopplerffttemp1[ii * DOPPLER_FFT_SIZE * 2], &RD_Map1[ii * DOPPLER_FFT_SIZE], DOPPLER_FFT_SIZE);



            for (uint16_t test_k =0;test_k < DOPPLER_FFT_SIZE;test_k++)
            {
                //printf("%f", RD_Map[ii * DOPPLER_FFT_SIZE + test_k]);
                //printf(",");
            }                                                            //result is ok                    

    }

    //非相干累积
    for (uint16_t mapidx = 0;mapidx < DOPPLER_FFT_SIZE * RANGE_FFT_SIZE;mapidx++)
    {
        RD_Map[mapidx] = RD_Map[mapidx] + RD_Map1[mapidx];
    }


}