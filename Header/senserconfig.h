#include<stdint.h>
#include<stdio.h>
#include <string.h>


/*****************************************************************************/
/** \file sensor_config.h
 **
 **
 ** History:
 **   - 2021-06-15        First version
 **
 *****************************************************************************/
#ifndef __SENSOR_CONFIG_H__
#define __SENSOR_CONFIG_H__
 /*****************************************************************************
  * Include files
  *****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif
    /**
     ******************************************************************************
     ** \defgroup __SENSOR_CONFIG_H__
     **
     ******************************************************************************/
     //@{

     /**
      *******************************************************************************
      ** function prototypes.
      ******************************************************************************/

      /******************************************************************************
       * Global type definitions
       ******************************************************************************/

       /******************************************************************************
        * Global definitions
        ******************************************************************************/
#define RANGE_FFT_SIZE        128
#define DOPPLER_FFT_SIZE      64
#define CURRENT_NUM_OF_TRACKS 2
#define MTI_Size              128
#define PRINT_FLAG            0
#define MAXNUM_OF_DETECTION (32U)

#define NumTx                2
#define NumRx                4
#define NumchannelMimo       12
#define RDatalength         (RANGE_FFT_SIZE * DOPPLER_FFT_SIZE)



        // #define UART_DEVICE                                   UART5
#define UART_BAUDRATE 9600 // 9600 // 460800//256000//2400
#define SPI_DEVICE    SPI0

#define LOW_POWER_MODE_ENABLE 0    /*! 开启/关闭MCU低功耗模式*/
#define CALC_VPP_ENABLE       1    /*! 开启、关闭计算幅度（时域）*/

#define SENSOR_TRX_MODE      (1U)  /**< TRX mode: 0:1T1R 1:1T2R 2:2T4R */
#define SENSOR_NUM_OF_CHIRPS (16U) /**< Valid range of chirps in relation to the SAMPLES_PER_CHIRP, [1 - 16] */

/*1.6V-0.9V 23.8-24.15GHz*/
/*1.7V-0.8V 23.7-24.26GHz*/

#define SENSOR_RF_SWEEP_START_MHZ 23690 /*FMCW扫描起始频率*/
#define SENSOR_RF_SWEEP_STOP_MHZ  24560 /*FMCW扫描截止频率*/

// #define SENSOR_RF_SWEEP_START_MHZ                       23290   /*FMCW扫描起始频率*/
// #define SENSOR_RF_SWEEP_STOP_MHZ                        24160   /*FMCW扫描截止频率*/     23725   6.32

#define SENSOR_VT_SWEEP_START_MV  1857.0f /*! VT扫描起始电压*/ // 量化值   电压变大带宽变大1800
#define SENSOR_VT_SWEEP_STOP_MV   407.0f                       /*! VT扫描终止电压*/
#define SENSOR_VT_SWEEP_TIME_US   800.0f                       /*！VT扫描时长*/
#define SENSOR_ADC_SAMPLE_FREQ_HZ 250000.0f                    /*! ADC采样频率*/
#define SENSOR_DAC_SAMPLE_FREQ_HZ 1000000.0f                   /*! DAC采样频率*/

#define HW_BOARD_TYPE 1                                        /*1: RKB1243F_THREEZONE*/

#define HW_VERSION_MAJOR 1
#define HW_VERSION_MINOR 0

#define BOOTLOADER_VERSION_MAJOR 1
#define BOOTLOADER_VERSION_MINOR 0
#define APP_VERSION_MAJOR        3
#define APP_VERSION_MINOR        2

#define SENSOR_TH3 450 /*检测门限1 */ // 150
#define SENSOR_TH1 120 /*检测门限1 */ // 200
#define SENSOR_TH2 600                // 600                // 1200               // 600 /*检测门限1 */ // 150

// #define SENSOR_TH3 450 /*检测门限1 */         // 150
// #define SENSOR_TH1 800 /*检测门限1 */         // 200
// #define SENSOR_TH2 200 /*检测门限1 */         // 150

#define SENSOR_TH1_MOV 20                     // 35//40      /*检测门限1 */
#define SENSOR_TH2_MOV 20                     // 25//40      /*检测门限1 */

#define SENSOR_TH1_OCC 18                     // 20//20      /*检测门限1 */
#define SENSOR_TH2_OCC 20                     // 22//25      /*检测门限1 */

#define SENSOR_MID_ANGLE 20 /*中区检测角度 */ // 12

#define SENSOR_TX_GAIN_DEFAULT 15             // 12
#define SENSOR_RX_GAIN_DEFAULT 7              // 6

#define CoefMul  10000                        // 校准/修正因子
#define CoefPlus 0

#define SENSOR_DAC_BUFFER_SIZE_MAX         1024
#define SENSOR_SAMPLE_SIZE_MAX             256U
#define SENSOR_ADC_BUFFER_SIZE_PER_CHANNEL 128U

#define SENSOR_ADC_DAC_BUFFER_PROTECT_SIZE        16U /*! adc/dac数组保护大小，防止数组越界*/
#define SENSOR_WAIT_TIME_BEFORE_SAMPLE_MS_DEFAULT 60U /*! 阶跃响应时间, 此时间段内数据丢弃*/

#define ADC_DATA_LENGTH  64 * (SENSOR_ADC_BUFFER_SIZE_PER_CHANNEL / 64) /*64整数倍*/
#define FFT_SIZE         256                                            /*！FFT 大小*/
#define FFT_APPLY_WINDOW 0                                              /*！加窗*/

#define SENSOR_SWEEP_BANDWIDTH_MHz 730.0f                               /*! 扫描带宽，与VT电压相关*/
#define SENSOR_VALID_BANDWIDTH_MHz                                                                                     \
    SENSOR_SWEEP_BANDWIDTH_MHz *ADC_DATA_LENGTH / SENSOR_ADC_BUFFER_SIZE_PER_CHANNEL /*!                               \
                                                                                        计算用扫描带宽，与VT电压相关*/
#define SENSOR_DISTANCE_PER_BIN                                                                                        \
    300.0f * SENSOR_VT_SWEEP_TIME_US *SENSOR_ADC_SAMPLE_FREQ_HZ / ADC_DATA_LENGTH /                                    \
        (0.2 * SENSOR_VALID_BANDWIDTH_MHz) // 单位：dm         /*! c*Ns/2b/Nfft */

#define SENSROR_DETECTION_RANGE_MIN 0.2f
#define SENSROR_DETECTION_RANGE_MAX 15.0f

#define SENSOR_GPIO_DELAY_TIME_MS_DEFAULT 5000U /*GPIO输出延时时间*/
#define SENSOR_GPIO_LOCK_TIME_MS_DEFAULT  1000U /*GPIO封锁时间，当输出后等待一段时间，此时间需保证无触发*/

/******************************************************************************
 * Global variable declarations ('extern', definition in C source)
 ******************************************************************************/

 /******************************************************************************
  * Global function prototypes (definition in C source)
  ******************************************************************************/

#ifdef __cplusplus
}
#endif
#endif //__SENSOR_CONFIG_H__
/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
