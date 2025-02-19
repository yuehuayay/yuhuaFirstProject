#ifndef __WIN_API_H__
#define __WIN_API_H__

#include "stdint.h"
//#include <vector>
//using namespace std;
//#ifdef WIN_API_EXPORTS
//#define DLL_WIN_API _declspec (dllexport)
//#else
//#define DLL_WIN_API _declspec (dllimport)
//#endif

#ifdef __cplusplus
extern "C" {
#endif

#define restrict
#define MAX_TOKEN (1024)	/** 一般性记号长度 */
	typedef struct Complexfloat_t
	{
		float image;
		float real;
	}Complexfloat;

	typedef struct sdkdll_para_t_
	{
		uint16_t numRange;      //440
		uint16_t numDoppler;    //64
		uint16_t numVirtualAnt; //16
		uint16_t numRangecube;  //64*16
		uint16_t numRdm;        //440*64
		float fftWin_Q; //FFT加窗QFORMAT系数
		float scale_cfar_os;
		float scale_cfar_ca;
	} sdkdll_para_t;

	typedef struct vBits24_t_
	{
		int32_t value : 24;
		//int32_t reserved : 8;
	} vBits24_t;

	typedef struct xTcmplx32ReIm_t_
	{
		vBits24_t real; /*!< @brief real part */
		vBits24_t image; /*!< @brief imaginary part */
	} xTcmplx32ReIm_t;


	//typedef struct cmplx16ImRe_t_
	//{
	//	int16_t imag; /*!< @brief imaginary part */
	//	int16_t real; /*!< @brief real part */
	//} cmplx16ImRe_t;


	//DLL_WIN_API void win_2d_fft_process(
	//	cmplx16ImRe_t* ptr_x,
	//	cmplx32ImRe_t* Fft_2dOut,
	//	uint32_t* detMatrix,
	//	int32_t* window2D_tx0,
	//	int npoints,
	//	int scale);
	int win_fft(int N, Complexfloat f[], short scaleFactor, double gat);

#ifdef __cplusplus
}
#endif

#endif