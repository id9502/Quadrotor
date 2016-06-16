/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
//Author: ¾£Ã÷Ðù
//Source code: https://github.com/id9502/Quadrotor

#ifndef __MOTO_H__
#define __MOTO_H__
#include "stm32f10x_conf.h"
#include "sys.h"

//
//   M1   M2
//      ^
//
//   M3   M4
//
#define MOTO_ERROR_1			0x01
#define MOTO_ERROR_UNDERFLOW_1	0x01
#define MOTO_ERROR_OVERFLOW_1	0x11
#define MOTO_ERROR_2			0x02
#define MOTO_ERROR_UNDERFLOW_2	0x02
#define MOTO_ERROR_OVERFLOW_2	0x22
#define MOTO_ERROR_3			0x04
#define MOTO_ERROR_UNDERFLOW_3	0x04
#define MOTO_ERROR_OVERFLOW_3	0x44
#define MOTO_ERROR_4			0x08
#define MOTO_ERROR_UNDERFLOW_4	0x08
#define MOTO_ERROR_OVERFLOW_4	0x88

#define __MOTO_SOFT_DEC			50

typedef uint8_t MOTO_ERROR_CODE;

MOTO_ERROR_CODE motoBeep(uint32_t freq, uint8_t delay_x100ms, uint32_t times);
MOTO_ERROR_CODE motoSet(int16_t MOTO1,int16_t MOTO2,int16_t MOTO3,int16_t MOTO4);
MOTO_ERROR_CODE motoAdjust(int16_t MOTO1_diff,int16_t MOTO2_diff,int16_t MOTO3_diff,int16_t MOTO4_diff);
void motoSoftOFF(void);
void MOTO_Setup(void);

#endif //__MOTO_H__
