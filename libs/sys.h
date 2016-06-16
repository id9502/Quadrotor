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

#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f10x_conf.h"
#include "leds.h"

// should be at least 1us
#define SYSTICK_INTERVAL	20

#if defined(__cplusplus)
extern "C"
{
#endif

void Sys_Setup();
/*
 * to measure the elapse time, write code as this:
 * uint32_t t = 0;
 * getElapseTime_us(&t);
 * <--your procedure here-->
 * uint32_t elapse_time = getElapseTime_us(&t);
 */
uint32_t getElapseTime_us(uint32_t *last);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void __fixLoopPeriod(uint32_t us, uint32_t *counter);

#if defined(__cplusplus)
};
#endif

typedef
struct{
	float x;
	float y;
	float z;
} float_vect3;

typedef
struct{
	float r;
	float i;
	float j;
	float k;
} quater;
#endif // __SYS_H__
