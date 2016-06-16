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

#ifndef __LEDS_H__
#define __LEDS_H__
#include "stm32f10x_conf.h"
#include "BDF.h"

#if USE_LED_CFG == 1
#define LED_0_PORT				GPIOC
#define LED_0_PIN				GPIO_Pin_13
#define LED_1_PORT				GPIOB
#define LED_1_PIN				GPIO_Pin_3

#define LED_0_ON				(LED_0_PORT->BSRR = LED_0_PIN)
#define LED_0_OFF				(LED_0_PORT->BSRR = (LED_0_PIN<<16))
#define LED_1_ON				(LED_1_PORT->BSRR = (LED_1_PIN<<16))
#define LED_1_OFF				(LED_1_PORT->BSRR = LED_1_PIN)
#elif USE_LED_CFG == 2
#define LED_0_PORT				GPIOB
#define LED_0_PIN				GPIO_Pin_11
#define LED_1_PORT				GPIOB
#define LED_1_PIN				GPIO_Pin_12

#define LED_0_ON				(LED_0_PORT->BSRR = (LED_0_PIN<<16))
#define LED_0_OFF				(LED_0_PORT->BSRR = LED_0_PIN)
#define LED_1_ON				(LED_1_PORT->BSRR = (LED_1_PIN<<16))
#define LED_1_OFF				(LED_1_PORT->BSRR = LED_1_PIN)
#endif


#define LED_OFF					0x00
#define LED_ON					0x01
#define LED_TWINKLE_LF			0x02
#define LED_TWINKLE_MF			0x03
#define LED_TWINKLE_HF			0x04

void LED_Setup(void);
void led0Ctl(uint8_t code);
void led1Ctl(uint8_t code);

#endif //__LEDS_H__
