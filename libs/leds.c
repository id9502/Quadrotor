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
//Author: 荆明轩
//Source code: https://github.com/id9502/Quadrotor

#include "leds.h"
#include "stm32f10x_conf.h"

void LED_Setup(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);	//关闭JTAG-DP,开启SW-DP，释放PB3，PB4
	DBGMCU->CR &= ~DBGMCU_CR_TRACE_IOEN;
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStruct.GPIO_Pin = LED_0_PIN;
	GPIO_Init(LED_0_PORT,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = LED_1_PIN;
	GPIO_Init(LED_1_PORT,&GPIO_InitStruct);
	LED_0_OFF;
	LED_1_OFF;
}
void led0Ctl(uint8_t code)
{
	if(code == LED_0_OFF) LED_0_OFF;
	else if(code == LED_0_ON) LED_0_ON;
	return;
}
void led1Ctl(uint8_t code)
{
	if(code == LED_OFF) LED_1_OFF;
	else if(code == LED_ON) LED_1_ON;
	return;
}
