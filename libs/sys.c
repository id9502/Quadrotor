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

#include "sys.h"

volatile uint32_t __us_counter = 0;

//#define __SYS_US_COUNTER	TIM3->CNT
#define __SYS_US_COUNTER	__us_counter

void SysTick_Handler(void)
{
	__SYS_US_COUNTER++;
}
void delay_us(uint32_t us)
{
	uint32_t start = __SYS_US_COUNTER + ((us-1)/SYSTICK_INTERVAL)+1;
//	while(__SYS_US_COUNTER > start);
	while(__SYS_US_COUNTER < start);
}
void delay_ms(uint32_t ms)
{
	delay_us(1000*ms);
}
void Sys_Setup(void)
{
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
	SysTick_Config(SystemCoreClock/8000000*SYSTICK_INTERVAL-1);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); // SYSTICK_INTERVAL(us)
	__us_counter = 0;
	NVIC_SetPriority(SysTick_IRQn,0x00);
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//		/* Time base configuration */
//	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;						// 计数上限
//	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000*SYSTICK_INTERVAL-1;// 时钟分频
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		// 向上计数
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//	TIM_ARRPreloadConfig(TIM3, ENABLE);
//	TIM_Cmd(TIM3,ENABLE);
}

inline uint32_t getElapseTime_us(uint32_t *last)
{
	uint32_t tmp = __SYS_US_COUNTER - *last;
	*last = __SYS_US_COUNTER;
	return tmp*SYSTICK_INTERVAL;
}

inline void __fixLoopPeriod(uint32_t us, uint32_t *counter)
{
	if(us != 0){
		uint32_t end = *counter + ((us-1)/SYSTICK_INTERVAL)+1;
//		while(__SYS_US_COUNTER > end);
		if(__SYS_US_COUNTER >= end-2) LED_1_ON;
		while(__SYS_US_COUNTER < end);
	}
	*counter = __SYS_US_COUNTER;
}
