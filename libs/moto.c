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

#include "moto.h"

#define MOTO_PWM_MAX	1024
#define MOTO_PWM_MIN	0

#define MOTO1_PWM		(TIM2->CCR1)
#define MOTO2_PWM		(TIM2->CCR2)
#define MOTO3_PWM		(TIM2->CCR3)
#define MOTO4_PWM		(TIM2->CCR4)

uint8_t motoOn;
uint8_t updateSpeed;

uint16_t __moto_cnt1,__moto_cnt2,__moto_cnt3,__moto_cnt4;

MOTO_ERROR_CODE motoBeep(uint32_t freq, uint8_t delay_x100ms, uint32_t times)
{
	if(freq <= ((SystemCoreClock >> 16)/MOTO_PWM_MAX) || freq >= 20000) return 0xFF;
	motoOn = 1;
	uint16_t freqdiv = SystemCoreClock/MOTO_PWM_MAX/freq;
	TIM_Cmd(TIM2, DISABLE);
	TIM_PrescalerConfig(TIM2,freqdiv-1,TIM_PSCReloadMode_Immediate);
	TIM2->CCR1 = MOTO_PWM_MAX/200;
	TIM2->CCR2 = MOTO_PWM_MAX/200;
	TIM2->CCR3 = MOTO_PWM_MAX/200;
	TIM2->CCR4 = MOTO_PWM_MAX/200;

	for(;times>0;times--){
		TIM_Cmd(TIM2,ENABLE);
		delay_ms(delay_x100ms*100);
		TIM_Cmd(TIM2,DISABLE);
		delay_ms(delay_x100ms*100);
	}

	TIM_PrescalerConfig(TIM2,0,TIM_PSCReloadMode_Immediate);
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;

	TIM_Cmd(TIM2,ENABLE);
	motoOn = 0;
	return 0;
}

MOTO_ERROR_CODE motoSet(int16_t MOTO1,int16_t MOTO2,int16_t MOTO3,int16_t MOTO4)
{
	MOTO_ERROR_CODE error_code = 0;
	/****** MOTO 1 *******/
	if(MOTO1>MOTO_PWM_MAX){
		MOTO1 = MOTO_PWM_MAX;
		error_code |= MOTO_ERROR_OVERFLOW_1;
	}else if(MOTO1<MOTO_PWM_MIN){
		MOTO1 = MOTO_PWM_MIN;
		error_code |= MOTO_ERROR_UNDERFLOW_1;
	}
	/****** MOTO 2 *******/
	if(MOTO2>MOTO_PWM_MAX){
		MOTO2 = MOTO_PWM_MAX;
		error_code |= MOTO_ERROR_OVERFLOW_2;
	}else if(MOTO2<MOTO_PWM_MIN){
		MOTO2 = MOTO_PWM_MIN;
		error_code |= MOTO_ERROR_UNDERFLOW_2;
	}
	/****** MOTO 3 *******/
	if(MOTO3>MOTO_PWM_MAX){
		MOTO3 = MOTO_PWM_MAX;
		error_code |= MOTO_ERROR_OVERFLOW_3;
	}else if(MOTO3<MOTO_PWM_MIN){
		MOTO3 = MOTO_PWM_MIN;
		error_code |= MOTO_ERROR_UNDERFLOW_3;
	}
	/****** MOTO 4 *******/
	if(MOTO4>MOTO_PWM_MAX){
		MOTO4 = MOTO_PWM_MAX;
		error_code |= MOTO_ERROR_OVERFLOW_4;
	}else if(MOTO4<MOTO_PWM_MIN){
		MOTO4 = MOTO_PWM_MIN;
		error_code |= MOTO_ERROR_UNDERFLOW_4;
	}

	if(error_code != 0) return error_code;
	if(MOTO1 == 0 && MOTO2 == 0 && MOTO3 == 0 && MOTO4 == 0) motoOn = 0;
	else motoOn = 1;

	if(updateSpeed){
		__moto_cnt1 = MOTO1;
		__moto_cnt2 = MOTO2;
		__moto_cnt3 = MOTO3;
		__moto_cnt4 = MOTO4;
	}

	MOTO1_PWM = MOTO1;
	MOTO2_PWM = MOTO2;
	MOTO3_PWM = MOTO3;
	MOTO4_PWM = MOTO4;

	return error_code;
}

MOTO_ERROR_CODE motoAdjust(int16_t MOTO1_diff,int16_t MOTO2_diff,int16_t MOTO3_diff,int16_t MOTO4_diff)
{
	updateSpeed = 0;
	return motoSet(__moto_cnt1+MOTO1_diff,__moto_cnt2+MOTO2_diff,
			__moto_cnt3+MOTO3_diff,__moto_cnt4+MOTO4_diff);
	updateSpeed = 1;
}

inline void Tim2_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = MOTO_PWM_MAX;				// 计数上限
	TIM_TimeBaseStructure.TIM_Prescaler = 0;						// PWM时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		// 向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	
	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;								// 初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_Cmd(TIM2, ENABLE);
}
void motoSoftOFF(void)
{
	if(!motoOn) return;
	while(MOTO1_PWM!=0||MOTO2_PWM!=0||MOTO3_PWM!=0||MOTO4_PWM!=0){
		if(MOTO1_PWM!=0){
			if(MOTO1_PWM>=__MOTO_SOFT_DEC)
				MOTO1_PWM -= __MOTO_SOFT_DEC;
			else
				MOTO1_PWM = 0;
		}
		if(MOTO2_PWM!=0){
			if(MOTO2_PWM>=__MOTO_SOFT_DEC)
				MOTO2_PWM -= __MOTO_SOFT_DEC;
			else
				MOTO2_PWM = 0;
		}
		if(MOTO3_PWM!=0){
			if(MOTO3_PWM>=__MOTO_SOFT_DEC)
				MOTO3_PWM -= __MOTO_SOFT_DEC;
			else
				MOTO3_PWM = 0;
		}
		if(MOTO4_PWM!=0){
			if(MOTO4_PWM>=__MOTO_SOFT_DEC)
				MOTO4_PWM -= __MOTO_SOFT_DEC;
			else
				MOTO4_PWM = 0;
		}
		delay_ms(100);
	}
	motoOn = 0;
}
void MOTO_Setup(void)
{
	motoOn = 0;

	GPIO_InitTypeDef GPIO_InitStructure;
	// 使能电机用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
	// 设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	Tim2_init();

	updateSpeed = 1;
	motoSet(0,0,0,0);
}

