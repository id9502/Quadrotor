//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
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

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f10x_conf.h"
#include "diag/Trace.h"
#include "libs/RemoteCTL.h"
#include "libs/sys.h"
#include "libs/leds.h"
#ifdef Quadrotor
#include "libs/moto.h"
#include "libs/attitude.h"
#include "libs/MPU6050.h"
#include "libs/ADC_Lib.h"
#else
#include "libs/USART_Lib.h"
#endif

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 empty sample (trace via STDOUT).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the STDOUT output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#ifdef Quadrotor
void PID_Angular_Velocity(float_vect3 W_target, float F);
float_vect3 PD_Angular(float_vect3 T_target);
float PID_Force(float F_target);

void PID_Angular_Velocity(float_vect3 W_target, float F)
{
	static float_vect3 lastE = {0.0f,0.0f,0.0f};
	static float_vect3 I = {0.0f,0.0f,0.0f};
	float_vect3 E,D,u;

	E = vsub(W_target, GYRO_v3f);
	I = vadd(I, vmultn(E,0.1f));
	D = vsub(E, lastE);

	if(I.x > 100.0f) I.x = 100.0f;
	else if(I.x < -100.0f) I.x = -100.0f;
	if(I.y > 100.0f) I.y = 100.0f;
	else if(I.y < -100.0f) I.y = -100.0f;
	if(I.z > 100.0f) I.z = 100.0f;
	else if(I.z < -100.0f) I.z = -100.0f;

	lastE = E;

	u.x = (100.0*D.x + 100.0*E.x + I.x);
	u.y = (100.0*D.y + 100.0*E.y + I.y);
	u.z = (100.0*D.z + 100.0*E.z + I.z);

	int16_t u1,u2,u3,u4;
	u1 = 0.5*u.y + 0.25*u.z + 0.25*F;
	u2 = 0.5*u.x - 0.25*u.z + 0.25*F;
	u3 = -0.5*u.y + 0.25*u.z + 0.25*F;
	u4 = -0.5*u.x - 0.25*u.z + 0.25*F;

	//if(motoAdjust(u1,u4,u2,u3)) LED_0_ON;
	//else LED_0_OFF;
}
float_vect3 PD_Angular(float_vect3 T_target)
{
	static float_vect3 lastE = {0.0f,0.0f,0.0f};
	float_vect3 T_current,E,D,u;
	float_vect3 crossresult = vdivn(vcross(ACC_v3f, attitude_gravity),vabs(attitude_gravity)*vabs(ACC_v3f));
	T_current = vmultn(crossresult, asin(vabs(crossresult))/vabs(crossresult));

	E = vsub(T_target,T_current);
	D = vsub(E,lastE);
	lastE = E;

	u = vadd(vmultn(E,5.0f), vmultn(D,1.0f));

	return u;
}

float PID_Force(float F_target)
{
	static float I = 0.0f;
	static float lastE = 0.0f;
	float E,D,F_current,u;

	F_current = attitude_acceleration.z;//vabs(ACC_v3f) - vabs(attitude_gravity);

	E = F_target - F_current;
	D = E - lastE;
	lastE = E;
	I = I + 50.0f*E;

	if(I>200.0f) I = 200.0f;
	else if(I<-200.0f) I = -200.0f;

	u = 100.0f*E + I + 10.0f*D;

	return u;
}

#endif
int main()
{
	Sys_Setup();
	LED_Setup();
#ifdef Quadrotor
	ADC_Setup();
	if(NRF24L01_Setup(MODEL_PRIM_TX,100)) while(1);
	MOTO_Setup();
	motoBeep(440,3,3);
	MPU6050_Setup();
	MPU6050_SelfAdjust();
	attitudeAdjust();

	float_vect3	T_target = {0.0f,0.0f,0.0f};
	float		F_target = 1.0f;					//支持力，正方向指向天

	//motoSet(780,780,780,780);
	uint32_t cnt = 0;
	uint32_t tmr = 0;
	getElapseTime_us(&tmr);
	int loop_cnt = 0;
	int loop_on = 4500;
	__fixLoopPeriod(0,&cnt);
	float_vect3 W_target;
	float F_output = 0.0f;;
	while (1)
	{
		__fixLoopPeriod(2000,&cnt);		//0.002s
		MPU6050_Read();
		attitudeUpdate((float)getElapseTime_us(&tmr)/1e6f);
		loop_cnt++;
		loop_on--;
		if(loop_cnt == 500)				//1.0s, only first time
			F_target = -F_target;
		if(loop_cnt == 1000)
			F_target = 0;
		if((loop_cnt % 5) == 0)			//0.01s
		{
			RemoteControl_Loop();
		}
		if((loop_cnt % 10) == 0)		//0.02s
		{
			W_target = PD_Angular(T_target);
		}
		if((loop_cnt % 50) == 0)		//0.1s
		{
			F_output = PID_Force(F_target);
		}
		if((loop_cnt % 500) == 0)
		{

		}
		PID_Angular_Velocity(W_target,F_output);
	}
	motoSet(0,0,0,0);
	while(1);
//
//	while(1){
//		prepareTxData();
//		uint8_t tmp = NRF_SendPacket(ACK_REQUEST);
//		if(tmp & TX_DS) LED_0_ON;
//		else LED_0_OFF;
//		NRF_RX_BUF[30] = 0x00;
//		if(NRF_ReadPacket() != 0xFF)
//		{
//			if(NRF_RX_BUF[30] == 0x05) LED_1_ON;
//		}else LED_1_OFF;
//		__fixLoopPeriod(20000,&cnter);
//	}
#else
	USART1_Setup();
	if(NRF24L01_Setup(MODEL_PRIM_RX,100)) while(1);
	while(1){
		if(RemoteControl_Loop() != 0xFF) LED_1_ON;
		else LED_1_OFF;
	}
//	int i= 0;
//	prepareTxData();
//	while(1){
//		NRF_SendPacket(ACK_REQUEST);
//		i++;
//		if(i==250){
//			//if(NRF_SPI_CheckConnection()) LED_0_ON;
//		}else if(i==500){
//			LED_0_OFF;
//		}else if(i>500)i=0;
//		if(NRF_ReadPacket() != 0xFF){
//			if(NRF_RX_BUF[30] == 0x05) LED_1_ON;
//		}
//		else LED_1_OFF;
//		__fixLoopPeriod(2000,&cnter);
//	}
#endif
}

#pragma GCC diagnostic pop
