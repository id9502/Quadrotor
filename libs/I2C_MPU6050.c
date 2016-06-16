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

#include "MPU6050.h"
#include <stdio.h>
#include <stdlib.h>

#define ACC_MAX_SCALE	8.0f
#define GYRO_MAX_SCALE	1000.0f
/***************************************************************/

float_vect3 ACC_v3f,GYRO_v3f;		//最新一次读取值
float MPU6050_TEMPERATURE;

int16_vect3 __MPU6050_GYRO_OFFSET = {0,0,0};	//零漂
int16_t __average_buf_gyro__[MPU6050_GYRO_BUFFER_LEN][3];
int16_t __average_buf_acc__[MPU6050_ACC_BUFFER_LEN][3];
int8_t __loop_cnt_gyro__,__loop_cnt_acc__;


void uint16tofloat(void)
{
	int32_t data[6] = {0,0,0,0,0,0};
	float tmpx,tmpy,tmpz;
	for(int i = 0 ; i < MPU6050_ACC_BUFFER_LEN ; i++){
		data[0] += __average_buf_acc__[i][0];
		data[1] += __average_buf_acc__[i][1];
		data[2] += __average_buf_acc__[i][2];
	}
	for(int i = 0 ; i < MPU6050_GYRO_BUFFER_LEN ; i++){
		data[3] += __average_buf_gyro__[i][0];
		data[4] += __average_buf_gyro__[i][1];
		data[5] += __average_buf_gyro__[i][2];
	}
	tmpx = ((float)data[0])*(ACC_MAX_SCALE/32768.0f/(float)MPU6050_ACC_BUFFER_LEN*9.8f);
	tmpy = ((float)data[1])*(ACC_MAX_SCALE/32768.0f/(float)MPU6050_ACC_BUFFER_LEN*9.8f);
	tmpz = ((float)data[2])*(ACC_MAX_SCALE/32768.0f/(float)MPU6050_ACC_BUFFER_LEN*9.8f);
	ACC_v3f.x = (-tmpx + tmpy)*0.7071067812;
	ACC_v3f.y = (tmpx + tmpy)*0.7071067812;
	ACC_v3f.z = -tmpz;
	MPU6050_TEMPERATURE = 0;//36.53f + ((float)data[3])/340.0f;
	tmpx = ((float)data[3])*(GYRO_MAX_SCALE/32768.0f/(float)MPU6050_GYRO_BUFFER_LEN)*0.01745329252f;
	tmpy = ((float)data[4])*(GYRO_MAX_SCALE/32768.0f/(float)MPU6050_GYRO_BUFFER_LEN)*0.01745329252f;
	tmpz = ((float)data[5])*(GYRO_MAX_SCALE/32768.0f/(float)MPU6050_GYRO_BUFFER_LEN)*0.01745329252f;
	GYRO_v3f.x = (-tmpx + tmpy)*0.7071067812;
	GYRO_v3f.y = (tmpx + tmpy)*0.7071067812;
	GYRO_v3f.z = -tmpz;
}

uint8_t MPU6050_SelfAdjust(void)
{
	uint8_t buffer[14];					//iic读取后存放数据
	int32_t avg_value[7];
	//////////////////////////////////////////////////////////////////
	///////////////////////  Normal output  //////////////////////////
	//////////////////////////////////////////////////////////////////
	avg_value[0] = avg_value[1] = avg_value[2] = avg_value[3]
				= avg_value[4] = avg_value[5] = avg_value[6] = 0;
	uint32_t t = 0;
	__fixLoopPeriod(0,&t);
	for(int i = 0 ; i < 1024 ; i ++){
		__fixLoopPeriod(2000,&t);
		I2C_ReadData(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, buffer, 14);
		while(I2C_isBusy());
		avg_value[0] += (int16_t)((((uint16_t)buffer[0])<<8) | ((uint16_t)buffer[1]));
		avg_value[1] += (int16_t)((((uint16_t)buffer[2])<<8) | ((uint16_t)buffer[3]));
		avg_value[2] += (int16_t)((((uint16_t)buffer[4])<<8) | ((uint16_t)buffer[5]));
		avg_value[4] += (int16_t)((((uint16_t)buffer[8])<<8) | ((uint16_t)buffer[9]));
		avg_value[5] += (int16_t)((((uint16_t)buffer[10])<<8) | ((uint16_t)buffer[11]));
		avg_value[6] += (int16_t)((((uint16_t)buffer[12])<<8) | ((uint16_t)buffer[13]));
	}
	float tmpx,tmpy,tmpz;
	tmpx = ((float)avg_value[0])*(ACC_MAX_SCALE/32768.0f/1024.0f*9.8f);
	tmpy = ((float)avg_value[1])*(ACC_MAX_SCALE/32768.0f/1024.0f*9.8f);
	tmpz = ((float)avg_value[2])*(ACC_MAX_SCALE/32768.0f/1024.0f*9.8f);
	attitude_gravity.x = (-tmpx + tmpy)*0.7071067812;
	attitude_gravity.y = (tmpx + tmpy)*0.7071067812;
	attitude_gravity.z = -tmpz;

	__MPU6050_GYRO_OFFSET.x = avg_value[4] >> 10;
	__MPU6050_GYRO_OFFSET.y = avg_value[5] >> 10;
	__MPU6050_GYRO_OFFSET.z = avg_value[6] >> 10;
	///////////////////////////////////////////////////////////////////////////////////////
	for(int i = 0 ; i < MPU6050_ACC_BUFFER_LEN || i < MPU6050_GYRO_BUFFER_LEN ; i++){
		__fixLoopPeriod(2000,&t);
		MPU6050_Read();
	}
	return 0;
}

uint8_t MPU6050_Setup(void)
{
	I2C_Setup();

	uint8_t data[2] = {0,0};

	data[0] = MPU6050_CLOCK_PLL_ZGYRO;
	if(I2C_WriteData(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, data, 1))//电源管理，典型值：0x00(正常启用)
		return 1;
	delay_ms(2);
	data[0] = 0x07;
	if(I2C_WriteData(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, data, 1))//陀螺仪采样率，典型值：0x04，五分频
		return 2;
	delay_ms(2);
	data[0] = MPU6050_CONFIG_DLPF_CFG_95HZ;
	if(I2C_WriteData(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, data, 1))//低通滤波频率，典型值：0x02(截止频率100Hz)
		return 3;
	delay_ms(2);
	data[0] = MPU6050_GYRO_CONFIG_FS_SEL_1000;
	data[1] = MPU6050_ACCEL_CONFIG_HPF_5HZ|MPU6050_ACCEL_CONFIG_FS_SEL_8G;
	I2C_WriteData(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, data, 2);//加速计自检、测量范围及高通滤波频率，典型值：0x10(不自检，8G，5Hz)
	delay_ms(10);
	__loop_cnt_acc__ = 0;
	__loop_cnt_gyro__ = 0;
	return 0;
}
void MPU6050_Read(void)
{
	__loop_cnt_gyro__ = (__loop_cnt_gyro__ < MPU6050_GYRO_BUFFER_LEN - 1 ? (__loop_cnt_gyro__ + 1) : 0);
	__loop_cnt_acc__ = (__loop_cnt_acc__ < MPU6050_ACC_BUFFER_LEN - 1 ? (__loop_cnt_acc__ + 1) : 0);
	uint8_t buffer[14];
	I2C_ReadData(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, buffer, 14);
	while(I2C_isBusy());
	__average_buf_acc__[__loop_cnt_acc__][0] = (int16_t)((((uint16_t)buffer[0])<<8) | ((uint16_t)buffer[1]));// - __MPU6050_ACC_OFFSET.x;
	__average_buf_acc__[__loop_cnt_acc__][1] = (int16_t)((((uint16_t)buffer[2])<<8) | ((uint16_t)buffer[3]));// - __MPU6050_ACC_OFFSET.y;
	__average_buf_acc__[__loop_cnt_acc__][2] = (int16_t)((((uint16_t)buffer[4])<<8) | ((uint16_t)buffer[5]));// - __MPU6050_ACC_OFFSET.z;

	__average_buf_gyro__[__loop_cnt_gyro__][0] = (int16_t)((((uint16_t)buffer[8])<<8) | ((uint16_t)buffer[9])) - __MPU6050_GYRO_OFFSET.x;
	__average_buf_gyro__[__loop_cnt_gyro__][1] = (int16_t)((((uint16_t)buffer[10])<<8) | ((uint16_t)buffer[11])) - __MPU6050_GYRO_OFFSET.y;
	__average_buf_gyro__[__loop_cnt_gyro__][2] = (int16_t)((((uint16_t)buffer[12])<<8) | ((uint16_t)buffer[13])) - __MPU6050_GYRO_OFFSET.z;
	uint16tofloat();
}
