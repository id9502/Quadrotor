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

#include "IIC_Lib.h"

#ifdef __IIC_LIB_USE_SOFTWARE_IIC__
void I2C_Setup(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	/* Configure IO connected to IIC*********************/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitStructure.I2C_OwnAddress1 = 0xAA;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

uint8_t I2C_WriteData(uint8_t dev_address, uint8_t address, uint8_t *buffer, uint16_t data_len)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));				//等待I2C
	I2C_GenerateSTART(I2C1, ENABLE);									//产生起始条件
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));		//等待ACK
	I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Transmitter);			//向设备发送设备地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//等待ACK
	I2C_SendData(I2C1, address & 0xFF);									//寄存器地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//等待ACK
	for(int i = 0 ; i < data_len ; i++){
		I2C_SendData(I2C1, buffer[i]);								//发送数据
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//发送完成
	}
	I2C_GenerateSTOP(I2C1, ENABLE);									//产生结束信号
	/*************** Wait for write delay ***********************/
	//for(volatile int i = 0 ; i < 0xfffff ; i++);
	return 0;
}
uint8_t I2C_ReadData(uint8_t dev_address, uint8_t address, uint8_t *buffer, uint16_t data_len)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));				//等待I2C
	I2C_GenerateSTART(I2C1, ENABLE);							//产生起始信号
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));	//EV5
	I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Transmitter);	//发送地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//EV6
	I2C_SendData(I2C1, address & 0xFF);								//发送读得地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));//EV8
	I2C_GenerateSTART(I2C1, ENABLE);							//重新发送
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));	//EV5
	I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Receiver);		//发送读地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//EV6
	int i = 0;
	for(;i < data_len-1 ; i++){
		while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
		buffer[i] = I2C_ReceiveData(I2C1);
	}
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);								//关闭应答和停止条件产生
	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));

	buffer[i] = I2C_ReceiveData(I2C1);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0;
}
uint8_t I2C_isBusy(void){return 0;}

#endif
#ifdef __IIC_LIB_USE_INTERRUPT_IIC__
uint16_t i2c_data_len;
uint8_t *i2c_data_buff;
uint8_t i2c_dev_addr;
uint8_t i2c_reg_addr;
uint8_t i2c_dir;

uint8_t I2C_BUSY;

void I2C2_EV_IRQHandler(void)
{
	static uint8_t state = 0;
	static uint16_t data_pos = 0;
	switch(state){
	case 0:	//Start Bit Sent, sending device address now
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)){
			state = 1;
			I2C_Send7bitAddress(I2C2, i2c_dev_addr, I2C_Direction_Transmitter);
		}
		break;
	case 1: //Device Address sent, sending register address now
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
			state = (i2c_dir==I2C_Direction_Transmitter ? 2 : 4);
			I2C_SendData(I2C2,i2c_reg_addr);
			data_pos = 0;
		}
		break;
	case 2: //Transmitter mode, sending data
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING)){
			if(i2c_data_len <= data_pos + 1) state = 3;
			I2C_SendData(I2C2, i2c_data_buff[data_pos]);
			data_pos++;
		}
		break;
	case 3: //Transmitter mode, sending stop
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
			state = 0;
			I2C_GenerateSTOP(I2C2,ENABLE);
			I2C_BUSY = 0;
		}
		break;
	case 4: //Receiver mode, re-generating start bit
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
			state = 5;
			I2C_GenerateSTART(I2C2,ENABLE);
		}
		break;
	case 5: //Receiver mode, Start bit sent, sending address again
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)){
			state = 6;
			I2C_Send7bitAddress(I2C2, i2c_dev_addr, I2C_Direction_Receiver);
		}
		break;
	case 6: //Device Address sent
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
			if(i2c_data_len == 1){
				I2C_AcknowledgeConfig(I2C2, DISABLE);
				I2C_GenerateSTOP(I2C2,ENABLE);
				state = 8;
			}else state = 7;
		}
		break;
	case 7: //One data read
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)){
			if(i2c_data_len <= data_pos + 2){
				I2C_AcknowledgeConfig(I2C2, DISABLE);
				I2C_GenerateSTOP(I2C2,ENABLE);
				state = 8;
			}
			i2c_data_buff[data_pos] = I2C_ReceiveData(I2C2);
			data_pos++;
		}
		break;
	case 8: //The last data
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)){
			i2c_data_buff[data_pos] = I2C_ReceiveData(I2C2);
			data_pos++;
			I2C_AcknowledgeConfig(I2C2, ENABLE);
			state = 0;
			I2C_BUSY = 0;
		}
		break;
	default:
		__BKPT(100);
		state = 0;
		I2C_Cmd(I2C2,DISABLE);
		I2C_Cmd(I2C2,ENABLE);
		I2C_BUSY = 0;
		break;
	}
}

void I2C_Setup()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	/* Configure IO connected to IIC*********************/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitStructure.I2C_OwnAddress1 = 0xAA;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 350000;
	I2C_Init(I2C2, &I2C_InitStructure);
	/******* I2C Variables setup *************/
	i2c_data_len = 0;
	i2c_dev_addr = 0;
	i2c_reg_addr = 0;
	i2c_dir = 0;
	I2C_BUSY = 0;
	/******* I2C Interrupts setup ************/
	I2C_ITConfig(I2C2,I2C_IT_EVT|I2C_IT_BUF,ENABLE);
	NVIC_SetPriority(I2C2_EV_IRQn,0x00);
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	I2C_Cmd(I2C2, ENABLE);
}
uint8_t I2C_WriteData(uint8_t dev_address, uint8_t address, uint8_t *buffer, uint16_t data_len){
	if(I2C_BUSY) return 1;
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) return 2;
	I2C_BUSY = 1;
	i2c_data_len = data_len;
	i2c_data_buff = buffer;
	i2c_dev_addr = dev_address;
	i2c_reg_addr = address;
	i2c_dir = I2C_Direction_Transmitter;
	I2C_GenerateSTART(I2C2,ENABLE);
	return 0;
}
uint8_t I2C_ReadData(uint8_t dev_address, uint8_t address, uint8_t *buffer, uint16_t data_len){
	if(I2C_BUSY) return 1;
	if(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) return 2;
	I2C_BUSY = 1;
	i2c_data_len = data_len;
	i2c_data_buff = buffer;
	i2c_dev_addr = dev_address;
	i2c_reg_addr = address;
	i2c_dir = I2C_Direction_Receiver;
	I2C_GenerateSTART(I2C2,ENABLE);
	return 0;
}
inline uint8_t I2C_isBusy(void){return I2C_BUSY;}
#endif
