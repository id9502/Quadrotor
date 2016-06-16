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

#include "RemoteCTL.h"

#ifdef Quadrotor
#include "MPU6050.h"
#include "ADC_Lib.h"
#include "leds.h"
extern int16_t __average_buf_gyro__[MPU6050_GYRO_BUFFER_LEN][3];
extern int16_t __average_buf_acc__[MPU6050_ACC_BUFFER_LEN][3];
extern uint32_t __us_counter;
#else
#include "USART_Lib.h"
#include "stdio.h"
uint8_t __led_0_ctl,__led_1_ctl;
#endif

void prepareTxData(uint8_t CMD)
{
	switch(CMD){
	case CMD_TEST_CONNECTION:
	/*
	 * T->R then R->T
	 * 0:CMD_TEST_CONNECTION
	 * 1:0x01
	 * 2:0x02
	 * ......
	 * 31:0x1F
	 */
		NRF_TX_BUF[0] = CMD_TEST_CONNECTION;
		for(int i = 1 ; i < TX_PLOAD_WIDTH_MAX ; i++) NRF_TX_BUF[i] = i;
		NRF_TX_BUF[32] = 32;
		break;
	case CMD_NOP:
		NRF_TX_BUF[0] = CMD_NOP;
		NRF_TX_BUF[32] = 1;
		break;
#ifdef Quadrotor
	case CMD_DATA_EXCHANGE:{
	/*
	 * T->R only
	 * 0:CMD_DATA_EXCHANGE
	 * 1:2:3:4:32bit timestamp
	 * 5:6:16bit ACCX value
	 * 7:8:16bit ACCY value
	 * ...
	 * 15:16:16bit GYROZ value
	 * 17:ADC value
	 */
		int32_t data[3] = {0,0,0};
		for(int i = 0 ; i < MPU6050_ACC_BUFFER_LEN ; i++){
			data[0] += __average_buf_acc__[i][0];
			data[1] += __average_buf_acc__[i][1];
			data[2] += __average_buf_acc__[i][2];
		}
		data[0] /= MPU6050_ACC_BUFFER_LEN;
		data[1] /= MPU6050_ACC_BUFFER_LEN;
		data[2] /= MPU6050_ACC_BUFFER_LEN;

		NRF_TX_BUF[0] = CMD_DATA_EXCHANGE;
		uint32_t t = (__us_counter*SYSTICK_INTERVAL)/1000;
		NRF_TX_BUF[1] = (uint8_t)((t>>24));
		NRF_TX_BUF[2] = (uint8_t)((t>>16));
		NRF_TX_BUF[3] = (uint8_t)((t>>8));
		NRF_TX_BUF[4] = (uint8_t)((t>>0));

		NRF_TX_BUF[5] = (uint8_t)((data[0]>>8)&0xFF);
		NRF_TX_BUF[6] = (uint8_t)((data[0])&0xFF);
		NRF_TX_BUF[7] = (uint8_t)((data[1]>>8)&0xFF);
		NRF_TX_BUF[8] = (uint8_t)((data[1])&0xFF);
		NRF_TX_BUF[9] = (uint8_t)((data[2]>>8)&0xFF);
		NRF_TX_BUF[10] = (uint8_t)((data[2])&0xFF);
		for(int i = 0 ; i < MPU6050_GYRO_BUFFER_LEN ; i++){
			data[0] += __average_buf_gyro__[i][0];
			data[1] += __average_buf_gyro__[i][1];
			data[2] += __average_buf_gyro__[i][2];
		}
		data[0] /= MPU6050_GYRO_BUFFER_LEN;
		data[1] /= MPU6050_GYRO_BUFFER_LEN;
		data[2] /= MPU6050_GYRO_BUFFER_LEN;

		NRF_TX_BUF[11] = (uint8_t)((data[0]>>8)&0xFF);
		NRF_TX_BUF[12] = (uint8_t)((data[0])&0xFF);
		NRF_TX_BUF[13] = (uint8_t)((data[1]>>8)&0xFF);
		NRF_TX_BUF[14] = (uint8_t)((data[1])&0xFF);
		NRF_TX_BUF[15] = (uint8_t)((data[2]>>8)&0xFF);
		NRF_TX_BUF[16] = (uint8_t)((data[2])&0xFF);
		uint8_t adcv = (ADC_Read()>>4)&0xFF;
		NRF_TX_BUF[17] = adcv;
		NRF_TX_BUF[32] = 18;
	}
	break;
	case CMD_CONTROL:
		NRF_TX_BUF[0] = CMD_CONTROL;
		NRF_TX_BUF[32] = 1;
		break;
#else
	case CMD_DATA_EXCHANGE:
		NRF_TX_BUF[0] = CMD_DATA_EXCHANGE;
		NRF_TX_BUF[32] = 1;
		break;
	case CMD_CONTROL:
	/*
	 * R -> T only
	 * 0:CMD_CONTROL
	 * 1:bit 0:led0 status;bit 1:led1 status
	 */
		NRF_TX_BUF[0] = CMD_CONTROL;
		NRF_TX_BUF[1] = 0x00;
		if(__led_1_ctl)NRF_TX_BUF[1] |= 0x02;
		if(__led_0_ctl)NRF_TX_BUF[1] |= 0x01;
		NRF_TX_BUF[32] = 2;
		break;
#endif
	case CMD_CONTROL_ACK:
		NRF_TX_BUF[0] = CMD_CONTROL_ACK;
		NRF_TX_BUF[32] = 1;
		break;

	}
}
/*
 * 0xFF: check connection
 * 0x00: R:return NOP, T:send data
 * 0x01: R:return CTL, T:send data
 */
uint8_t status = 0xFF;
#ifdef Quadrotor
uint8_t RemoteControl_Loop()
{
	/////////////////////  TX  /////////////////////////
	switch(status){
	case 0xFF:
		prepareTxData(CMD_TEST_CONNECTION);
		NRF_SendPacket(ACK_REQUEST);
		break;
	case 0x00:
	case 0x01:
		prepareTxData(CMD_DATA_EXCHANGE);
		NRF_SendPacket(ACK_REQUEST);
		break;
	}
	/////////////////////  RX  /////////////////////////
	if(NRF_ReadPacket() == 0xFF) return status;
	switch(NRF_RX_BUF[0]){
	case CMD_TEST_CONNECTION:
	case CMD_NOP:
		status = 0x00;
		break;
	case CMD_CONTROL:
		status = 0x00;
		if(NRF_RX_BUF[1] & 0x01) LED_0_ON;
		else LED_0_OFF;
		if(NRF_RX_BUF[1] & 0x02) LED_1_ON;
		else LED_1_OFF;
		break;
	default:
		status = 0xFF;
		break;
	}
	return status;
}
#else
uint8_t send_str_on = 0;
void usart_send_str()
{
	char str[128];
	sprintf(str,"----------\r\nTime: %u\r\nACC:  (%f, %f, %f)\r\nGYRO: (%f, %f ,%f)\r\nVbat: %f\r\n",
			((uint32_t)NRF_RX_BUF[1]<<24)|((uint32_t)NRF_RX_BUF[2]<<16)|((uint32_t)NRF_RX_BUF[3]<<8)|((uint32_t)NRF_RX_BUF[4]<<0),
			(int16_t)(((uint16_t)NRF_RX_BUF[5]<<8)|((uint16_t)NRF_RX_BUF[6]<<0))*0.002392578125f,
			(int16_t)(((uint16_t)NRF_RX_BUF[7]<<8)|((uint16_t)NRF_RX_BUF[8]<<0))*0.002392578125f,
			(int16_t)(((uint16_t)NRF_RX_BUF[9]<<8)|((uint16_t)NRF_RX_BUF[10]<<0))*0.002392578125f,
			(int16_t)(((uint16_t)NRF_RX_BUF[11]<<8)|((uint16_t)NRF_RX_BUF[12]<<0))*0.000532632218017578125f,
			(int16_t)(((uint16_t)NRF_RX_BUF[13]<<8)|((uint16_t)NRF_RX_BUF[14]<<0))*0.000532632218017578125f,
			(int16_t)(((uint16_t)NRF_RX_BUF[15]<<8)|((uint16_t)NRF_RX_BUF[16]<<0))*0.000532632218017578125f,
			(int32_t)((uint32_t)NRF_RX_BUF[17])*0.02578125f);
	uint32_t len;
	for(len = 0 ; str[len] != '\0' ; len++);
	USART_Send((uint8_t*)str, len);
}

uint8_t RemoteControl_Loop()
{
	/////////////////////  TX  /////////////////////////
	switch(status){
	case 0xFF:
		prepareTxData(CMD_TEST_CONNECTION);
		NRF_SendPacket(ACK_REQUEST);
		break;
	case 0x00:
		prepareTxData(CMD_NOP);
		NRF_SendPacket(ACK_REQUEST);
		break;
	case 0x01:
		prepareTxData(CMD_CONTROL);
		NRF_SendPacket(ACK_REQUEST);
		break;
	}	/////////////////////  RX  /////////////////////////
	LED_0_OFF;
	while(NRF_ReadPacket() == 0xFF) delay_ms(2);
	LED_0_ON;
	switch(NRF_RX_BUF[0]){
	case CMD_TEST_CONNECTION:
	case CMD_DATA_EXCHANGE:
		status = 0x01;
		break;
	default:
		status = 0xFF;
		break;
	}
	if(send_str_on) usart_send_str();
	return status;
}

void USART_RX_CMD_Handler(uint8_t pc_cmd)
{
	switch(pc_cmd){
	case '0':{
			__led_0_ctl = !__led_0_ctl;
			uint8_t str = '0';
			USART_Send(&str, 1);
		}
		break;
	case '1':{
			__led_1_ctl = !__led_1_ctl;
			uint8_t str = '1';
			USART_Send(&str, 1);
		}
		break;
	case '2':
		send_str_on = !send_str_on;
		break;
	}
}
#endif
