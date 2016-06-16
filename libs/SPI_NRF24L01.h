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
//Author: ������
//Source code: https://github.com/id9502/Quadrotor

//ע�⣡ʹ����DMA��SPI���䲻�����ж���ʹ�ã����е��ñ���ŵ�mainִ�м���
#ifndef __SPI_NRF24L01_H__
#define __SPI_NRF24L01_H__

#include "stm32f10x_conf.h"

#include "SPI_Lib.h"
#include "sys.h"
#include "../diag/Trace.h"

//*********************NRF24L01P�Ĵ���ָ��*******************************
#define NRF_R				0x00  // ���Ĵ���ָ�+�Ĵ�����ַ
#define NRF_W				0x20  // д�Ĵ���ָ�+�Ĵ�����ַ
#define R_RX_PLOAD			0x61  // ��ȡ��������ָ�+1~32byte д������
#define W_TX_PLOAD			0xA0  // д��������ָ�+1~32byte ��������
#define FLUSH_TX			0xE1  // ��ϴ���� FIFOָ��
#define FLUSH_RX			0xE2  // ��ϴ���� FIFOָ��
#define REUSE_TX_PL			0xE3  // �����ظ�װ������ָ��
#define R_RX_PL_WID			0x60  // ��ȡRx PayLoad���ȣ�+1byte
#define W_ACK_PAYLOAD		0xA8  // д���ACKһ���͵����ݰ���+1~32byte д������
#define W_TX_PAYLOAD_NOACK	0xB0  // ��ֹ�Դ˰����Զ�ACKӦ��+1~32byte д������
#define LOCK_UNLOCK			0x50  // ���ӹ��ܼĴ����޸�����
#define NOP					0xFF  // ����
//*********************SPI(nRF24L01P)�Ĵ�����ַ***************************
#define CONFIG				0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA				0x01  // �Զ�Ӧ��������
#define EN_RXADDR			0x02  // �����ŵ�����
#define SETUP_AW			0x03  // �շ���ַ�������
#define SETUP_RETR			0x04  // �Զ��ط���������
#define RF_CH				0x05  // ����Ƶ������
#define RF_SETUP			0x06  // �������ʡ����Ĺ�������
#define STATUS				0x07  // ״̬�Ĵ���
#define OBSERVE_TX			0x08  // ���ͼ�⹦��
#define CD					0x09  // ��ַ���
#define RX_ADDR_P0			0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1			0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2			0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3			0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4			0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5			0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR				0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0			0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1			0x12  // ����Ƶ��1�������ݳ���
#define RX_PW_P2			0x13  // ����Ƶ��2�������ݳ���
#define RX_PW_P3			0x14  // ����Ƶ��3�������ݳ���
#define RX_PW_P4			0x15  // ����Ƶ��4�������ݳ���
#define RX_PW_P5			0x16  // ����Ƶ��5�������ݳ���
#define FIFO_STATUS			0x17  // FIFOջ��ջ��״̬�Ĵ�������
#define DYNPD				0x1C  // ��̬PayLoad����ʹ��
#define FEATURE				0x1D  // �趨����Ĺ���
//*********************nRF24L01P PIPE��*********************************
#define NRF_PIPE0			0x00
#define NRF_PIPE1			0x01
#define NRF_PIPE2			0x02
#define NRF_PIPE3			0x03
#define NRF_PIPE4			0x04
#define NRF_PIPE5			0x05
//**********************************************************************
//*********************NRF24L01*****************************************
#define RX_DR				(0x01<<6)
#define TX_DS				(0x01<<5)
#define MAX_RT				(0x01<<4)
#define TX_FULL				(0x01<<0)

#define MODEL_ONLY_RX		1     //��ͨ����
#define MODEL_ONLY_TX		2     //��ͨ����
#define MODEL_PRIM_RX		3     //�����գ�����ACK��������
#define MODEL_PRIM_TX		4     //�����ͣ�����ACK��������

#define RX_PLOAD_WIDTH_MAX	32
#define TX_PLOAD_WIDTH_MAX	32
#define TX_ADR_WIDTH		5
#define RX_ADR_WIDTH		5

#define ACK_REQUEST				1
#define ACK_IGNORE				0
////////////////////////////////////////////////////////////////////////

#if USE_SPI_n == 1
#define NRF_IRQN_PORT		GPIOB
#define NRF_IRQN_PIN		GPIO_Pin_0
#else
#define NRF_IRQN_PORT		GPIOC
#define NRF_IRQN_PIN		GPIO_Pin_15
#endif
#define NRF_IRQN			(NRF_IRQN_PORT->IDR & NRF_IRQN_PIN)

extern uint8_t NRF_TX_BUF[];
extern uint8_t NRF_RX_BUF[];

uint8_t NRF24L01_Setup(uint8_t model, uint8_t channel);

uint8_t NRF_SPI_CheckConnection(void);// ��˵���ã���������
////////////////////////////////////////////////////////////////////////
uint8_t NRF_SendPacket(uint32_t ACK);// ����status | 0x80*�Ƿ�ɹ�����
uint8_t NRF_ReadPacket(void);		// ���ض�ȡ��PIPE��,���ɹ�����0xff
////////////////////////////////////////////////////////////////////////

#endif // __SPI_NRF24L01_H__
