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

//注意！使用了DMA的SPI传输不能在中断中使用，所有调用必须放到main执行级里
#ifndef __SPI_NRF24L01_H__
#define __SPI_NRF24L01_H__

#include "stm32f10x_conf.h"

#include "SPI_Lib.h"
#include "sys.h"
#include "../diag/Trace.h"

//*********************NRF24L01P寄存器指令*******************************
#define NRF_R				0x00  // 读寄存器指令，+寄存器地址
#define NRF_W				0x20  // 写寄存器指令，+寄存器地址
#define R_RX_PLOAD			0x61  // 读取接收数据指令，+1~32byte 写入数据
#define W_TX_PLOAD			0xA0  // 写待发数据指令，+1~32byte 接收数据
#define FLUSH_TX			0xE1  // 冲洗发送 FIFO指令
#define FLUSH_RX			0xE2  // 冲洗接收 FIFO指令
#define REUSE_TX_PL			0xE3  // 定义重复装载数据指令
#define R_RX_PL_WID			0x60  // 读取Rx PayLoad长度，+1byte
#define W_ACK_PAYLOAD		0xA8  // 写入和ACK一起发送的数据包，+1~32byte 写入数据
#define W_TX_PAYLOAD_NOACK	0xB0  // 禁止对此包的自动ACK应答，+1~32byte 写入数据
#define LOCK_UNLOCK			0x50  // 附加功能寄存器修改锁定
#define NOP					0xFF  // 保留
//*********************SPI(nRF24L01P)寄存器地址***************************
#define CONFIG				0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA				0x01  // 自动应答功能设置
#define EN_RXADDR			0x02  // 可用信道设置
#define SETUP_AW			0x03  // 收发地址宽度设置
#define SETUP_RETR			0x04  // 自动重发功能设置
#define RF_CH				0x05  // 工作频率设置
#define RF_SETUP			0x06  // 发射速率、功耗功能设置
#define STATUS				0x07  // 状态寄存器
#define OBSERVE_TX			0x08  // 发送监测功能
#define CD					0x09  // 地址检测
#define RX_ADDR_P0			0x0A  // 频道0接收数据地址
#define RX_ADDR_P1			0x0B  // 频道1接收数据地址
#define RX_ADDR_P2			0x0C  // 频道2接收数据地址
#define RX_ADDR_P3			0x0D  // 频道3接收数据地址
#define RX_ADDR_P4			0x0E  // 频道4接收数据地址
#define RX_ADDR_P5			0x0F  // 频道5接收数据地址
#define TX_ADDR				0x10  // 发送地址寄存器
#define RX_PW_P0			0x11  // 接收频道0接收数据长度
#define RX_PW_P1			0x12  // 接收频道1接收数据长度
#define RX_PW_P2			0x13  // 接收频道2接收数据长度
#define RX_PW_P3			0x14  // 接收频道3接收数据长度
#define RX_PW_P4			0x15  // 接收频道4接收数据长度
#define RX_PW_P5			0x16  // 接收频道5接收数据长度
#define FIFO_STATUS			0x17  // FIFO栈入栈出状态寄存器设置
#define DYNPD				0x1C  // 动态PayLoad长度使能
#define FEATURE				0x1D  // 设定额外的功能
//*********************nRF24L01P PIPE号*********************************
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

#define MODEL_ONLY_RX		1     //普通接收
#define MODEL_ONLY_TX		2     //普通发送
#define MODEL_PRIM_RX		3     //主接收，借助ACK发送数据
#define MODEL_PRIM_TX		4     //主发送，借助ACK接收数据

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

uint8_t NRF_SPI_CheckConnection(void);// 虽说能用，但已弃用
////////////////////////////////////////////////////////////////////////
uint8_t NRF_SendPacket(uint32_t ACK);// 返回status | 0x80*是否成功发送
uint8_t NRF_ReadPacket(void);		// 返回读取的PIPE号,不成功返回0xff
////////////////////////////////////////////////////////////////////////

#endif // __SPI_NRF24L01_H__
