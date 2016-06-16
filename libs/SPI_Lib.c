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

#include "SPI_Lib.h"

#if USE_SPI_n == 1
#define SPIn				SPI1
#define SPI_PIN				GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7
#define SPI_PORT			GPIOA
#define SPI_CSN_PIN			GPIO_Pin_4
#define SPI_CSN_PORT		GPIOA
#define SPI_CSN_PORT_SPEED	GPIO_Speed_50MHz
#define SPI_DMA_RX			DMA1_Channel2
#define SPI_DMA_TX			DMA1_Channel3
#define SPI_DMA_RX_IRQn		DMA1_Channel2_IRQn
#define SPI_DMA_TX_IRQn		DMA1_Channel3_IRQn
#define SPI_DMA_RX_IT_TC	DMA1_IT_TC2
#define SPI_DMA_RX_IT_GL	DMA1_IT_GL2
#define SPI_DMA_TX_IT_TC	DMA1_IT_TC3
#define SPI_DMA_TX_IT_GL	DMA1_IT_GL3
#else
#define SPIn				SPI2
#define SPI_PIN				GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15
#define SPI_PORT			GPIOB
#define SPI_CSN_PIN			GPIO_Pin_13
#define SPI_CSN_PORT		GPIOC
#define SPI_CSN_PORT_SPEED	GPIO_Speed_2MHz
#define SPI_DMA_RX			DMA1_Channel4
#define SPI_DMA_TX			DMA1_Channel5
#define SPI_DMA_RX_IRQn		DMA1_Channel4_IRQn
#define SPI_DMA_TX_IRQn		DMA1_Channel5_IRQn
#define SPI_DMA_RX_IT_TC	DMA1_IT_TC4
#define SPI_DMA_RX_IT_GL	DMA1_IT_GL4
#define SPI_DMA_TX_IT_TC	DMA1_IT_TC5
#define SPI_DMA_TX_IT_GL	DMA1_IT_GL5
#endif

#define SPI_CSN_H			SPI_CSN_PORT->BSRR |= SPI_CSN_PIN
#define SPI_CSN_L			SPI_CSN_PORT->BSRR |= (SPI_CSN_PIN<<16)

uint8_t	SPI_Tx_Busy = 0;
uint8_t	SPI_Rx_Busy = 0;


void SPI_Setup(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
#if USE_SPI_n == 1
	RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1;
#else
	RCC->APB2ENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB;
	RCC->APB1ENR |= RCC_APB1Periph_SPI2;
#endif
	RCC->AHBENR |= RCC_AHBPeriph_DMA1;

	/*配置 SPI_NRF_SPI的 SCK,MISO,MOSI引脚，GPIOA^5,6,7 或GPIOB^13,14,15*/
	GPIO_InitStructure.GPIO_Pin = SPI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用功能
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_CSN_PIN; //csn
	GPIO_InitStructure.GPIO_Speed = SPI_CSN_PORT_SPEED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_CSN_PORT, &GPIO_InitStructure);

	SPI_CSN_H;
	SPI_I2S_DeInit(SPIn);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //数据大小8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //时钟极性，空闲时为低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第1个边沿有效，上升沿为采样时刻
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS信号由软件产生
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8分频，9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPIn, &SPI_InitStructure);

	/* DMA1 Channel2 (triggered by SPI1 Rx event) Config */
	DMA_DeInit(SPI_DMA_RX);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPIn->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&(SPIn->DR));					//放一个假地址，防止库函数报错
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(SPI_DMA_RX, &DMA_InitStructure);

	DMA_ITConfig(SPI_DMA_RX, DMA_IT_TC, ENABLE);
	DMA_Cmd(SPI_DMA_RX, DISABLE);

	/* DMA1 Channel3 (triggered by SPI1 Tx event) Config */
	DMA_DeInit(SPI_DMA_TX);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPIn->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&(SPIn->DR));					//放一个假地址，防止库函数报错
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(SPI_DMA_TX, &DMA_InitStructure);

	DMA_ITConfig(SPI_DMA_TX, DMA_IT_TC, ENABLE);
	DMA_Cmd(SPI_DMA_TX, DISABLE);

	/* Enable the SPI RX DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = SPI_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the SPI TX DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = SPI_DMA_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable SPIn DMA RX/TX request */
	SPI_I2S_DMACmd(SPIn,SPI_I2S_DMAReq_Rx|SPI_I2S_DMAReq_Tx,ENABLE);
	/* Enable SPIn */
	SPI_Cmd(SPIn, DISABLE);

	SPI_Rx_Busy = SPI_Tx_Busy = 0;
}
void SPI_RWData(uint8_t *buf, uint16_t len)
{
	SPI_Tx_Busy = SPI_Rx_Busy = 1;
	DMA_Cmd(SPI_DMA_RX,DISABLE);
	DMA_Cmd(SPI_DMA_TX,DISABLE);
	SPI_DMA_RX->CMAR = SPI_DMA_TX->CMAR = (uint32_t)buf;
	SPI_DMA_RX->CNDTR = SPI_DMA_TX->CNDTR = len;
	SPI_CSN_L;									// 选通器件
#if USE_SPI_n == 2
	for(volatile int i = 0 ; i < 10 ; i++);
#endif
	SPI_Cmd(SPIn,ENABLE);
	DMA_Cmd(SPI_DMA_RX,ENABLE);
	DMA_Cmd(SPI_DMA_TX,ENABLE);
}
inline int SPI_isBusy(void)
{
	return SPI_Tx_Busy | SPI_Rx_Busy | SPI_I2S_GetFlagStatus(SPIn,SPI_I2S_FLAG_BSY);
}
void SPI_DMA_RX_IRQ()
{
	if(DMA_GetITStatus(SPI_DMA_RX_IT_TC))
	{
		DMA_ClearITPendingBit(SPI_DMA_RX_IT_GL);    //清除全部中断标志
		DMA_Cmd(SPI_DMA_RX,DISABLE);
		while(SPI_I2S_GetFlagStatus(SPIn,SPI_I2S_FLAG_BSY));
		SPI_Cmd(SPIn,DISABLE);
		#if USE_SPI_n == 2
			for(volatile int i = 0 ; i < 20 ; i++);
		#endif
		SPI_CSN_H;									// 禁用器件
		SPI_Rx_Busy = 0;
	}
}
void SPI_DMA_TX_IRQ()
{
	if(DMA_GetITStatus(SPI_DMA_TX_IT_TC))
	{
		DMA_ClearITPendingBit(SPI_DMA_TX_IT_GL);    //清除全部中断标志
		DMA_Cmd(SPI_DMA_TX,DISABLE);
		SPI_Tx_Busy = 0;
	}
}
