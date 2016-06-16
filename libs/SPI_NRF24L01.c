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

#include "SPI_NRF24L01.h"

#if USE_SPI_n == 1
#define NRF_CE_PORT			GPIOB
#define NRF_CE_PIN			GPIO_Pin_2
#define NRF_CE_PORT_SPEED	GPIO_Speed_50MHz
#define NRF_IRQN_PORT_SPEED	GPIO_Speed_50MHz
#else
#define NRF_CE_PORT			GPIOC
#define NRF_CE_PIN			GPIO_Pin_14
#define NRF_CE_PORT_SPEED	GPIO_Speed_10MHz
#define NRF_IRQN_PORT_SPEED	GPIO_Speed_10MHz
#endif

#define NRF_CE_H			NRF_CE_PORT->BSRR |= NRF_CE_PIN
#define NRF_CE_L			NRF_CE_PORT->BSRR |= (NRF_CE_PIN<<16)

#define BUFFER_LEN		33

uint8_t __spi_buf[BUFFER_LEN];
uint8_t __model;

uint8_t NRF_TX_BUF[TX_PLOAD_WIDTH_MAX+1];
uint8_t NRF_RX_BUF[RX_PLOAD_WIDTH_MAX+1];

uint8_t NRF_SPI_WriteBuf(uint8_t reg, uint8_t *buffer, uint8_t len);
uint8_t NRF_SPI_ReadBuf(uint8_t reg, uint8_t *buffer, uint8_t len);
uint8_t NRF_SPI_WriteReg(uint8_t reg, uint8_t data);
uint8_t NRF_SPI_ReadReg(uint8_t reg);
void NRF_FlushBuffer(uint8_t flush_rx_tx);			// ���Rx/Tx����
uint8_t NRF_ResetEvent(uint8_t event_byte);

uint8_t NRF24L01_Setup(uint8_t model, uint8_t channel)
{
	GPIO_InitTypeDef GPIO_InitStructure;

#if USE_SPI_n == 1
	RCC->APB2ENR |= RCC_APB2Periph_GPIOB;
#else
	RCC->APB2ENR |= RCC_APB2Periph_GPIOC | RCC_APB2ENR_AFIOEN;
	PWR_BackupAccessCmd(ENABLE);//�����޸�RTC �ͺ󱸼Ĵ���
	RCC_LSEConfig(RCC_LSE_OFF);//�ر��ⲿ�����ⲿʱ���źŹ��� ��PC13 PC14 PC15 �ſ��Ե���ͨIO�á�
	PWR_BackupAccessCmd(DISABLE);//��ֹ�޸ĺ󱸼Ĵ���

#endif

	/*����NRF��CE����*/
	GPIO_InitStructure.GPIO_Pin = NRF_CE_PIN; //ce
	GPIO_InitStructure.GPIO_Speed = NRF_CE_PORT_SPEED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(NRF_CE_PORT, &GPIO_InitStructure);

	/*����NRF��IRQ����*/
	GPIO_InitStructure.GPIO_Pin = NRF_IRQN_PIN;
	GPIO_InitStructure.GPIO_Speed = NRF_IRQN_PORT_SPEED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ; //��������
	GPIO_Init(NRF_IRQN_PORT, &GPIO_InitStructure);

	NRF_CE_L;

	SPI_Setup();
	delay_ms(100);
	/////////////////////////////////////////////////////
	uint8_t TX_ADDRESS[TX_ADR_WIDTH]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//���ص�ַ
	uint8_t RX_ADDRESS[RX_ADR_WIDTH]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//���յ�ַ

	NRF_SPI_WriteReg(NRF_W | SETUP_AW,RX_ADR_WIDTH-2);				// �ڵ��ַ����5 bytes
	NRF_SPI_WriteBuf(NRF_W | RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	// дRX�ڵ��ַ
	NRF_SPI_WriteBuf(NRF_W | TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		// дTX�ڵ��ַ
	NRF_SPI_WriteReg(NRF_W | EN_AA,0x01); 							// ʹ��ͨ��0���Զ�Ӧ��
	if(NRF_SPI_ReadReg(EN_AA) != 0x01) return 0xFF;					// �ж��豸δ����
	NRF_SPI_WriteReg(NRF_W | EN_RXADDR,0x01);						// ʹ��ͨ��0�Ľ��յ�ַ
	NRF_SPI_WriteReg(NRF_W | SETUP_RETR,0x11);						// �����Զ��ط����ʱ��:500us;����Զ��ط�����1��
	NRF_SPI_WriteReg(NRF_W | RF_CH,channel);						// ����RFͨ��ΪCHANAL
	NRF_SPI_WriteReg(NRF_W | RF_SETUP,0x0e);						// ����TX�������,-6db����,2Mbps,���������濪��
	/////////////////////////////////////////////////////////
	NRF_FlushBuffer(FLUSH_RX|FLUSH_TX);
	if(model == MODEL_ONLY_RX){										// RX
		NRF_SPI_WriteReg(NRF_W | RX_PW_P0,RX_PLOAD_WIDTH_MAX);		// ѡ��ͨ��0����Ч���ݿ��
		NRF_SPI_WriteReg(NRF_W | CONFIG, 0x0f);						// IRQ�շ�����жϿ���,16λCRC,������
	}else if(model == MODEL_ONLY_TX){								// TX
		NRF_SPI_WriteReg(NRF_W | RX_PW_P0,RX_PLOAD_WIDTH_MAX);		// ѡ��ͨ��0����Ч���ݿ��
		NRF_SPI_WriteReg(NRF_W | CONFIG, 0x0e);						// IRQ�շ�����жϿ���,16λCRC,������
	}else if(model == MODEL_PRIM_RX){								// ��RX α˫��
		NRF_SPI_WriteReg(NRF_W | DYNPD,0x01);
		NRF_SPI_WriteReg(NRF_W | FEATURE,0x07);
		if(NRF_SPI_ReadReg(FEATURE) == 0x00){
			NRF_SPI_WriteReg(LOCK_UNLOCK,0x73);						// �򿪸��ӹ���
			NRF_SPI_WriteReg(NRF_W | DYNPD,0x01);
			NRF_SPI_WriteReg(NRF_W | FEATURE,0x07);
		}
		if(NRF_SPI_ReadReg(FEATURE) != 0x07) return 0xFE;			// �ж�ģʽ�Ƿ�֧��
		NRF_SPI_WriteReg(NRF_W | CONFIG, 0x0f);						// IRQ�շ�����жϿ���,16λCRC,������
	}else if(model == MODEL_PRIM_TX){								// ��TX α˫��
		NRF_SPI_WriteReg(NRF_W | DYNPD,0x01);
		NRF_SPI_WriteReg(NRF_W | FEATURE,0x07);
		if(NRF_SPI_ReadReg(FEATURE) == 0x00){
			NRF_SPI_WriteReg(LOCK_UNLOCK,0x73);						// �򿪸��ӹ���
			NRF_SPI_WriteReg(NRF_W | DYNPD,0x01);
			NRF_SPI_WriteReg(NRF_W | FEATURE,0x07);
		}
		if(NRF_SPI_ReadReg(FEATURE) != 0x07) return 0xFE;			// �ж�ģʽ�Ƿ�֧��
		NRF_SPI_WriteReg(NRF_W | CONFIG, 0x0e);						// IRQ�շ�����жϿ���,16λCRC,������
	}
	__model = model;
	NRF_CE_H;													// ��ʼ�շ�����
	delay_ms(3);
	return 0x00;
}

uint8_t NRF_SPI_WriteBuf(uint8_t reg, uint8_t *buffer, uint8_t len)
{
	if(len > BUFFER_LEN-1) {
		__BKPT(103);
		return -1;
	}
	while(SPI_isBusy());
	__spi_buf[0] = reg;
	for(int i = 0 ; i < len ; i++) __spi_buf[i+1] = buffer[i];

	SPI_RWData(__spi_buf,len+1);
	while(SPI_isBusy());
	return __spi_buf[0];
}
uint8_t NRF_SPI_ReadBuf(uint8_t reg, uint8_t *buffer, uint8_t len)
{
	if(len > BUFFER_LEN - 1) {
		__BKPT(104);
		return -1;
	}
	while(SPI_isBusy());
	__spi_buf[0] = reg;
	for(int i = 0 ; i < len ; i++) __spi_buf[i+1] = 0;
	SPI_RWData(__spi_buf,len+1);
	while(SPI_isBusy());
	for(int i = 0 ; i < len ; i++) buffer[i] = __spi_buf[i+1];
	return __spi_buf[0];
}
uint8_t NRF_SPI_WriteReg(uint8_t reg, uint8_t data)
{
	uint8_t tmp = data;
	return NRF_SPI_WriteBuf(reg,&tmp,1);
}
uint8_t NRF_SPI_ReadReg(uint8_t reg)
{
	uint8_t tmp = 0;
	NRF_SPI_ReadBuf(reg,&tmp,1);
	return tmp;
}
uint8_t NRF_SPI_CheckConnection(void)
{
	uint8_t buffer[5] = {1,2,3,4,5};
	NRF_SPI_WriteBuf(NRF_W | RX_ADDR_P1,buffer,5);
	buffer[4] = buffer[3] = buffer[2] = buffer[1] = buffer[0] = 0;
	NRF_SPI_ReadBuf(RX_ADDR_P1,buffer,5);
	if(buffer[0] == 1 && buffer[1] == 2 && buffer[2] == 3 && buffer[3] == 4 && buffer[4] == 5) return 1;
	else return 0;
}

uint8_t NRF_SendPacket(uint32_t ACK)
{
	uint8_t status = NRF_ResetEvent(MAX_RT | TX_DS);
	if(status & MAX_RT){
		NRF_FlushBuffer(FLUSH_TX);
		NRF_ResetEvent(MAX_RT);
	}else if(status & TX_FULL) return status;
	switch(__model){
	case MODEL_PRIM_RX:
		NRF_SPI_WriteBuf(W_ACK_PAYLOAD,NRF_TX_BUF,NRF_TX_BUF[TX_PLOAD_WIDTH_MAX]);	// װ������
		break;
	case MODEL_PRIM_TX:
		if(ACK)
			NRF_SPI_WriteBuf(W_TX_PLOAD,NRF_TX_BUF,NRF_TX_BUF[TX_PLOAD_WIDTH_MAX]);// װ������
		else
			NRF_SPI_WriteBuf(W_TX_PAYLOAD_NOACK,NRF_TX_BUF,NRF_TX_BUF[TX_PLOAD_WIDTH_MAX]);	// װ������
		break;
	case MODEL_ONLY_TX:
		if(ACK)
			NRF_SPI_WriteBuf(W_TX_PLOAD,NRF_TX_BUF,TX_PLOAD_WIDTH_MAX);				// װ������
		else
			NRF_SPI_WriteBuf(W_TX_PAYLOAD_NOACK,NRF_TX_BUF,TX_PLOAD_WIDTH_MAX);		// װ������
		break;
	default:
		return status;
	}
	return status|0x80;
}
uint8_t NRF_ReadPacket(void)
{
	uint8_t status = NRF_ResetEvent(RX_DR);
	if(status & RX_DR){
		uint8_t len = NRF_SPI_ReadReg(R_RX_PL_WID);
		if(len > RX_PLOAD_WIDTH_MAX){
			NRF_FlushBuffer(FLUSH_RX);
			return 0xFF;
		}else{
			NRF_SPI_ReadBuf(R_RX_PLOAD,NRF_RX_BUF,len);
			NRF_RX_BUF[RX_PLOAD_WIDTH_MAX] = len;
		}
		return (status >> 1) & 0x07;
	}else return 0xFF;
}

void NRF_FlushBuffer(uint8_t flush_rx_tx){
	if((flush_rx_tx & FLUSH_TX) == FLUSH_TX) NRF_SPI_WriteBuf(FLUSH_TX,0,0);		// ���TXFIFO
	if((flush_rx_tx & FLUSH_RX) == FLUSH_RX) NRF_SPI_WriteBuf(FLUSH_RX,0,0);		// ���RXFIFO
}
inline uint8_t NRF_ResetEvent(uint8_t event_byte)
{
	return NRF_SPI_WriteReg(NRF_W | STATUS,event_byte&0x70);
}
