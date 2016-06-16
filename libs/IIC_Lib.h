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

#ifndef __IIC_LIB__
#define __IIC_LIB__

#include "stm32f10x_conf.h"

//#define __IIC_LIB_USE_SOFTWARE_IIC__
#define __IIC_LIB_USE_INTERRUPT_IIC__
//#define __IIC_LIB_USE_DMA_IIC__


void I2C_Setup(void);
uint8_t I2C_WriteData(uint8_t dev_address, uint8_t address, uint8_t *buffer, uint16_t data_len);
uint8_t I2C_ReadData(uint8_t dev_address, uint8_t address, uint8_t *buffer, uint16_t data_len);
uint8_t I2C_isBusy(void);

#ifdef __IIC_LIB_USE_INTERRUPT_IIC__
void I2C2_EV_IRQHandler(void);
#endif

#endif // __IIC_LIB__
