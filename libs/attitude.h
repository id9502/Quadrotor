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

#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include "quater.h"
#include "MPU6050.h"
#include "sys.h"

extern float_vect3 ACC_v3f,GYRO_v3f;		//����һ��6���ȡֵ
extern quater attitude_Q;					//��̬��Ԫ������
extern float_vect3 attitude_acceleration;	//��Ե�����ٶ���Ϣ
extern float_vect3 attitude_velocity;		//��Ե�������ٶ���Ϣ
extern float_vect3 attitude_position;		//��Ե������λ����Ϣ
extern float_vect3 attitude_gravity;		//��ʼ���궨��������������mpu6050��ʼ����������

void attitudeUpdate(float dt);

void attitudeAdjust();

#endif // __ATTITUDE_H__
